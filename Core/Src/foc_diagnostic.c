/**
 * @file foc_diagnostic.c
 * @brief Runs a low-voltage FOC/SinePWM diagnostic path without replacing legacy RUN.
 */

#include "foc_diagnostic.h"

#include <math.h>
#include <stdint.h>

#include "adc.h"
#include "app_context.h"
#include "diagnostics.h"
#include "foc_driver_hal.h"
#include "foc_math.h"
#include "gpio.h"
#include "motor.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#define FOC_DIAG_VOLTAGE_LIMIT (2.0f)
// Legacy zero_calib is compatible with raw- sensor angle plus zero offset.
#define FOC_DIAG_DEFAULT_TORQUE_SIGN (-1.0f)
#define FOC_DIAG_FREEWHEEL_CNT_ON_STOP (60000U)
#define FOC_DIAG_ELEC_CNT (5461)
#define FOC_DIAG_ELEC_RAW_TO_RAD (2.0f * (float)M_PI / (float)FOC_DIAG_ELEC_CNT)
#define FOC_DIAG_PHASE_ADVANCE_MODEL_RAD_PER_RPS (-0.003457f)
#define FOC_DIAG_PHASE_ADVANCE_STEP_RAD (0.01745329252f)
#define FOC_DIAG_PHASE_ADVANCE_LIMIT_RAD (0.78539816339f)
#define FOC_DIAG_RAD_TO_DEG (57.2957795131f)
#define FOC_DIAG_AXIS_OFFSET_RAD ((float)M_PI * 0.5f)

typedef struct
{
  int raw;
  float legacy_output_radian;
  float zero_calib;
  float cmd_speed;
  float real_rps;
  float out_v_final;
} foc_diag_angle_snapshot_t;

static volatile bool foc_diag_active;
static volatile float foc_diag_phase_advance_trim_rad;

static inline float clampFocDiagVoltage(float voltage)
{
  if (voltage > FOC_DIAG_VOLTAGE_LIMIT) {
    return FOC_DIAG_VOLTAGE_LIMIT;
  }
  if (voltage < -FOC_DIAG_VOLTAGE_LIMIT) {
    return -FOC_DIAG_VOLTAGE_LIMIT;
  }
  return voltage;
}

static inline float clampFocDiagPhaseAdvance(float angle)
{
  if (angle > FOC_DIAG_PHASE_ADVANCE_LIMIT_RAD) {
    return FOC_DIAG_PHASE_ADVANCE_LIMIT_RAD;
  }
  if (angle < -FOC_DIAG_PHASE_ADVANCE_LIMIT_RAD) {
    return -FOC_DIAG_PHASE_ADVANCE_LIMIT_RAD;
  }
  return angle;
}

bool isFocDiagnosticActive(void)
{
  return foc_diag_active;
}

static inline float rawElectricalAnglePositive(const as5047p_t * enc)
{
  return (float)(enc->enc_raw % FOC_DIAG_ELEC_CNT) * FOC_DIAG_ELEC_RAW_TO_RAD;
}

static inline float rawElectricalAngleNegative(const as5047p_t * enc)
{
  return focNormalizeAngle(-rawElectricalAnglePositive(enc));
}

static inline float selectFocDiagnosticElectricalAngleIndex(uint8_t motor_idx)
{
  const float sensor_angle = rawElectricalAngleNegative(&as5047p[motor_idx]);
  return focNormalizeAngle(sensor_angle + enc_offset[motor_idx].zero_calib + sys.manual_offset_radian);
}

static inline float selectFocDiagnosticElectricalAngle(bool motor)
{
  return selectFocDiagnosticElectricalAngleIndex(motor ? 1U : 0U);
}

static inline float focDiagnosticPhaseAdvance(float speed_rps)
{
  const float model = FOC_DIAG_PHASE_ADVANCE_MODEL_RAD_PER_RPS * fabsf(speed_rps);
  return clampFocDiagPhaseAdvance(model + foc_diag_phase_advance_trim_rad);
}

static inline float focDiagnosticTorqueAxisOffset(float output_voltage)
{
  (void)output_voltage;
  return FOC_DIAG_AXIS_OFFSET_RAD;
}

void startFocDiagnosticMode(void)
{
  foc_diag_active = true;
  sys.free_wheel_cnt = 0U;
  sys.zero_output_sleep_cnt = 0U;
  cmd[0].speed = 0.0f;
  cmd[1].speed = 0.0f;
  cmd[0].out_v = 0.0f;
  cmd[1].out_v = 0.0f;
  cmd[0].out_v_final = 0.0f;
  cmd[1].out_v_final = 0.0f;
  cmd[0].timeout_cnt = -1;
  cmd[1].timeout_cnt = -1;
  setPwmAll(TIM_PWM_CENTER);
  resumePwmOutput();
  p("\nFOC diag start limit %+4.1fV angle raw-+zero+pi/2 tq %+3.0f adv %+4.1fdeg, w/s cmd, v angle, [/]/P phase, V stop\n",
    FOC_DIAG_VOLTAGE_LIMIT,
    FOC_DIAG_DEFAULT_TORQUE_SIGN,
    foc_diag_phase_advance_trim_rad * FOC_DIAG_RAD_TO_DEG);
}

void stopFocDiagnosticMode(void)
{
  foc_diag_active = false;
  cmd[0].speed = 0.0f;
  cmd[1].speed = 0.0f;
  cmd[0].out_v = 0.0f;
  cmd[1].out_v = 0.0f;
  cmd[0].out_v_final = 0.0f;
  cmd[1].out_v_final = 0.0f;
  sys.free_wheel_cnt = FOC_DIAG_FREEWHEEL_CNT_ON_STOP;
  setPwmAll(TIM_PWM_CENTER);
  setPwmOutPutFreeWheel();
  p("\nFOC diag stop, freewheel\n");
}

void toggleFocDiagnosticMode(void)
{
  if (foc_diag_active) {
    stopFocDiagnosticMode();
  } else {
    startFocDiagnosticMode();
  }
}

void adjustFocDiagnosticPhaseAdvance(float delta_rad)
{
  foc_diag_phase_advance_trim_rad = clampFocDiagPhaseAdvance(foc_diag_phase_advance_trim_rad + delta_rad);
  p("\nFOC diag phase trim %+5.1fdeg model %+6.4frad/rps\n",
    foc_diag_phase_advance_trim_rad * FOC_DIAG_RAD_TO_DEG,
    FOC_DIAG_PHASE_ADVANCE_MODEL_RAD_PER_RPS);
}

void resetFocDiagnosticPhaseAdvance(void)
{
  foc_diag_phase_advance_trim_rad = 0.0f;
  p("\nFOC diag phase trim %+5.1fdeg model %+6.4frad/rps\n",
    foc_diag_phase_advance_trim_rad * FOC_DIAG_RAD_TO_DEG,
    FOC_DIAG_PHASE_ADVANCE_MODEL_RAD_PER_RPS);
}

void printFocDiagnosticAngleState(void)
{
  foc_diag_angle_snapshot_t snapshot[2];

  __disable_irq();
  for (uint8_t i = 0U; i < 2U; i++) {
    snapshot[i].raw = as5047p[i].enc_raw;
    snapshot[i].legacy_output_radian = as5047p[i].output_radian;
    snapshot[i].zero_calib = enc_offset[i].zero_calib;
    snapshot[i].cmd_speed = cmd[i].speed;
    snapshot[i].real_rps = motor_real[i].rps;
    snapshot[i].out_v_final = cmd[i].out_v_final;
  }
  __enable_irq();

  p("\n[FOC ANG] convention raw- + zero + manual + pi/2, tq %+3.0f, trim %+5.1fdeg\n",
    FOC_DIAG_DEFAULT_TORQUE_SIGN,
    foc_diag_phase_advance_trim_rad * FOC_DIAG_RAD_TO_DEG);

  for (uint8_t i = 0U; i < 2U; i++) {
    const as5047p_t enc = {.enc_raw = snapshot[i].raw};
    const float raw_pos = rawElectricalAnglePositive(&enc);
    const float raw_neg = rawElectricalAngleNegative(&enc);
    const float foc_base = focNormalizeAngle(raw_neg + snapshot[i].zero_calib + sys.manual_offset_radian);
    const float axis_offset = focDiagnosticTorqueAxisOffset(snapshot[i].out_v_final);
    const float foc_axis = focNormalizeAngle(foc_base + axis_offset);
    const float phase_model = focDiagnosticPhaseAdvance(snapshot[i].cmd_speed);
    const float phase_used = (snapshot[i].cmd_speed < 0.0f) ? -phase_model : phase_model;
    const float foc_used = focNormalizeAngle(foc_axis + phase_used);
    const float legacy = focNormalizeAngle(snapshot[i].legacy_output_radian + snapshot[i].zero_calib + sys.manual_offset_radian);
    const float voltage_q = FOC_DIAG_DEFAULT_TORQUE_SIGN * snapshot[i].out_v_final;

    p("[FOC ANG] M%u raw %5d raw+ %+6.3f raw- %+6.3f zero %+6.3f legacy %+6.3f foc %+6.3f axis %+5.2f used %+6.3f adv %+5.1fdeg cmd %+6.1f rps %+6.1f uq %+4.1f\n",
      i,
      snapshot[i].raw,
      raw_pos,
      raw_neg,
      snapshot[i].zero_calib,
      legacy,
      foc_base,
      axis_offset,
      foc_used,
      phase_used * FOC_DIAG_RAD_TO_DEG,
      snapshot[i].cmd_speed,
      snapshot[i].real_rps,
      voltage_q);
  }
}

void focDiagnosticMode(void)
{
  if (sys.manual_offset_radian > M_PI * 2) {
    sys.manual_offset_radian = 0.0f;
  }

  for (int i = 0; i < 2; i++) {
    if (isPushedSW1()) {
      cmd[i].speed = 10.0f;
    } else if (isPushedSW2()) {
      cmd[i].speed = -10.0f;
    }

    speedToOutputVoltage(&pid[i], &motor_real[i], &motor_param[i], &cmd[i]);
    cmd[i].out_v = clampFocDiagVoltage(cmd[i].out_v);
    cmd[i].out_v_final = cmd[i].out_v;
  }

  printRuntimeDiagnostics();
}

void focDiagnosticProcess_itr(bool motor)
{
  updateADC(motor);
  updateAS5047P(motor);

  const uint8_t motor_idx = motor ? 1U : 0U;
  const float voltage_q = FOC_DIAG_DEFAULT_TORQUE_SIGN * cmd[motor_idx].out_v_final;
  if (voltage_q == 0.0f) {
    foc_pwm_compare_t center = {TIM_PWM_CENTER, TIM_PWM_CENTER, TIM_PWM_CENTER, false};
    focDriverSetPwmCompare(motor, center);
    return;
  }

  float electrical = selectFocDiagnosticElectricalAngle(motor);
  electrical += focDiagnosticTorqueAxisOffset(cmd[motor_idx].out_v_final);
  const float phase_advance = focDiagnosticPhaseAdvance(cmd[motor_idx].speed);
  electrical += (cmd[motor_idx].speed < 0.0f) ? -phase_advance : phase_advance;
  focDriverApplySineVoltage(motor, voltage_q, 0.0f, electrical, getBatteryVoltage());
}
