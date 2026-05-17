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

typedef enum
{
  FOC_DIAG_ANGLE_LEGACY = 0,
  FOC_DIAG_ANGLE_RAW_POS,
  FOC_DIAG_ANGLE_RAW_NEG,
  FOC_DIAG_ANGLE_COUNT
} foc_diag_angle_source_t;

static volatile bool foc_diag_active;
static volatile foc_diag_angle_source_t foc_diag_angle_source[2] = {
  FOC_DIAG_ANGLE_RAW_NEG,
  FOC_DIAG_ANGLE_RAW_NEG
};
static volatile float foc_diag_torque_sign[2] = {
  FOC_DIAG_DEFAULT_TORQUE_SIGN,
  FOC_DIAG_DEFAULT_TORQUE_SIGN
};
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

static const char * focDiagAngleSourceName(foc_diag_angle_source_t source)
{
  switch (source) {
    case FOC_DIAG_ANGLE_LEGACY:
      return "legacy";
    case FOC_DIAG_ANGLE_RAW_POS:
      return "raw+";
    case FOC_DIAG_ANGLE_RAW_NEG:
      return "raw-";
    default:
      return "unknown";
  }
}

static inline float rawElectricalAnglePositive(const as5047p_t * enc)
{
  return (float)(enc->enc_raw % FOC_DIAG_ELEC_CNT) * FOC_DIAG_ELEC_RAW_TO_RAD;
}

static inline float rawElectricalAngleNegative(const as5047p_t * enc)
{
  return 2.0f * (float)M_PI - rawElectricalAnglePositive(enc);
}

static inline float selectFocDiagnosticElectricalAngle(bool motor)
{
  float sensor_angle;
  const uint8_t motor_idx = motor ? 1U : 0U;
  const foc_diag_angle_source_t source = foc_diag_angle_source[motor_idx];

  if (source == FOC_DIAG_ANGLE_RAW_POS) {
    sensor_angle = rawElectricalAnglePositive(&as5047p[motor_idx]);
  } else if (source == FOC_DIAG_ANGLE_RAW_NEG) {
    sensor_angle = rawElectricalAngleNegative(&as5047p[motor_idx]);
  } else {
    sensor_angle = as5047p[motor_idx].output_radian;
  }

  return sensor_angle + enc_offset[motor_idx].zero_calib + sys.manual_offset_radian;
}

static inline float focDiagnosticPhaseAdvance(float speed_rps)
{
  const float model = FOC_DIAG_PHASE_ADVANCE_MODEL_RAD_PER_RPS * fabsf(speed_rps);
  return clampFocDiagPhaseAdvance(model + foc_diag_phase_advance_trim_rad);
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
  p("\nFOC diag start limit %+4.1fV M0 %s/%+3.0f M1 %s/%+3.0f adv %+4.1fdeg, w/s cmd, B angle, T tq, [/]/P phase, V stop\n",
    FOC_DIAG_VOLTAGE_LIMIT,
    focDiagAngleSourceName(foc_diag_angle_source[0]),
    foc_diag_torque_sign[0],
    focDiagAngleSourceName(foc_diag_angle_source[1]),
    foc_diag_torque_sign[1],
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

void cycleFocDiagnosticAngleSource(void)
{
  cycleFocDiagnosticAngleSourceMotor(false);
  cycleFocDiagnosticAngleSourceMotor(true);
}

void cycleFocDiagnosticAngleSourceMotor(bool motor)
{
  const uint8_t motor_idx = motor ? 1U : 0U;
  foc_diag_angle_source[motor_idx] =
    (foc_diag_angle_source_t)(((int)foc_diag_angle_source[motor_idx] + 1) % (int)FOC_DIAG_ANGLE_COUNT);
  p("\nFOC diag M%u angle %s\n", motor_idx, focDiagAngleSourceName(foc_diag_angle_source[motor_idx]));
}

void toggleFocDiagnosticTorqueSign(void)
{
  toggleFocDiagnosticTorqueSignMotor(false);
  toggleFocDiagnosticTorqueSignMotor(true);
}

void toggleFocDiagnosticTorqueSignMotor(bool motor)
{
  const uint8_t motor_idx = motor ? 1U : 0U;
  foc_diag_torque_sign[motor_idx] = -foc_diag_torque_sign[motor_idx];
  p("\nFOC diag M%u torque sign %+3.0f\n", motor_idx, foc_diag_torque_sign[motor_idx]);
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
  const float voltage_q = foc_diag_torque_sign[motor_idx] * cmd[motor_idx].out_v_final;
  if (voltage_q == 0.0f) {
    foc_pwm_compare_t center = {TIM_PWM_CENTER, TIM_PWM_CENTER, TIM_PWM_CENTER, false};
    focDriverSetPwmCompare(motor, center);
    return;
  }

  float electrical = selectFocDiagnosticElectricalAngle(motor);
  const float phase_advance = focDiagnosticPhaseAdvance(cmd[motor_idx].speed);
  electrical += (cmd[motor_idx].speed < 0.0f) ? -phase_advance : phase_advance;
  focDriverApplySineVoltage(motor, voltage_q, 0.0f, electrical, getBatteryVoltage());
}
