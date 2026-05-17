/**
 * @file foc_diagnostic.c
 * @brief Runs a low-voltage FOC/SinePWM diagnostic path without replacing legacy RUN.
 */

#include "foc_diagnostic.h"

#include <math.h>

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
#define FOC_DIAG_DEFAULT_TORQUE_SIGN (-1.0f)
#define FOC_DIAG_FREEWHEEL_CNT_ON_STOP (60000U)
#define FOC_DIAG_ELEC_CNT (5461)
#define FOC_DIAG_ELEC_RAW_TO_RAD (2.0f * (float)M_PI / (float)FOC_DIAG_ELEC_CNT)

typedef enum
{
  FOC_DIAG_ANGLE_LEGACY = 0,
  FOC_DIAG_ANGLE_RAW_POS,
  FOC_DIAG_ANGLE_RAW_NEG,
  FOC_DIAG_ANGLE_COUNT
} foc_diag_angle_source_t;

static volatile bool foc_diag_active;
static volatile foc_diag_angle_source_t foc_diag_angle_source = FOC_DIAG_ANGLE_LEGACY;
static volatile float foc_diag_torque_sign = FOC_DIAG_DEFAULT_TORQUE_SIGN;

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
  const foc_diag_angle_source_t source = foc_diag_angle_source;

  if (source == FOC_DIAG_ANGLE_RAW_POS) {
    sensor_angle = rawElectricalAnglePositive(&as5047p[motor]);
  } else if (source == FOC_DIAG_ANGLE_RAW_NEG) {
    sensor_angle = rawElectricalAngleNegative(&as5047p[motor]);
  } else {
    sensor_angle = as5047p[motor].output_radian;
  }

  return sensor_angle + enc_offset[motor].zero_calib + sys.manual_offset_radian;
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
  p("\nFOC diag start limit %+4.1fV angle %s tq %+3.0f, w/s cmd, B angle, T torque, V stop\n",
    FOC_DIAG_VOLTAGE_LIMIT,
    focDiagAngleSourceName(foc_diag_angle_source),
    foc_diag_torque_sign);
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
  foc_diag_angle_source = (foc_diag_angle_source_t)(((int)foc_diag_angle_source + 1) % (int)FOC_DIAG_ANGLE_COUNT);
  p("\nFOC diag angle %s\n", focDiagAngleSourceName(foc_diag_angle_source));
}

void toggleFocDiagnosticTorqueSign(void)
{
  foc_diag_torque_sign = -foc_diag_torque_sign;
  p("\nFOC diag torque sign %+3.0f\n", foc_diag_torque_sign);
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

  const float voltage_q = foc_diag_torque_sign * cmd[motor].out_v_final;
  if (voltage_q == 0.0f) {
    foc_pwm_compare_t center = {TIM_PWM_CENTER, TIM_PWM_CENTER, TIM_PWM_CENTER, false};
    focDriverSetPwmCompare(motor, center);
    return;
  }

  const float electrical = selectFocDiagnosticElectricalAngle(motor);
  focDriverApplySineVoltage(motor, voltage_q, 0.0f, electrical, getBatteryVoltage());
}
