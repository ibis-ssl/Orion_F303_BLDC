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
#define FOC_DIAG_SENSOR_TORQUE_DIRECTION (-1.0f)
#define FOC_DIAG_FREEWHEEL_CNT_ON_STOP (60000U)

static volatile bool foc_diag_active;

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
  p("\nFOC diag start limit %+4.1fV, use w/s for command, V to stop\n", FOC_DIAG_VOLTAGE_LIMIT);
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

  const float voltage_q = FOC_DIAG_SENSOR_TORQUE_DIRECTION * cmd[motor].out_v_final;
  if (voltage_q == 0.0f) {
    foc_pwm_compare_t center = {TIM_PWM_CENTER, TIM_PWM_CENTER, TIM_PWM_CENTER, false};
    focDriverSetPwmCompare(motor, center);
    return;
  }

  const float electrical = as5047p[motor].output_radian + enc_offset[motor].zero_calib + sys.manual_offset_radian;
  focDriverApplySineVoltage(motor, voltage_q, 0.0f, electrical, getBatteryVoltage());
}
