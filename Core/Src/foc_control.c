/**
 * @file foc_control.c
 * @brief Shared voltage-FOC output helpers for RUN, diagnostics, and calibration.
 */

#include "foc_control.h"

#include <math.h>
#include <stdint.h>

#include "adc.h"
#include "app_context.h"
#include "foc_driver_hal.h"
#include "foc_math.h"
#include "main.h"
#include "spi.h"
#include "tim.h"

#define FOC_CONTROL_ELEC_CNT (5461)
#define FOC_CONTROL_ELEC_RAW_TO_RAD (2.0f * (float)M_PI / (float)FOC_CONTROL_ELEC_CNT)
#define FOC_CONTROL_TORQUE_SIGN (-1.0f)
#define FOC_CONTROL_AXIS_OFFSET_RAD ((float)M_PI * 0.5f)
#define FOC_CONTROL_PHASE_ADVANCE_MODEL_RAD_PER_RPS (-0.003457f)
#define FOC_CONTROL_PHASE_ADVANCE_LIMIT_RAD (0.78539816339f)

static volatile float foc_control_phase_advance_trim_rad;

static inline float clampFocControlPhaseAdvance(float angle)
{
  if (angle > FOC_CONTROL_PHASE_ADVANCE_LIMIT_RAD) {
    return FOC_CONTROL_PHASE_ADVANCE_LIMIT_RAD;
  }
  if (angle < -FOC_CONTROL_PHASE_ADVANCE_LIMIT_RAD) {
    return -FOC_CONTROL_PHASE_ADVANCE_LIMIT_RAD;
  }
  return angle;
}

static inline float clampFocControlVoltage(float voltage, float limit)
{
  if (limit <= 0.0f) {
    return 0.0f;
  }
  if (voltage > limit) {
    return limit;
  }
  if (voltage < -limit) {
    return -limit;
  }
  return voltage;
}

float focControlRawPositiveElectricalAngle(int raw)
{
  return (float)(raw % FOC_CONTROL_ELEC_CNT) * FOC_CONTROL_ELEC_RAW_TO_RAD;
}

float focControlRawNegativeElectricalAngle(int raw)
{
  return focNormalizeAngle(-focControlRawPositiveElectricalAngle(raw));
}

float focControlBaseElectricalAngle(bool motor)
{
  const uint8_t motor_idx = motor ? 1U : 0U;
  return focNormalizeAngle(focControlRawNegativeElectricalAngle(as5047p[motor_idx].enc_raw)
    + enc_offset[motor_idx].zero_calib
    + sys.manual_offset_radian);
}

float focControlPhaseAdvanceModel(float speed_rps)
{
  const float model = FOC_CONTROL_PHASE_ADVANCE_MODEL_RAD_PER_RPS * fabsf(speed_rps);
  return clampFocControlPhaseAdvance(model);
}

float focControlPhaseAdvance(float speed_rps)
{
  return clampFocControlPhaseAdvance(focControlPhaseAdvanceModel(speed_rps) + foc_control_phase_advance_trim_rad);
}

float focControlAxisOffset(void)
{
  return FOC_CONTROL_AXIS_OFFSET_RAD;
}

float focControlTorqueSign(void)
{
  return FOC_CONTROL_TORQUE_SIGN;
}

void focControlAdjustPhaseAdvance(float delta_rad)
{
  foc_control_phase_advance_trim_rad = clampFocControlPhaseAdvance(foc_control_phase_advance_trim_rad + delta_rad);
}

void focControlResetPhaseAdvance(void)
{
  foc_control_phase_advance_trim_rad = 0.0f;
}

float focControlGetPhaseAdvanceTrim(void)
{
  return foc_control_phase_advance_trim_rad;
}

void focControlApplyVoltage(bool motor, float output_voltage, float speed_rps, float voltage_limit)
{
  const float voltage_q = FOC_CONTROL_TORQUE_SIGN * clampFocControlVoltage(output_voltage, voltage_limit);
  if (voltage_q == 0.0f) {
    foc_pwm_compare_t center = {TIM_PWM_CENTER, TIM_PWM_CENTER, TIM_PWM_CENTER, false};
    focDriverSetPwmCompare(motor, center);
    return;
  }

  float electrical = focControlBaseElectricalAngle(motor) + FOC_CONTROL_AXIS_OFFSET_RAD;
  const float phase_advance = focControlPhaseAdvance(speed_rps);
  electrical += (speed_rps < 0.0f) ? -phase_advance : phase_advance;
  focDriverApplySineVoltage(motor, voltage_q, 0.0f, electrical, getBatteryVoltage());
}

void focControlApplyFixedAngleVoltage(bool motor, float electrical_angle, float output_voltage, float voltage_limit)
{
  const float voltage_q = FOC_CONTROL_TORQUE_SIGN * clampFocControlVoltage(output_voltage, voltage_limit);
  if (voltage_q == 0.0f) {
    foc_pwm_compare_t center = {TIM_PWM_CENTER, TIM_PWM_CENTER, TIM_PWM_CENTER, false};
    focDriverSetPwmCompare(motor, center);
    return;
  }
  focDriverApplySineVoltage(motor, voltage_q, 0.0f, electrical_angle, getBatteryVoltage());
}
