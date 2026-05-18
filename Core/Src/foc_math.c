/**
 * @file foc_math.c
 * @brief SimpleFOC-style voltage FOC math helpers without Arduino or C++ dependencies.
 */

#include "foc_math.h"

#include <math.h>

#include "tim.h"

#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define FOC_TWO_PI ((float)(2.0 * M_PI))
#define FOC_SQRT3_2 (0.8660254037844386f)

void focMathInit(void)
{
  /*
   * No local sine table is allocated in this branch. tim.c already owns a large
   * legacy table, and RAM is tight on STM32F303RBTx. FOC sine/cosine lookup uses
   * that table through fast_sin().
   */
}

float focNormalizeAngle(float angle)
{
  while (angle >= FOC_TWO_PI) {
    angle -= FOC_TWO_PI;
  }
  while (angle < 0.0f) {
    angle += FOC_TWO_PI;
  }
  return angle;
}

static void focFastSinCos(float angle, float * sin_out, float * cos_out)
{
  angle = focNormalizeAngle(angle);
  *sin_out = fast_sin(angle);
  *cos_out = fast_sin(angle + (float)M_PI * 0.5f);
}

float focElectricalAngle(float shaft_angle, int pole_pairs, float zero_electric_angle, int sensor_direction)
{
  const float direction = (sensor_direction < 0) ? -1.0f : 1.0f;
  return focNormalizeAngle(direction * (float)pole_pairs * shaft_angle - zero_electric_angle);
}

float focLimitSymmetric(float value, float limit)
{
  if (limit <= 0.0f) {
    return 0.0f;
  }
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

foc_phase_voltage_t focSetPhaseVoltageSine(float uq, float ud, float angle_el, float voltage_limit)
{
  foc_phase_voltage_t phase;
  const float half_limit = voltage_limit * 0.5f;
  float ualpha;
  float ubeta;
  float sin_el;
  float cos_el;

  if (voltage_limit <= 0.0f) {
    phase.ua = 0.0f;
    phase.ub = 0.0f;
    phase.uc = 0.0f;
    return phase;
  }

  uq = focLimitSymmetric(uq, half_limit);
  ud = focLimitSymmetric(ud, half_limit);
  focFastSinCos(angle_el, &sin_el, &cos_el);

  /* Inverse Park + Clarke transform, with B/C phase order matching legacy PWM. */
  ualpha = cos_el * ud - sin_el * uq;
  ubeta = sin_el * ud + cos_el * uq;

  phase.ua = ualpha + half_limit;
  phase.ub = -0.5f * ualpha - FOC_SQRT3_2 * ubeta + half_limit;
  phase.uc = -0.5f * ualpha + FOC_SQRT3_2 * ubeta + half_limit;

  return phase;
}

foc_pwm_compare_t focPhaseVoltageToCompare(foc_phase_voltage_t phase, float voltage_power_supply, uint16_t pwm_period)
{
  foc_pwm_compare_t cmp = {0};

  if (voltage_power_supply <= 0.0f || pwm_period == 0U) {
    cmp.limited = true;
    return cmp;
  }

  const float scale = (float)pwm_period / voltage_power_supply;
  float v = phase.ua;
  if (v < 0.0f) {
    v = 0.0f;
    cmp.limited = true;
  } else if (v > voltage_power_supply) {
    v = voltage_power_supply;
    cmp.limited = true;
  }
  cmp.a = (uint16_t)(v * scale);

  v = phase.ub;
  if (v < 0.0f) {
    v = 0.0f;
    cmp.limited = true;
  } else if (v > voltage_power_supply) {
    v = voltage_power_supply;
    cmp.limited = true;
  }
  cmp.b = (uint16_t)(v * scale);

  v = phase.uc;
  if (v < 0.0f) {
    v = 0.0f;
    cmp.limited = true;
  } else if (v > voltage_power_supply) {
    v = voltage_power_supply;
    cmp.limited = true;
  }
  cmp.c = (uint16_t)(v * scale);

  return cmp;
}

foc_pwm_compare_t focBuildSinePwmUq(float uq, float angle_el, float voltage_power_supply, uint16_t pwm_period)
{
  foc_pwm_compare_t cmp = {0};
  if (voltage_power_supply <= 0.0f || pwm_period == 0U) {
    cmp.limited = true;
    return cmp;
  }

  const float half_supply = voltage_power_supply * 0.5f;
  uq = focLimitSymmetric(uq, half_supply);

  const float sin_el = fast_sin(angle_el);
  const float cos_el = fast_sin(angle_el + (float)M_PI * 0.5f);
  const float ualpha = -sin_el * uq;
  const float ubeta = cos_el * uq;
  const float scale = (float)pwm_period / voltage_power_supply;

  float v = ualpha + half_supply;
  if (v < 0.0f) {
    v = 0.0f;
    cmp.limited = true;
  } else if (v > voltage_power_supply) {
    v = voltage_power_supply;
    cmp.limited = true;
  }
  cmp.a = (uint16_t)(v * scale);

  v = -0.5f * ualpha - FOC_SQRT3_2 * ubeta + half_supply;
  if (v < 0.0f) {
    v = 0.0f;
    cmp.limited = true;
  } else if (v > voltage_power_supply) {
    v = voltage_power_supply;
    cmp.limited = true;
  }
  cmp.b = (uint16_t)(v * scale);

  v = -0.5f * ualpha + FOC_SQRT3_2 * ubeta + half_supply;
  if (v < 0.0f) {
    v = 0.0f;
    cmp.limited = true;
  } else if (v > voltage_power_supply) {
    v = voltage_power_supply;
    cmp.limited = true;
  }
  cmp.c = (uint16_t)(v * scale);

  return cmp;
}
