/**
 * @file foc_math.h
 * @brief SimpleFOC-style voltage FOC math helpers for the local C/HAL control stack.
 */

#ifndef INC_FOC_MATH_H_
#define INC_FOC_MATH_H_

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  float ua;
  float ub;
  float uc;
} foc_phase_voltage_t;

typedef struct
{
  uint16_t a;
  uint16_t b;
  uint16_t c;
  bool limited;
} foc_pwm_compare_t;

float focNormalizeAngle(float angle);
float focElectricalAngle(float shaft_angle, int pole_pairs, float zero_electric_angle, int sensor_direction);
float focLimitSymmetric(float value, float limit);
foc_phase_voltage_t focSetPhaseVoltageSine(float uq, float ud, float angle_el, float voltage_limit);
foc_pwm_compare_t focPhaseVoltageToCompare(foc_phase_voltage_t phase, float voltage_power_supply, uint16_t pwm_period);
bool focRunMathSelfTest(foc_pwm_compare_t * sample_out);

#endif /* INC_FOC_MATH_H_ */
