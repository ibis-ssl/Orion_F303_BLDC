/**
 * @file foc_driver_hal.h
 * @brief HAL timer bridge for SimpleFOC-style phase voltage outputs.
 */

#ifndef INC_FOC_DRIVER_HAL_H_
#define INC_FOC_DRIVER_HAL_H_

#include <stdbool.h>

#include "foc_math.h"

void focDriverSetPwmCompare(bool motor, foc_pwm_compare_t compare);
foc_pwm_compare_t focDriverBuildSinePwm(float uq, float ud, float angle_el, float voltage_power_supply);
foc_pwm_compare_t focDriverBuildSinePwmUq(float uq, float angle_el, float voltage_power_supply);
void focDriverApplySineVoltage(bool motor, float uq, float ud, float angle_el, float voltage_power_supply);
void focDriverApplySineVoltageUq(bool motor, float uq, float angle_el, float voltage_power_supply);

#endif /* INC_FOC_DRIVER_HAL_H_ */
