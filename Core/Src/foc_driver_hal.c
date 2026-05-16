/**
 * @file foc_driver_hal.c
 * @brief HAL timer bridge for SimpleFOC-style phase voltage outputs.
 */

#include "foc_driver_hal.h"

#include "tim.h"
#include "usart.h"

#define FOC_PWM_PERIOD (TIM_PWM_CENTER * 2U)

void focDriverSetPwmCompare(bool motor, foc_pwm_compare_t compare)
{
  if (motor == 0) {
    htim1.Instance->CCR1 = compare.a;
    htim1.Instance->CCR2 = compare.b;
    htim1.Instance->CCR3 = compare.c;
  } else {
    htim8.Instance->CCR1 = compare.a;
    htim8.Instance->CCR2 = compare.b;
    htim8.Instance->CCR3 = compare.c;
  }
}

foc_pwm_compare_t focDriverBuildSinePwm(float uq, float ud, float angle_el, float voltage_power_supply)
{
  foc_phase_voltage_t phase = focSetPhaseVoltageSine(uq, ud, angle_el, voltage_power_supply);
  return focPhaseVoltageToCompare(phase, voltage_power_supply, FOC_PWM_PERIOD);
}

void focDriverApplySineVoltage(bool motor, float uq, float ud, float angle_el, float voltage_power_supply)
{
  foc_pwm_compare_t compare = focDriverBuildSinePwm(uq, ud, angle_el, voltage_power_supply);
  focDriverSetPwmCompare(motor, compare);
}

void focDriverPrintState(void)
{
  p("PWM TIM1 CCR %4ld %4ld %4ld CCER 0x%04lx BDTR 0x%04lx\n", htim1.Instance->CCR1, htim1.Instance->CCR2, htim1.Instance->CCR3, htim1.Instance->CCER, htim1.Instance->BDTR);
  p("PWM TIM8 CCR %4ld %4ld %4ld CCER 0x%04lx BDTR 0x%04lx\n", htim8.Instance->CCR1, htim8.Instance->CCR2, htim8.Instance->CCR3, htim8.Instance->CCER, htim8.Instance->BDTR);
}
