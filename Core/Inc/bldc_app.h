/**
 * @file bldc_app.h
 * @brief BLDC application layer that owns control state, diagnostics, and HAL callbacks.
 */

#ifndef INC_BLDC_APP_H_
#define INC_BLDC_APP_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f3xx_hal.h"

typedef enum
{
  BLDC_APP_MODE_BOOT = 0,
  BLDC_APP_MODE_FREEWHEEL,
  BLDC_APP_MODE_READY,
  BLDC_APP_MODE_RUN,
  BLDC_APP_MODE_FAULT
} bldc_app_mode_t;

typedef struct
{
  float target_rps;
  float measured_rps;
  float measured_rps_ave;
  float voltage_q;
  float voltage_limit;
  float voltage_per_rps;
  float zero_electric_angle;
  int pre_raw;
  uint16_t command_timeout_ms;
  bool output_limited;
} bldc_app_motor_state_t;

void bldcAppInit(void);
void bldcAppTick1kHz(void);
void bldcAppOnTimerElapsed(TIM_HandleTypeDef * htim);
void bldcAppSetFreewheelMs(uint32_t ms);
void bldcAppEnableRun(void);
void bldcAppForceFault(uint16_t id, uint16_t info, float value);
uint32_t bldcAppGetCanRxCount(void);
bldc_app_mode_t bldcAppGetMode(void);
const bldc_app_motor_state_t * bldcAppGetMotorState(uint8_t motor);

#endif /* INC_BLDC_APP_H_ */

