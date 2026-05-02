/**
 * @file app_rebuild.h
 * @brief Rebuilt BLDC application layer that owns control state, diagnostics, and HAL callbacks.
 */

#ifndef INC_APP_REBUILD_H_
#define INC_APP_REBUILD_H_

#include <stdbool.h>
#include <stdint.h>

#include "stm32f3xx_hal.h"

typedef enum
{
  APP_MODE_BOOT = 0,
  APP_MODE_FREEWHEEL,
  APP_MODE_READY,
  APP_MODE_RUN,
  APP_MODE_FAULT
} app_mode_t;

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
} app_motor_state_t;

void appInit(void);
void appTick1kHz(void);
void appOnTimerElapsed(TIM_HandleTypeDef * htim);
void appSetFreewheelMs(uint32_t ms);
void appEnableRun(void);
void appForceFault(uint16_t id, uint16_t info, float value);
uint32_t appGetCanRxCount(void);
app_mode_t appGetMode(void);
const app_motor_state_t * appGetMotorState(uint8_t motor);

#endif /* INC_APP_REBUILD_H_ */
