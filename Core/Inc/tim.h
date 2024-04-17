/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim8;

/* USER CODE BEGIN Private defines */
#define TIM_PWM_CENTER (900)

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM8_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef * htim);

/* USER CODE BEGIN Prototypes */

void initFirstSin(void);
float fast_sin(float rad);

void forceStopAllPwmOutputAndTimer(void);
void setPwmOutPutFreeWheel(void);
void resumePwmOutput(void);
void stopTimerInterrupt(void);
void setPwmAll(uint32_t pwm_cnt);
void setOutputRadianMotor(bool motor, float out_rad, float output_voltage, float battery_voltage, float output_voltage_limit);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
