/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdbool.h>
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */

typedef struct
{
  int cs_motor[2];
  int batt_v;
  int temp_fet[2];
  int temp_motor[2];
  int cs_adc_offset;
  int gd_dcdc_v;
} adc_raw_t;
extern adc_raw_t adc_raw;
/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */
float getBatteryVoltage(void);
float getGateDriverDCDCVoltage(void);
bool isNotZeroCurrent();

float getCurrentMotor(bool motor);
int getTempFET(bool motor);
int getTempMotor(bool motor);
void updateADC(bool motor);
void adcUpdateTemperatureFilters(void);

static inline float adcBatteryVoltageFast(void)
{
  return (float)adc_raw.batt_v * (3.3f * 11.0f / 4096.0f);
}

static inline float adcCurrentMotorFast(bool motor)
{
  return (float)(adc_raw.cs_motor[motor] - adc_raw.cs_adc_offset) * (3.3f * 8.0f / 4096.0f);
}

static inline void adcUpdateFast(bool motor)
{
  if (motor == 0) {
    GPIOC->BSRR = GPIO_PIN_13;
    adc_raw.batt_v = (int)ADC3->JDR1;
    adc_raw.cs_motor[1] = (int)ADC2->JDR1;
    adc_raw.temp_fet[0] = (int)ADC2->JDR2;
    adc_raw.temp_fet[1] = (int)ADC2->JDR3;
    ADC2->CR |= ADC_CR_JADSTART;
    ADC3->CR |= ADC_CR_JADSTART;
    GPIOC->BSRR = ((uint32_t)GPIO_PIN_13 << 16U);
  } else {
    GPIOC->BSRR = GPIO_PIN_13;
    adc_raw.cs_motor[0] = (int)ADC1->JDR1;
    adc_raw.temp_motor[0] = (int)ADC1->JDR2;
    adc_raw.temp_motor[1] = (int)ADC1->JDR3;
    adc_raw.gd_dcdc_v = (int)ADC1->JDR4;
    ADC1->CR |= ADC_CR_JADSTART;
    GPIOC->BSRR = ((uint32_t)GPIO_PIN_13 << 16U);
  }
}

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
