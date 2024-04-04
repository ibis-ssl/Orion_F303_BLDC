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
  int cs_m0;
  int cs_m1;
  int batt_v;
  int temp_fet0;
  int temp_fet1;
  int temp_m0;
  int temp_m1;
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
float getCurrentM0(void);
float getCurrentM1(void);
int getTempFET0(void);
int getTempM1(void);
int getTempM0(void);
int getTempFET1(void);
void updateADC_M0(void);
void updateADC_M1(void);
float getGateDriverDCDCVoltage(void);
bool isNotZeroCurrent();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

