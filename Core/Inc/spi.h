/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */

typedef struct
{
  float output_radian;
  int enc_raw;
  int enc_elec_raw;
  int pre_enc_raw;
  int diff_enc, diff_min, diff_max, diff_max_cnt, diff_min_cnt;
  uint32_t crc_error_count;
  uint32_t status_error_count;
  uint32_t undervoltage_count;
  uint32_t angle_raw_21bit;
  uint32_t last_frame;
  uint8_t status;
  uint8_t last_crc;
  uint8_t calculated_crc;
} mt6835_t;

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */

extern mt6835_t mt6835[2];

void updateMT6835(bool motor);
void updateMT6835Diagnostics(bool motor);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
