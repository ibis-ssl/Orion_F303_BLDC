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
  struct
  {
    uint8_t error, prog, diagagc, mag, angleenc, anglecom;
  } reg;
} as5047p_t;

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */

extern as5047p_t as5047p[2];

uint16_t readRegisterAS5047P(bool enc, uint16_t address);
void updateAS5047P(bool motor);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */
