/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
enum {
  NONE = 0,
  UNDER_VOLTAGE = 0x0001,
  OVER_CURRENT = 0x0002,
  MOTOR_OVER_HEAT = 0x0004,
  OVER_LOAD = 0x0008,
};



typedef struct
{
  float pre_elec_radian;
  float radian_ave;
  int ave_cnt;
  int pre_raw;
  float result_cw;
  int result_cw_cnt;
  float result_ccw;
  int result_ccw_cnt;
  float rps_integral;
} calib_point_t;
typedef struct
{
  float final;
  float zero_calib;
} enc_offset_t;

typedef struct
{
  float speed;
  float limit;
  float out_v;
  float out_v_final;
  int timeout_cnt;
} motor_control_cmd_t;
typedef struct
{
  int pre_enc_cnt_raw;
  int diff_cnt_max, diff_cnt_min;

  float rps;
  float pre_rps;
  float k;
} motor_real_t;

typedef struct
{
  float voltage_per_rps;
} motor_param_t;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
