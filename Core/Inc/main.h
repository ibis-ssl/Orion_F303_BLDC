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
#include <stdbool.h>
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
  ENC_ERROR = 0x0010,
  OVER_VOLTAGE = 0x0020,
};

// by manual tuned
#define ROTATION_OFFSET_RADIAN (2.0)
#define ENC_CNT_MAX (65536)
#define HARF_OF_ENC_CNT_MAX (32768)

#define SPEED_CMD_LIMIT_RPS (50)

typedef struct
{
  int ave_cnt;
  int pre_raw;
  int result_cw_cnt;
  int result_ccw_cnt;
  float rps_integral;

  struct
  {
    float radian_ave_x, radian_ave_y;
    float result_cw_x, result_cw_y;
    float result_ccw_x, result_ccw_y;
  } xy_field;

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
  int diff_cnt_max;

  float rps;
  float pre_rps;
  float k;
} motor_real_t;

typedef struct
{
  float voltage_per_rps;
  float output_voltage_limit;
} motor_param_t;

typedef struct
{
  float eff_voltage;
  float pid_kp, pid_kd, pid_ki;
  float error, error_integral, error_diff;
  float error_integral_limit;
  float pre_real_rps;
  float diff_voltage_limit;
  int load_limit_cnt;
  bool output_voltage_limitting;
} motor_pid_control_t;

typedef struct
{
  uint32_t free_wheel_cnt;     // エンコーダ飛び対策等 != 0でフリー回転(pwm全カット)
  float manual_offset_radian;  // デバッグとキャリブレーション時のエンコーダオフセット
  uint32_t power_enable_cnt;   // リセット時に待つ時間 x2ms
  uint32_t print_cnt, print_idx;
  bool is_starting_mode;
} system_t;

typedef struct
{
  uint16_t id, info;
  float value;
} error_t;

typedef struct
{
  bool detect_flag;
  int idx;
  int cnt;
} enc_error_watcher_t;

typedef struct
{
  uint32_t enc_calib_cnt;
  uint32_t motor_calib_cnt;  // disable : 0, init : 5000, start : 3500~ , end : 1
  uint32_t motor_calib_mode;
  float motor_calib_voltage;
  float force_rotation_speed;

  bool print_flag;
} calib_process_t;

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
