/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.h
 * @brief   This file contains all the function prototypes for
 *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

#include <stdint.h>

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

typedef union {
  uint8_t data[8];

  struct
  {
    float rev_p_sec;
    float omni_angle;
  } speed;

  struct
  {
    uint16_t id;
    uint16_t info;
    float value;
  } error;

  float value[2];
} can_msg_buf_t;

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

void CAN_Filter_Init(uint16_t board_addr);
void updateCANRXBuffer(void);
void sendFloatDual(uint32_t can_id, float data1, float data2);
void sendSpeed(int board_id, int motor, float speed, float angle);
void sendVoltage(int board_id, int motor, float voltage);
void sendTemperature(int board_id, int motor, float motor_temp, float fet_temp);
void sendCurrent(int board_id, int motor, float current);
void sendError(uint16_t error_id, uint16_t error_info, float error_value);
uint32_t getCanError(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */
