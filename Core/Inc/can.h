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

  typedef union
  {
    uint8_t data[8];

    float speed;
    float power;
    float voltage;
    float current;
    float temperature;
  } can_msg_buf_t;

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */

  void CAN_Filter_Init(uint16_t board_addr);
  void updateCANRXBuffer(void);
  void sendFloat(uint16_t can_id, float data);
  void sendSpeed(int board_id, int motor, float speed);
  void sendVoltage(int board_id, int motor, float voltage);
  void sendTemperature(int board_id, int motor, float temp);
  void sendCurrent(int board_id, int motor, float current);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

