/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    can.c
 * @brief   This file provides code for the configuration
 *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
int ex_can_send_fail_cnt;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{
  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */
}

void HAL_CAN_MspInit(CAN_HandleTypeDef * canHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (canHandle->Instance == CAN) {
    /* USER CODE BEGIN CAN_MspInit 0 */

    /* USER CODE END CAN_MspInit 0 */
    /* CAN clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN interrupt Init */
    HAL_NVIC_SetPriority(CAN_RX1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(CAN_RX1_IRQn);
    /* USER CODE BEGIN CAN_MspInit 1 */

    /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef * canHandle)
{
  if (canHandle->Instance == CAN) {
    /* USER CODE BEGIN CAN_MspDeInit 0 */

    /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

    /* CAN interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN_RX1_IRQn);
    /* USER CODE BEGIN CAN_MspDeInit 1 */

    /* USER CODE END CAN_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void CAN_Filter_Init(uint16_t board_addr)
{
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterActivation = ENABLE;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterIdHigh = (0x100 + board_addr * 2) << 5;  //speed
  sFilterConfig.FilterIdLow = (0x310) << 5;                    //motor calib
  sFilterConfig.FilterMaskIdHigh = (0x000) << 5;               // emg stop
  sFilterConfig.FilterMaskIdLow = (0x001) << 5;                // error report
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  sFilterConfig.FilterIdHigh = (0x110) << 5;                       // kick
  sFilterConfig.FilterIdLow = (0x010) << 5;                        // power enable
  sFilterConfig.FilterMaskIdHigh = (0x101 + board_addr * 2) << 5;  //speed
  sFilterConfig.FilterMaskIdLow = (0x320) << 5;                    // notused
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1;
  sFilterConfig.FilterBank = 1;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
}

void sendFloatDual(uint32_t can_id, float data1, float data2)
{
  can_msg_buf_t msg;
  CAN_TxHeaderTypeDef can_header;
  uint32_t can_mailbox;
  can_header.StdId = can_id;
  can_header.ExtId = 0;
  can_header.RTR = CAN_RTR_DATA;
  can_header.DLC = 8;
  can_header.IDE = CAN_ID_STD;
  can_header.TransmitGlobalTime = DISABLE;
  msg.value[0] = data1;
  msg.value[1] = data2;
  if (HAL_CAN_AddTxMessage(&hcan, &can_header, msg.data, &can_mailbox) != 0) {
    ex_can_send_fail_cnt++;
  }
}

void sendSpeedInfo(uint32_t can_id, float rev_per_sec_, float omni_angle_)
{
  can_msg_buf_t msg;
  CAN_TxHeaderTypeDef can_header;
  uint32_t can_mailbox;
  can_header.StdId = can_id;
  can_header.ExtId = 0;
  can_header.RTR = CAN_RTR_DATA;
  can_header.DLC = 8;
  can_header.IDE = CAN_ID_STD;
  can_header.TransmitGlobalTime = DISABLE;
  msg.speed.rev_p_sec = rev_per_sec_;
  msg.speed.omni_angle = omni_angle_;
  if (HAL_CAN_AddTxMessage(&hcan, &can_header, msg.data, &can_mailbox) != 0) {
    ex_can_send_fail_cnt++;
  }
}

void sendSpeed(int board_id, int motor, float speed, float angle) { sendSpeedInfo(0x200 + board_id * 2 + motor, speed, angle); }

void sendVoltage(int board_id, int motor, float voltage) { sendFloatDual(0x210 + board_id * 2 + motor, voltage, 0); }

void sendTemperature(int board_id, int motor, float motor_temp, float fet_temp) { sendFloatDual(0x220 + board_id * 2 + motor, motor_temp, fet_temp); }

void sendCurrent(int board_id, int motor, float current) { sendFloatDual(0x230 + board_id * 2 + motor, current, 0); }

// id : motor
void sendError(uint16_t error_id, uint16_t error_info, float error_value)
{
  can_msg_buf_t msg;
  CAN_TxHeaderTypeDef can_header;
  uint32_t can_mailbox;
  can_header.StdId = 0;
  can_header.ExtId = 0;
  can_header.RTR = CAN_RTR_DATA;
  can_header.DLC = 8;
  can_header.IDE = CAN_ID_STD;
  can_header.TransmitGlobalTime = DISABLE;
  msg.error.id = error_id;
  msg.error.info = error_info;
  msg.error.value = error_value;
  if (HAL_CAN_AddTxMessage(&hcan, &can_header, msg.data, &can_mailbox) != 0) {
    ex_can_send_fail_cnt++;
  }
}

uint32_t getCanError(void)
{
  uint32_t err = HAL_CAN_GetError(&hcan);
  HAL_CAN_ResetError(&hcan);
  return err;
}
/* USER CODE END 1 */
