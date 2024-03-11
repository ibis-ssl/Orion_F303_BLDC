/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 2000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PC4     ------> USART1_TX
    PC5     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 15, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PC4     ------> USART1_TX
    PC5     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4|GPIO_PIN_5);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

#define UART_TEMP_BUF_SIZE (800)
static char first_buf[UART_TEMP_BUF_SIZE];
static char second_buf[UART_TEMP_BUF_SIZE];
volatile int second_buf_len = 0, first_buf_len = 0;
volatile bool sending_second_buf = false, sending_first_buf = false;
volatile bool is_in_printf_func = false;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

  if (sending_first_buf)
  {                            // FIRST buf complete
    sending_first_buf = false; // complete!

    if (second_buf_len > 0 && is_in_printf_func == false)
    { // another buffer?
      sending_second_buf = true;
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)second_buf, second_buf_len);
      second_buf_len = 0;
    }
  }
  else if (sending_second_buf)
  {                             // SECOND buf complete
    sending_second_buf = false; // complete!

    if (first_buf_len > 0 && is_in_printf_func == false)
    { // another buffer?
      sending_first_buf = true;
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)first_buf, first_buf_len);
      first_buf_len = 0;
    }
  }
}

void p(const char *format, ...)
{
  va_list ap;
  va_start(ap, format);
  is_in_printf_func = true;

  if (sending_first_buf)
  {
    if (second_buf_len > UART_TEMP_BUF_SIZE / 2)
    {
      is_in_printf_func = false;
      return;
    }
    second_buf_len += vsprintf(second_buf + second_buf_len, format, ap);
    va_end(ap);
    if (sending_first_buf == false)
    {
      second_buf_len = (int)strlen(second_buf);
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)second_buf, second_buf_len); // 2ms
    }
  }
  else if (sending_second_buf)
  {
    if (first_buf_len > UART_TEMP_BUF_SIZE / 2)
    {

      is_in_printf_func = false;
      return;
    }

    first_buf_len += vsprintf(first_buf + first_buf_len, format, ap);
    va_end(ap);

    if (sending_second_buf == false)
    {
      first_buf_len = (int)strlen(first_buf);
      HAL_UART_Transmit_DMA(&huart1, (uint8_t *)first_buf, first_buf_len); // 2ms
    }
  }
  else
  {
    // start !!
    first_buf_len = vsprintf(first_buf, format, ap);
    va_end(ap);
    sending_first_buf = true;
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)first_buf, first_buf_len); // 2ms
    first_buf_len = (int)strlen(first_buf);
    first_buf_len = 0;
    second_buf_len = 0;
  }
  is_in_printf_func = false;
  return;
}
/* USER CODE END 1 */
