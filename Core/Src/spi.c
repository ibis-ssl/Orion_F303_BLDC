/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    spi.c
 * @brief   This file provides code for the configuration
 *          of the SPI instances.
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
#include "spi.h"

/* USER CODE BEGIN 0 */
as5047p_t as5047p[2];
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

void HAL_SPI_MspInit(SPI_HandleTypeDef * spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (spiHandle->Instance == SPI1) {
    /* USER CODE BEGIN SPI1_MspInit 0 */

    /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN SPI1_MspInit 1 */

    /* USER CODE END SPI1_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef * spiHandle)
{
  if (spiHandle->Instance == SPI1) {
    /* USER CODE BEGIN SPI1_MspDeInit 0 */

    /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();

    /**SPI1 GPIO Configuration
    PB3     ------> SPI1_SCK
    PB4     ------> SPI1_MISO
    PB5     ------> SPI1_MOSI
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    /* USER CODE BEGIN SPI1_MspDeInit 1 */

    /* USER CODE END SPI1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

static inline void updateDiff(bool enc)
{
  int temp = as5047p[enc].pre_enc_raw - as5047p[enc].enc_raw;
  if (temp < -HARF_OF_ENC_CNT_MAX) {
    temp += ENC_CNT_MAX;
  } else if (temp > HARF_OF_ENC_CNT_MAX) {
    temp -= ENC_CNT_MAX;
  }

  as5047p[enc].diff_enc = temp;
}

static inline void selectAS5047P(bool enc)
{
  GPIOB->BSRR = (uint32_t)(enc ? GPIO_PIN_6 : GPIO_PIN_7) << 16U;
}

static inline void deselectAS5047P(bool enc)
{
  GPIOB->BSRR = enc ? GPIO_PIN_6 : GPIO_PIN_7;
}

static uint16_t as5047pEvenParity(uint16_t value)
{
  value ^= value >> 8;
  value ^= value >> 4;
  value ^= value >> 2;
  value ^= value >> 1;
  return value & 1U;
}

static uint16_t makeAS5047PReadCommand(uint16_t reg_address)
{
  uint16_t command = (reg_address & 0x3FFFU) | 0x4000U;
  if (as5047pEvenParity(command) != 0U) {
    command |= 0x8000U;
  }
  return command;
}

static inline void as5047pCsSettle(void)
{
  for (uint8_t i = 0U; i < 64U; i++) {
    __NOP();
  }
}

static uint16_t transferAS5047P(uint16_t tx)
{
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET) {
  }
  hspi1.Instance->DR = tx;
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE) == RESET) {
  }
  const uint16_t rx = (uint16_t)hspi1.Instance->DR;
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET) {
  }
  return rx;
}

uint16_t readRegisterAS5047P(bool enc, uint16_t reg_address)
{
  selectAS5047P(enc);
  as5047pCsSettle();
  (void)transferAS5047P(makeAS5047PReadCommand(reg_address));
  deselectAS5047P(enc);

  as5047pCsSettle();
  selectAS5047P(enc);
  as5047pCsSettle();
  const uint16_t rx = transferAS5047P(0xFFFFU);
  deselectAS5047P(enc);

  return (rx & 0x3FFF);
}

void updateAS5047PDiagnostics(bool motor)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  as5047p[motor].reg.error = readRegisterAS5047P(motor, 0x0001U);
  as5047p[motor].reg.prog = readRegisterAS5047P(motor, 0x0003U);
  as5047p[motor].reg.diagagc = readRegisterAS5047P(motor, 0x3FFCU);
  as5047p[motor].reg.mag = readRegisterAS5047P(motor, 0x3FFDU);
  as5047p[motor].reg.angleenc = readRegisterAS5047P(motor, 0x3FFEU);
  as5047p[motor].reg.anglecom = readRegisterAS5047P(motor, 0x3FFFU);
  if (primask == 0U) {
    __enable_irq();
  }
}

static inline void updateAS5047P_Common(as5047p_t * enc)
{
  enc->pre_enc_raw = enc->enc_raw;

  // addr 0x3FFF & 1 << 14 (read) & parity
  // 0x3FFE : without dynamic errro compensation
  // 0x3FFF : with dynamic errro compensation
  const uint16_t frame = transferAS5047P(0xFFFF);
  enc->last_frame = frame;
  if ((frame & 0x4000U) != 0U) {
    enc->spi_error_count++;
    return;
  }
  // 分解能は14bitだが、後段であまり算をするため、16bitに変換
  enc->enc_raw = (int)((frame & 0x3FFFU) << 2);
}

void updateAS5047P(bool motor)
{
  if (motor == 0) {
    selectAS5047P(0);
    as5047pCsSettle();

    updateAS5047P_Common(&as5047p[0]);
    updateDiff(0);

    deselectAS5047P(0);
  } else {
    selectAS5047P(1);
    as5047pCsSettle();

    updateAS5047P_Common(&as5047p[1]);
    updateDiff(1);

    deselectAS5047P(1);
  }
}

/* USER CODE END 1 */
