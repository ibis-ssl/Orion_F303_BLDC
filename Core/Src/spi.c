/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    spi.c
 * @brief   Configures SPI1 and updates MT6835 encoder angle/diagnostic data.
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
#include <stdlib.h>

#define MT6835_DIFF_PEAK_INTERVAL (16U)
#define MT6835_BURST_READ_COMMAND (0xA0U)
#define MT6835_ANGLE_REGISTER (0x03U)
#define MT6835_STATUS_UNDERVOLTAGE (0x04U)
#define MT6835_HEALTH_ERROR_LIMIT (30U)
#define MT6835_RAW_TO_CONTROL_DIRECTION(raw) ((ENC_CNT_MAX - (raw)) % ENC_CNT_MAX)

mt6835_t mt6835[2];
static uint8_t mt6835_diff_peak_div[2];

static const uint8_t mt6835_crc8_table[256] = {
  0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
  0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
  0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
  0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
  0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
  0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
  0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
  0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
  0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
  0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
  0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
  0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
  0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
  0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
  0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
  0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3,
};
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  int temp = mt6835[enc].pre_enc_raw - mt6835[enc].enc_raw;
  if (temp < -HARF_OF_ENC_CNT_MAX) {
    temp += ENC_CNT_MAX;
  } else if (temp > HARF_OF_ENC_CNT_MAX) {
    temp -= ENC_CNT_MAX;
  }

  mt6835[enc].diff_enc = temp;

  mt6835_diff_peak_div[enc]++;
  if (mt6835_diff_peak_div[enc] < MT6835_DIFF_PEAK_INTERVAL) {
    return;
  }
  mt6835_diff_peak_div[enc] = 0U;

  if (abs(mt6835[enc].diff_max) < abs(temp)) {
    mt6835[enc].diff_max = temp;
    mt6835[enc].diff_max_cnt = mt6835[enc].enc_raw;
  }
  if (abs(mt6835[enc].diff_min) > abs(temp)) {
    mt6835[enc].diff_min = temp;
    mt6835[enc].diff_min_cnt = mt6835[enc].enc_raw;
  }
}

static inline void selectMT6835(bool enc)
{
  GPIOB->BSRR = (uint32_t)(enc ? GPIO_PIN_6 : GPIO_PIN_7) << 16U;
}

static inline void deselectMT6835(bool enc)
{
  GPIOB->BSRR = enc ? GPIO_PIN_6 : GPIO_PIN_7;
}

static inline void mt6835CsSettle(void)
{
  __NOP();
  __NOP();
  __NOP();
  __NOP();
}

static inline uint8_t transferMT6835(uint8_t tx)
{
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_TXE) == RESET) {
  }
  *(__IO uint8_t *)&hspi1.Instance->DR = tx;
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_RXNE) == RESET) {
  }
  return *(__IO uint8_t *)&hspi1.Instance->DR;
}

static inline uint8_t mt6835CalculateCrc(uint8_t angle_high, uint8_t angle_middle, uint8_t angle_low_status)
{
  uint8_t crc = mt6835_crc8_table[angle_high];
  crc = mt6835_crc8_table[crc ^ angle_middle];
  return mt6835_crc8_table[crc ^ angle_low_status];
}

static inline uint32_t readMT6835BurstFrame(bool enc)
{
  selectMT6835(enc);
  mt6835CsSettle();
  (void)transferMT6835(MT6835_BURST_READ_COMMAND);
  (void)transferMT6835(MT6835_ANGLE_REGISTER);
  const uint8_t angle_high = transferMT6835(0x00U);
  const uint8_t angle_middle = transferMT6835(0x00U);
  const uint8_t angle_low_status = transferMT6835(0x00U);
  const uint8_t crc = transferMT6835(0x00U);
  while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_BSY) != RESET) {
  }
  deselectMT6835(enc);
  return ((uint32_t)angle_high << 24U) | ((uint32_t)angle_middle << 16U) | ((uint32_t)angle_low_status << 8U) | crc;
}

static inline void updateMT6835Common(mt6835_t * enc, uint32_t frame)
{
  const uint8_t angle_high = (uint8_t)(frame >> 24U);
  const uint8_t angle_middle = (uint8_t)(frame >> 16U);
  const uint8_t angle_low_status = (uint8_t)(frame >> 8U);
  const uint8_t received_crc = (uint8_t)frame;
  const uint8_t calculated_crc = mt6835CalculateCrc(angle_high, angle_middle, angle_low_status);

  enc->last_frame = frame;
  enc->last_crc = received_crc;
  enc->calculated_crc = calculated_crc;
  enc->status = angle_low_status & 0x07U;
  if (calculated_crc != received_crc) {
    enc->crc_error_count++;
    enc->consecutive_error_count++;
    return;
  }
  if (enc->status != 0U) {
    enc->status_error_count++;
    if ((enc->status & MT6835_STATUS_UNDERVOLTAGE) != 0U) {
      enc->undervoltage_count++;
    }
    enc->consecutive_error_count++;
    return;
  }

  enc->pre_enc_raw = enc->enc_raw;
  enc->angle_raw_21bit = ((uint32_t)angle_high << 13U) | ((uint32_t)angle_middle << 5U) | ((uint32_t)angle_low_status >> 3U);
  enc->enc_raw = MT6835_RAW_TO_CONTROL_DIRECTION((int)(enc->angle_raw_21bit >> 5U));
  enc->enc_elec_raw = 5461 - (enc->enc_raw % 5461);
  enc->output_radian = (float)enc->enc_elec_raw / 5461.0f * 2.0f * (float)M_PI;
  enc->consecutive_error_count = 0U;
  enc->successful_update_count++;
}

void updateMT6835(bool motor)
{
  const uint8_t index = motor ? 1U : 0U;
  updateMT6835Common(&mt6835[index], readMT6835BurstFrame(index));
  updateDiff(index);
}

void updateMT6835Diagnostics(bool motor)
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  updateMT6835(motor);
  if (primask == 0U) {
    __enable_irq();
  }
}

bool isMT6835Ready(bool motor)
{
  const uint8_t index = motor ? 1U : 0U;
  return mt6835[index].successful_update_count != 0U;
}

bool isMT6835Healthy(bool motor)
{
  const uint8_t index = motor ? 1U : 0U;
  return isMT6835Ready(motor) && mt6835[index].consecutive_error_count < MT6835_HEALTH_ERROR_LIMIT;
}

/* USER CODE END 1 */
