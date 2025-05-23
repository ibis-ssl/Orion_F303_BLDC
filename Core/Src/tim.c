/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include <math.h>

#include "stm32f3xx_hal_tim_ex.h"
/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

/* TIM1 init function */
void MX_TIM1_Init(void)
{
  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 10;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}
/* TIM8 init function */
void MX_TIM8_Init(void)
{
  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1800;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 10;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 4;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 4;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef * tim_pwmHandle)
{
  if (tim_pwmHandle->Instance == TIM1) {
    /* USER CODE BEGIN TIM1_MspInit 0 */

    /* USER CODE END TIM1_MspInit 0 */
    /* TIM1 clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    /* USER CODE BEGIN TIM1_MspInit 1 */

    /* USER CODE END TIM1_MspInit 1 */
  } else if (tim_pwmHandle->Instance == TIM8) {
    /* USER CODE BEGIN TIM8_MspInit 0 */

    /* USER CODE END TIM8_MspInit 0 */
    /* TIM8 clock enable */
    __HAL_RCC_TIM8_CLK_ENABLE();

    /* TIM8 interrupt Init */
    HAL_NVIC_SetPriority(TIM8_UP_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM8_UP_IRQn);
    HAL_NVIC_SetPriority(TIM8_CC_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
    /* USER CODE BEGIN TIM8_MspInit 1 */

    /* USER CODE END TIM8_MspInit 1 */
  }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef * timHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if (timHandle->Instance == TIM1) {
    /* USER CODE BEGIN TIM1_MspPostInit 0 */

    /* USER CODE END TIM1_MspPostInit 0 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**TIM1 GPIO Configuration
    PA7     ------> TIM1_CH1N
    PB0     ------> TIM1_CH2N
    PB1     ------> TIM1_CH3N
    PA8     ------> TIM1_CH1
    PA9     ------> TIM1_CH2
    PA10     ------> TIM1_CH3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM1_MspPostInit 1 */

    /* USER CODE END TIM1_MspPostInit 1 */
  } else if (timHandle->Instance == TIM8) {
    /* USER CODE BEGIN TIM8_MspPostInit 0 */

    /* USER CODE END TIM8_MspPostInit 0 */

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**TIM8 GPIO Configuration
    PC6     ------> TIM8_CH1
    PC7     ------> TIM8_CH2
    PC8     ------> TIM8_CH3
    PC10     ------> TIM8_CH1N
    PC11     ------> TIM8_CH2N
    PC12     ------> TIM8_CH3N
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_TIM8;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM8_MspPostInit 1 */

    /* USER CODE END TIM8_MspPostInit 1 */
  }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef * tim_pwmHandle)
{
  if (tim_pwmHandle->Instance == TIM1) {
    /* USER CODE BEGIN TIM1_MspDeInit 0 */

    /* USER CODE END TIM1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();

    /* TIM1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM1_UP_TIM16_IRQn);
    HAL_NVIC_DisableIRQ(TIM1_CC_IRQn);
    /* USER CODE BEGIN TIM1_MspDeInit 1 */

    /* USER CODE END TIM1_MspDeInit 1 */
  } else if (tim_pwmHandle->Instance == TIM8) {
    /* USER CODE BEGIN TIM8_MspDeInit 0 */

    /* USER CODE END TIM8_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM8_CLK_DISABLE();

    /* TIM8 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM8_UP_IRQn);
    HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
    /* USER CODE BEGIN TIM8_MspDeInit 1 */

    /* USER CODE END TIM8_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
float rad_to_sin_cnv_array[0x400 * 4] = {0};
inline void initFirstSin(void)
{
  for (int i = 0; i < 0x400 * 4; i++) {
    float temp_rad = (float)i / 0x400 * M_PI * 2;
    rad_to_sin_cnv_array[i] = sin(temp_rad);
    // printf("rad %4.3f sin %4.3f\n",temp_rad,rad_to_sin_cnv_array[i]);
    // HAL_Delay(1);
  }
}

float get_sin_table(uint16_t idx)
{
  return rad_to_sin_cnv_array[idx];
}

inline float fast_sin(float rad)
{
  return rad_to_sin_cnv_array[(uint16_t)(((float)(rad + M_PI * 4) / (M_PI * 2) * 0x400)) & 0x3FF];
}

#define BATTERY_VOLTAGE_BOTTOM (18)
#define X2_PER_R3 (1.154)

inline void setOutputRadianMotor(bool motor, float out_rad, float output_voltage, float battery_voltage, float output_voltage_limit)
{
  int voltage_propotional_cnt;

  if (battery_voltage < BATTERY_VOLTAGE_BOTTOM) {
    battery_voltage = BATTERY_VOLTAGE_BOTTOM;
  }
  if (output_voltage < 0) {
    output_voltage = -output_voltage;
  }
  if (output_voltage > output_voltage_limit) {
    output_voltage = 0;
  }
  voltage_propotional_cnt = output_voltage / battery_voltage * TIM_PWM_CENTER * X2_PER_R3;

  uint16_t rad_to_cnt = (uint16_t)(((float)(out_rad + M_PI * 4) / (M_PI * 2) * 0x400)) & 0x3FF;
  uint16_t output_ccr_cnt[3];
  output_ccr_cnt[0] = TIM_PWM_CENTER + voltage_propotional_cnt * rad_to_sin_cnv_array[rad_to_cnt];
  output_ccr_cnt[1] = TIM_PWM_CENTER + voltage_propotional_cnt * rad_to_sin_cnv_array[341 + rad_to_cnt];  //+85 = 1/3 -> 1024x1/3
  output_ccr_cnt[2] = TIM_PWM_CENTER + voltage_propotional_cnt * rad_to_sin_cnv_array[683 + rad_to_cnt];  //-85 = 2/4 -> 1024x2/3
  if (motor == 0) {
    htim1.Instance->CCR1 = output_ccr_cnt[0];
    htim1.Instance->CCR2 = output_ccr_cnt[1];  //+85 = 1/3 -> 1024x1/3
    htim1.Instance->CCR3 = output_ccr_cnt[2];  //-85 = 2/4 -> 1024x2/3
  } else {
    htim8.Instance->CCR1 = output_ccr_cnt[0];
    htim8.Instance->CCR2 = output_ccr_cnt[1];
    htim8.Instance->CCR3 = output_ccr_cnt[2];
  }
}

inline void setOutputRadianM0(float out_rad, float output_voltage, float battery_voltage, float output_voltage_limit)
{
  int voltage_propotional_cnt;

  if (battery_voltage < BATTERY_VOLTAGE_BOTTOM) {
    battery_voltage = BATTERY_VOLTAGE_BOTTOM;
  }
  if (output_voltage < 0) {
    output_voltage = -output_voltage;
  }
  if (output_voltage > output_voltage_limit) {
    output_voltage = 0;
  }
  voltage_propotional_cnt = output_voltage / battery_voltage * TIM_PWM_CENTER * X2_PER_R3;

  uint16_t rad_to_cnt = (uint8_t)((out_rad + M_PI * 4) / (M_PI * 2) * 255);
  htim1.Instance->CCR1 = TIM_PWM_CENTER + voltage_propotional_cnt * rad_to_sin_cnv_array[rad_to_cnt];
  htim1.Instance->CCR2 = TIM_PWM_CENTER + voltage_propotional_cnt * rad_to_sin_cnv_array[85 + rad_to_cnt];
  htim1.Instance->CCR3 = TIM_PWM_CENTER + voltage_propotional_cnt * rad_to_sin_cnv_array[170 + rad_to_cnt];
}

inline void setOutputRadianM1(float out_rad, float output_voltage, float battery_voltage, float output_voltage_limit)
{
  int voltage_propotional_cnt;
  if (battery_voltage < BATTERY_VOLTAGE_BOTTOM) {
    battery_voltage = BATTERY_VOLTAGE_BOTTOM;
  }
  if (output_voltage < 0) {
    output_voltage = -output_voltage;
  }
  if (output_voltage > output_voltage_limit) {
    output_voltage = 0;
  }
  voltage_propotional_cnt = output_voltage / battery_voltage * TIM_PWM_CENTER * X2_PER_R3;

  uint16_t rad_to_cnt = (uint8_t)((out_rad + M_PI * 4) / (M_PI * 2) * 255);
  htim8.Instance->CCR1 = TIM_PWM_CENTER + voltage_propotional_cnt * rad_to_sin_cnv_array[rad_to_cnt];
  htim8.Instance->CCR2 = TIM_PWM_CENTER + voltage_propotional_cnt * rad_to_sin_cnv_array[85 + rad_to_cnt];
  htim8.Instance->CCR3 = TIM_PWM_CENTER + voltage_propotional_cnt * rad_to_sin_cnv_array[170 + rad_to_cnt];
}

void setPwmAll(uint32_t pwm_cnt)
{
  htim8.Instance->CCR1 = pwm_cnt;
  htim8.Instance->CCR2 = pwm_cnt;
  htim8.Instance->CCR3 = pwm_cnt;
  htim1.Instance->CCR1 = pwm_cnt;
  htim1.Instance->CCR2 = pwm_cnt;
  htim1.Instance->CCR3 = pwm_cnt;
}

void forceStopAllPwmOutputAndTimer(void)
{
  setPwmAll(TIM_PWM_CENTER);

  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3);

  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIM_Base_Stop_IT(&htim8);

  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
  __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

static void TIM_CCxNChannelCmd(TIM_TypeDef * TIMx, uint32_t Channel, uint32_t ChannelNState)
{
  uint32_t tmp;

  tmp = TIM_CCER_CC1NE << (Channel & 0x1FU); /* 0x1FU = 31 bits max shift */

  /* Reset the CCxNE Bit */
  TIMx->CCER &= ~tmp;

  /* Set or reset the CCxNE Bit */
  TIMx->CCER |= (uint32_t)(ChannelNState << (Channel & 0x1FU)); /* 0x1FU = 31 bits max shift */
}

void setPwmOutPutFreeWheel(void)
{
  TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
  TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_DISABLE);
  TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_DISABLE);

  TIM_CCxChannelCmd(htim8.Instance, TIM_CHANNEL_1, TIM_CCx_DISABLE);
  TIM_CCxChannelCmd(htim8.Instance, TIM_CHANNEL_2, TIM_CCx_DISABLE);
  TIM_CCxChannelCmd(htim8.Instance, TIM_CHANNEL_3, TIM_CCx_DISABLE);

  TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_DISABLE);
  TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCxN_DISABLE);
  TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE);

  TIM_CCxNChannelCmd(htim8.Instance, TIM_CHANNEL_1, TIM_CCxN_DISABLE);
  TIM_CCxNChannelCmd(htim8.Instance, TIM_CHANNEL_2, TIM_CCxN_DISABLE);
  TIM_CCxNChannelCmd(htim8.Instance, TIM_CHANNEL_3, TIM_CCxN_DISABLE);
}

void resumePwmOutput(void)
{
  TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
  TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);

  TIM_CCxChannelCmd(htim8.Instance, TIM_CHANNEL_1, TIM_CCx_ENABLE);
  TIM_CCxChannelCmd(htim8.Instance, TIM_CHANNEL_2, TIM_CCx_ENABLE);
  TIM_CCxChannelCmd(htim8.Instance, TIM_CHANNEL_3, TIM_CCx_ENABLE);

  TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_1, TIM_CCxN_ENABLE);
  TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_2, TIM_CCxN_ENABLE);
  TIM_CCxNChannelCmd(htim1.Instance, TIM_CHANNEL_3, TIM_CCxN_ENABLE);

  TIM_CCxNChannelCmd(htim8.Instance, TIM_CHANNEL_1, TIM_CCxN_ENABLE);
  TIM_CCxNChannelCmd(htim8.Instance, TIM_CHANNEL_2, TIM_CCxN_ENABLE);
  TIM_CCxNChannelCmd(htim8.Instance, TIM_CHANNEL_3, TIM_CCxN_ENABLE);
}

void stopTimerInterrupt(void)
{
  HAL_TIM_Base_Stop_IT(&htim1);
  HAL_TIM_Base_Stop_IT(&htim8);
}
/* USER CODE END 1 */
