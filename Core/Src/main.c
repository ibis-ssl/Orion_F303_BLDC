/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 * @note           : Dual BLDC control application. This file owns mode control,
 *                   safety management, and high-level scheduling.
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
#include "main.h"

#include "adc.h"
#include "can.h"
#include "calibration.h"
#include "comms.h"
#include "control_limits.h"
#include "control_mode.h"
#include "diagnostics.h"
#include "dma.h"
#include "gpio.h"
#include "protect.h"
#include "spi.h"
#include "startup_sequence.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "flash.h"
#include "motor.h"
#include "stm32f3xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void runMode(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 0 ~ M_PI*2 * 4

motor_control_cmd_t cmd[2];
calib_point_t calib[2];
enc_offset_t enc_offset[2];
motor_real_t motor_real[2];
motor_param_t motor_param[2];
motor_pid_control_t pid[2];

error_t error;
system_t sys;
enc_error_watcher_t enc_error_watcher;
calib_process_t calib_process;

static inline void updateMotorSpeedEstimate(void)
{
  for (int i = 0; i < 2; i++) {
    int ret = calcMotorSpeed(&motor_real[i], &as5047p[i], &sys, &enc_error_watcher);
    if (ret < 0) {
      p("stop!! speed error");
    }
  }
}

inline void motorProcess_itr(bool motor)
{
  updateADC(motor);
  updateAS5047P(motor);
  setOutputRadianMotor(motor, as5047p[motor].output_radian + enc_offset[motor].final, cmd[motor].out_v_final, getBatteryVoltage(), motor_param[motor].output_voltage_limit);
}

// 7APB 36MHz / 1800 cnt -> 20kHz interrupt -> 1ms cycle
#define INTERRUPT_KHZ_1MS (20)
volatile uint32_t interrupt_timer_cnt = 0, main_loop_remain_counter = 0;
volatile uint32_t system_exec_time_stamp[10] = {0};

static inline void waitForNextMainCycle(void)
{
  main_loop_remain_counter = INTERRUPT_KHZ_1MS - interrupt_timer_cnt;
  while (interrupt_timer_cnt <= INTERRUPT_KHZ_1MS);
  interrupt_timer_cnt = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  // TIM1 : M1
  // TIM8 : M0
  static bool motor_select_toggle = false;
  interrupt_timer_cnt++;
  motor_select_toggle = !motor_select_toggle;

  if (sys.is_starting_mode) {
    updateADC(motor_select_toggle);
    return;
  }

  setLedBlue(false);
  if (isEncoderCalibrationActive()) {
    calibrationProcess_itr(motor_select_toggle);
  } else {
    motorProcess_itr(motor_select_toggle);
  }

  setLedBlue(true);
  sys.task_complete_timer_cnt = htim->Instance->CNT;
}

void waitPowerOnTimeout()
{
  p("reset!!!");
  while (sys.power_enable_cnt > 0) {
    sys.power_enable_cnt--;
    sendCanData();
    sendError(error.id, error.info, error.value);
    HAL_Delay(2);
  }
  HAL_Delay(2);
  HAL_NVIC_SystemReset();
}

// 1KHz
void runMode(void)
{
  if (sys.free_wheel_cnt > 0) {
    sys.free_wheel_cnt--;
    if (sys.free_wheel_cnt == 0) {
      resumePwmOutput();
    }
  }
  system_exec_time_stamp[1] = interrupt_timer_cnt;

  if (sys.manual_offset_radian > M_PI * 2) {
    sys.manual_offset_radian = 0;
  }

  //- 0.5 max spd 0.75
  //+ 3.45 max spd 3.25
  // Manual test by switches may override speed command.
  // Used for quick standalone checks without CAN.
  for (int i = 0; i < 2; i++) {
    if (isPushedSW1()) {
      cmd[i].speed = 40.0;
    } else if (isPushedSW2()) {
      cmd[i].speed = -40.0;
    } else if (isPushedSW3()) {
      cmd[i].speed = 80.0;
      //resumePwmOutput();
    } else if (isPushedSW4()) {
      cmd[i].speed = -80.0;
      //setPwmOutPutFreeWheel();
    }

    speedToOutputVoltage(&pid[i], &motor_real[i], &motor_param[i], &cmd[i]);

    // Output Voltage Override
    if (cmd[i].timeout_cnt > 0) {
      cmd[i].timeout_cnt--;
    }

    if (cmd[i].timeout_cnt == 0) {
      cmd[i].out_v = 0;
    }

    if (sys.free_wheel_cnt > 0) {
      cmd[i].out_v = 0;
    }
  }

  system_exec_time_stamp[2] = interrupt_timer_cnt;
  for (int i = 0; i < 2; i++) {
    setFinalOutputVoltage(&cmd[i], &enc_offset[i], sys.manual_offset_radian);  // select Vq-offset angle
  }

  system_exec_time_stamp[3] = interrupt_timer_cnt;
  printRuntimeDiagnostics();
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  runStartupSequence();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    receiveUserSerialCommand();

    updateMotorSpeedEstimate();
    sendCanData();

    system_exec_time_stamp[0] = interrupt_timer_cnt;
    runControlMode();
    protect();

    setLedRed(true);

    // Wait until next 1ms cycle.
    // Expected idle/load split is roughly balanced in nominal case.
    waitForNextMainCycle();

    setLedRed(false);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_TIM1 | RCC_PERIPHCLK_TIM8 | RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  forceStopAllPwmOutputAndTimer();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t * file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
