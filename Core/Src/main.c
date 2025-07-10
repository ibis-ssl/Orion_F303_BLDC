/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "dma.h"
#include "gpio.h"
#include "spi.h"
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

// 1kHz
#define START_UP_FREE_WHEEL_CNT (1000)
#define KICK_FREE_WHEEL_CNT (200)
//400ms NG

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

void sendCanData(void);
void startCalibrationMode();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint32_t ex_can_send_fail_cnt;

uint8_t uart_rx_buf[10] = {0};
bool uart_rx_flag = false;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  uart_rx_flag = true;
}

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


// 7APB 36MHz / 1800 cnt -> 20kHz interrupt -> 1ms cycle
#define INTERRUPT_KHZ_1MS (20)

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  // TIM1 : M1
  // TIM8 : M0
  static bool motor_select_toggle = false;
  motor_select_toggle = !motor_select_toggle;

  updateADC(motor_select_toggle);
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

uint32_t can_rx_cnt = 0;
can_msg_buf_t can_rx_buf;
CAN_RxHeaderTypeDef can_rx_header;
// 50rps x 3.14 x 55mm = 8.635 m/s

static inline float clampSize(float in, float max)
{
  if (in > max) {
    in = max;
  }
  if (in < -max) {
    in = -max;
  }
  return in;
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_buf.data) != HAL_OK) {
    Error_Handler();
    return;
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &can_rx_header, can_rx_buf.data) != HAL_OK) {
    Error_Handler();
    return;
  }
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
  initFirstSin();

  // LED
  setLedRed(true);
  setLedGreen(true);
  setLedBlue(true);

  
  __HAL_SPI_ENABLE(&hspi1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  // 50 / 5 : 10ぐらい
  //
  HAL_Delay(1);

  p("AS5047P registors\n");
  HAL_Delay(1);
  for (int i = 0; i < 2; i++) {
    as5047p[i].reg.error = readRegisterAS5047P(i, 0x0001) & 0x07;       // 0-2 bit, clear error
    as5047p[i].reg.error = readRegisterAS5047P(i, 0x0001) & 0x07;       // 0-2 bit
    as5047p[i].reg.prog = readRegisterAS5047P(i, 0x0003) & 0x7F;        // 0-6bit
    as5047p[i].reg.diagagc = readRegisterAS5047P(i, 0x3FFC) & 0xFFF;    //0-11bit
    as5047p[i].reg.mag = readRegisterAS5047P(i, 0x3FFD) & 0x3FFF;       //0-13bit
    as5047p[i].reg.angleenc = readRegisterAS5047P(i, 0x3FFE) & 0x3FFF;  //0-13bit
    as5047p[i].reg.anglecom = readRegisterAS5047P(i, 0x3FFF) & 0x3FFF;  //0-13bit
    p("err 0x%02x prg 0x%02x diagagc 0x%03x ", as5047p[i].reg.error, as5047p[i].reg.prog, as5047p[i].reg.diagagc);
    p("mag 0x%03x angle : enc 0x%03x com 0x%03x\n", as5047p[i].reg.mag, as5047p[i].reg.angleenc, as5047p[i].reg.anglecom);
    HAL_Delay(1);
  }


  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc3);

  htim1.Instance->CNT = 0;
  htim8.Instance->CNT = 0;

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);


  // M0 : tim1
  // M1 : tim8

  HAL_TIM_PWM_Init(&htim8);
  HAL_TIM_PWM_Init(&htim1);
  setPwmAll(TIM_PWM_CENTER);


  CAN_Filter_Init(flash.board_id);

  HAL_CAN_Start(&hcan);
  p("start main loop!\n");

  setLedRed(false);
  setLedGreen(false);
  setLedBlue(false);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
