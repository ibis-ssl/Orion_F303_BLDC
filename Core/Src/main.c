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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f3xx_hal_spi.h"
#include "stm32f3xx_hal_can.h"
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int f getc(FILE *f)
#endif
*/
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ptr, len);
	return len;
}

/*void __io_putchar(uint8_t ch)
{
	HAL_UART_Transmit(&huart1, &ch, 1, 1);
}*/

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
CAN_TxHeaderTypeDef can_header;
uint8_t can_data[8];
uint32_t can_mailbox;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t uart_rx_buf[10] = {0};
bool uart_rx_flag = false;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_rx_flag = true;
}

// 0 ~ M_PI*2 * 4
uint16_t enc_raw = 0, enc_elec = 0;
float offset_radian = 0;
float output_radian = 0;
uint16_t rad_mapped = 0, print_mapped = 0;
int diff_cnt = 0, pre_enc_raw = 0;
float output_voltage = 0;

bool calibration_mode = false;

typedef struct
{
	int cs_m0;
	int cs_m1;
	int batt_v;
	int temp_m0;
	int temp_m1;
}adc_raw_t;
adc_raw_t adc_raw;

inline float getBatteryVoltage(void){
	return adc_raw.batt_v * 3.3 * 11 / 4096;
}
// 50V/V * 5m = 250mV/A
inline float getCurrentM0(void)
{
	return (adc_raw.cs_m0-2048) * 3.3 / 4096 * 4;
}
inline float getCurrentM1(void)
{
	return (adc_raw.cs_m1-2048) * 3.3 / 4096 * 4;
}


inline void updateADC_M0(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	adc_raw.cs_m0 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
	adc_raw.temp_m0 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
	adc_raw.temp_m1 = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

inline void updateADC_M1(void)
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	adc_raw.batt_v = HAL_ADCEx_InjectedGetValue(&hadc3, ADC_INJECTED_RANK_1);
	adc_raw.cs_m1 = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
	HAL_ADCEx_InjectedStart(&hadc2);
	HAL_ADCEx_InjectedStart(&hadc3);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

typedef struct
{
	float pre_elec_radian;
	float radian_ave;
	int ave_cnt;
	int pre_raw;
	float result_cw;
	int result_cw_cnt;
	float result_ccw;
	int result_ccw_cnt;
} calib_point_t;

float calib_rotation_speed = -0.005;
calib_point_t calib[2];
// board 1
// M0 5.7505 M1 4.7589M
// M0 4.7721 M1 3.7922
// M1 3.2c

// test board M0
// 1.3 / 2.6 -> ave 1.95 -> 4.3
// -0.1 / 2.50  -> 60deg
// + 2.6-2.7 = 4.3 - 1.7
// 4.3
// - 6.0-6.1 = 4.3 + 1.7
void checkAngle(motor)
{
	calib[motor].radian_ave += getElecRadianMA702(motor);
	calib[motor].ave_cnt++;
	if (calib[motor].pre_raw > 63335 / 2 && getRawMA702(motor) < 63335 / 2 && calib_rotation_speed < 0)
	{
		// ccw
		calib[motor].result_ccw_cnt++;
		calib[motor].result_ccw = calib[motor].radian_ave / calib[motor].ave_cnt;
		calib[motor].radian_ave = 0;
		calib[motor].ave_cnt = 0;
	}
	if (calib[motor].pre_raw < 63335 / 2 && getRawMA702(motor) > 63335 / 2 && calib_rotation_speed > 0)
	{
		//cw
		calib[motor].result_cw_cnt++;
		calib[motor].result_cw = calib[motor].radian_ave / calib[motor].ave_cnt;
		calib[motor].radian_ave = 0;
		calib[motor].ave_cnt = 0;
	}
	calib[motor].pre_raw = getRawMA702(motor);
}

// 1k -> 20k
inline void calibrationProcess(int motor)
{
	offset_radian -= calib_rotation_speed;

	if (offset_radian > M_PI * 2)
	{
		offset_radian -= M_PI * 2;
		checkAngle(motor);
	}
	if (offset_radian < 0)
	{
		offset_radian += M_PI * 2;
		checkAngle(motor);
	}
	if (motor)
	{
		updateADC_M0();

		updateMA702_M0();

		setOutputRadianM0(offset_radian, output_voltage, 24);
	}
	else
	{
		updateADC_M1();

		updateMA702_M1();

		setOutputRadianM1(offset_radian, output_voltage, 24);
	}
}

inline void motorProcess(int motor)
{

	if (motor)
	{
		updateADC_M0();

		updateMA702_M0();

		// ->5us
		setOutputRadianM0(getElecRadianMA702(0) + offset_radian, output_voltage, 24);
	}
	else
	{
		updateADC_M1();

		updateMA702_M1();

		setOutputRadianM1(getElecRadianMA702(1) + offset_radian, output_voltage, 24);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// TIM1 : M1
	// TIM8 : M0
	static bool motor_select_toggle = false;

	if (htim == &htim1)
	{
	}
	else if (htim == &htim8)
	{
		return;
	}

	motor_select_toggle = !motor_select_toggle;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	if (calibration_mode)
	{
		calibrationProcess(motor_select_toggle);
	}
	else
	{
		motorProcess(motor_select_toggle);
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
}

uint32_t can_rx_cnt = 0;
uint8_t can_rx_data[8];
CAN_RxHeaderTypeDef can_rx_header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_data) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}
	can_rx_cnt++;
}

float motor_accel = 0;
float user_offet_radian = 0;

void runMode(void)
{

	// offset_radian += 0.01;
	if (offset_radian > M_PI * 2)
	{
		offset_radian = 0;
	}
	//- 0.5 max spd 0.75
	//+ 3.45 max spd 3.25

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == 0)
	{
		output_voltage = 2.0;
	}
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1) == 0)
	{
		output_voltage = -2.0;
	}

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) == 0)
	{
		motor_accel = 0;
		output_voltage = 0;
	}

	output_voltage += motor_accel;
	if (output_voltage > 20.0)
	{
		motor_accel = -0.5;
	}
	if (output_voltage < -20.0)
	{
		motor_accel = 0.5;
	}

	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) == 0)
	{
		motor_accel = 0.1;
		printf("start auto speed!!\n");
		// output_voltage += 0.01;
	}

	if (output_voltage >= 0)
	{
		// 2.4
		offset_radian = -3.4 + user_offet_radian;
	}
	else
	{
		offset_radian = 0.0 + user_offet_radian;
	}

	// ADC raw ALL

	printf("CS M0 %+7.3f M1 %+7.3f / BV %6.3f ", getCurrentM0(), getCurrentM1(), getBatteryVoltage());
	printf("M0raw %8d M1raw %8d offset %4.3f, voltage %+6.3f rx %6ld\n", getRawMA702(0), getRawMA702(1), offset_radian, output_voltage * 2.7, can_rx_cnt);
}

// 2K / page
// 128Kbyte -> 64page
// page : 0~
void writeFlash(void){
	FLASH_EraseInitTypeDef erase;
	uint32_t page_error = 0;
	erase.TypeErase = TYPEERASE_PAGES;
	erase.PageAddress = 0x801F000;
	erase.NbPages = 1;
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&erase, &page_error);
	HAL_FLASH_Lock();

	HAL_FLASH_Unlock();
	float raw = 10.0;
	uint32_t flash_raw;
	memcpy(&flash_raw,&raw,4);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, 0x801F000, flash_raw);
	HAL_FLASH_Lock();
}

/* 1ms cycle by 1ms delay*/
void calibrationMode(void)
{
	printf("M0 ave %6.3f cnt %3d M1 ave %6.3f cnt %3d ", calib[0].radian_ave, calib[0].ave_cnt, calib[1].radian_ave, calib[1].ave_cnt);
	printf("result M0 %6.4f M1 %6.4f ", calib[0].result_cw, calib[1].result_cw);
	printf("M0raw %6d M1raw %6d offset %4.3f\n", getRawMA702(0), getRawMA702(1), offset_radian);

	// calib_rotation_speed is minus and CW rotation at 1st-calibration cycle
	if (calib[0].result_cw_cnt > 5 && calib_rotation_speed > 0)
	{

		calibration_mode = true;
		user_offet_radian = 0;
		offset_radian = 0;
		output_voltage = 2.0;
		calib_rotation_speed = -calib_rotation_speed;
	}
	if (calib[0].result_ccw_cnt > 5 && calib[0].result_cw_cnt > 5)
	{
		output_voltage = 0;
		HAL_Delay(1); // write out uart buffer

		user_offet_radian = (M_PI * 2) - ((calib[0].result_ccw + calib[0].result_cw) / 2);
		printf("elec-centor radian : %6f\n",user_offet_radian);
		HAL_Delay(1); // write out uart buffer

		// IF output_voltage is +, added + M_PI,

		user_offet_radian += 1.7;
		if(user_offet_radian > M_PI * 2){
			user_offet_radian -= M_PI * 2;
		}
		if (user_offet_radian < 0)
		{
			user_offet_radian += M_PI * 2;
		}
		printf("complete calibration!!\nccw %6f cw %6f result user offset %6.3f\n", calib[0].result_ccw ,calib[0].result_cw,user_offet_radian);

		offset_radian = 0;
		calibration_mode = false;

		HAL_Delay(1000);
	}
}

void forceStop(void)
{
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim8);

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

	htim8.Instance->CCR1 = 0;
	htim8.Instance->CCR2 = 0;
	htim8.Instance->CCR3 = 0;
	htim1.Instance->CCR1 = 0;
	htim1.Instance->CCR2 = 0;
	htim1.Instance->CCR3 = 0;

	__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
	__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
}

void trySendCanMsg(void)
{
	can_header.StdId = 0x00;
	can_header.RTR = CAN_RTR_DATA;
	can_header.DLC = 8;
	can_header.TransmitGlobalTime = DISABLE;
	can_data[0] = 0;
	can_data[1] = 0;
	can_data[2] = 1;
	can_data[3] = 1;
	HAL_CAN_AddTxMessage(&hcan, &can_header, can_data, &can_mailbox);
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

	// HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	// HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw_array, 2);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

	printf("Orion VV driver V1 start! ");

	float flash_raw;
	memcpy(&flash_raw, (uint32_t *)0x801F000,4);
	printf(" flash = %f\n", flash_raw);
	user_offet_radian = 1.0;
	writeFlash();
	while (1)
		;

	/*
	  printf("complete convert\n");
	  for(int i=0;i<1024;i++){
		  float temp_rad = (float)i/256*M_PI*2;
		  printf("rad %4.3f sin %4.3f fastsin %4.3f diff %6.5f\n",temp_rad,sin(temp_rad),fast_sin(temp_rad),sin(temp_rad)-fast_sin(temp_rad));
		  HAL_Delay(1);
	  }
	  while(1);*/
	enc_raw = hspi1.Instance->DR;

	__HAL_SPI_ENABLE(&hspi1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_Start(&hadc3);

	HAL_TIM_PWM_Init(&htim8);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

	HAL_TIM_PWM_Init(&htim1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	htim1.Instance->CNT = 0;
	htim8.Instance->CNT = 10;

	HAL_TIM_Base_Start_IT(&htim1);
	// HAL_TIM_Base_Start_IT(&htim8);
	HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);

	CAN_Filter_Init(0);

	HAL_CAN_Start(&hcan);
	printf("start main loop!\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	// offset_radian = 1.8;
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		if (uart_rx_flag)
		{
			uart_rx_flag = false;
			HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);
			switch (uart_rx_buf[0])
			{
			case 'c':
				printf("calibration mode!\n");

				calibration_mode = true;
				user_offet_radian = 0;
				offset_radian = 0;
				output_voltage = 2.0;
				calib_rotation_speed = -calib_rotation_speed;
				break;
			case 'n':
				printf("run mode!\n");

				calibration_mode = false;
				offset_radian = 0;
				output_voltage = 0;
				user_offet_radian = 0;
				break;
			case 'q':
				user_offet_radian += 0.05;
				break;
			case 'a':
				user_offet_radian -= 0.05;
				break;
			case 'w':
				output_voltage += 0.5;
				break;
			case 's':
				output_voltage -= 0.5;
				break;
			case 'p':
				motor_accel = 0.5;
				printf("start auto speed!!\n");
				break;
			case 'l':
				motor_accel = 0;
				output_voltage = 0;
				printf("stop auto speed!!\n");
				break;
			case '0':
				printf("enter sleep!\n");
				forceStop();
				while (1)
					;
				break;
			}
		}

		if (calibration_mode)
		{
			calibrationMode();
		}
		else
		{
			runMode();
		}

		if (getCurrentM0() > 3.0 /* || getCurrentM1() > 3.0*/)
		{
			forceStop();
			printf("over current!! : %d %d\n", adc_raw.cs_m0, adc_raw.cs_m1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
			while (1)
				;
		}
		if (getBatteryVoltage() < 20)
		{
			forceStop();
			printf("under operation voltaie!! %6.3f", getBatteryVoltage());
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
			while (1)
				;
		}
		HAL_Delay(1);
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM8;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
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
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
