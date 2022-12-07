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
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int f getc(FILE *f)
#endif

void __io_putchar(uint8_t ch)
{
	HAL_UART_Transmit(&huart1, &ch, 1, 1);
}

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
float rad_to_sin_cnv_array[1024] = {0};
uint16_t enc_raw = 0, enc_elec = 0;
float offset_radian = 0;
float output_radian = 0;
uint16_t rad_mapped = 0, print_mapped = 0;
int diff_cnt = 0, pre_enc_raw = 0;
float output_voltage = 0;

int diff_accel_min = 0, diff_accel_max = 0;

bool calibration_mode = false;

float fast_sin(float rad)
{
	return rad_to_sin_cnv_array[(uint8_t)(rad / (M_PI * 2) * 256)];
}

/*
int voltage_propotional_cnt;
void setOutputPhaseRadian(float out_rad, float voltage)
{
	const int pwm_cnt_centor = 700;
	if (voltage < 0)
	{
		voltage = -voltage;
	}
	if (voltage > 24)
	{
		voltage = 0;
	}
	voltage_propotional_cnt = voltage / 24 * pwm_cnt_centor;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
	uint16_t rad_to_cnt = (uint8_t)((out_rad + M_PI * 4) / (M_PI * 2) * 255);
	htim8.Instance->CCR1 = pwm_cnt_centor + voltage_propotional_cnt * rad_to_sin_cnv_array[rad_to_cnt];
	htim8.Instance->CCR2 = pwm_cnt_centor + voltage_propotional_cnt * rad_to_sin_cnv_array[85 + rad_to_cnt];
	htim8.Instance->CCR3 = pwm_cnt_centor + voltage_propotional_cnt * rad_to_sin_cnv_array[170 + rad_to_cnt];
	htim1.Instance->CCR1 = pwm_cnt_centor + voltage_propotional_cnt * rad_to_sin_cnv_array[rad_to_cnt];
	htim1.Instance->CCR2 = pwm_cnt_centor + voltage_propotional_cnt * rad_to_sin_cnv_array[85 + rad_to_cnt];
	htim1.Instance->CCR3 = pwm_cnt_centor + voltage_propotional_cnt * rad_to_sin_cnv_array[170 + rad_to_cnt];
	print_mapped = rad_mapped;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
}
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t speed_calc_cycle = 0;
	static int32_t pre_diff_cnt = 0, diff_accel;
	if (htim == &htim8)
	{
		// ->
		updateMA702_M0();

		// ->5us
		setOutputRadianTIM8(ma702_0.output_radian + offset_radian, output_voltage,24);

		speed_calc_cycle++;
		if (speed_calc_cycle >= 200)
		{
			speed_calc_cycle = 0;
			int temp_diff_cnt = enc_raw - pre_enc_raw;
			pre_enc_raw = enc_raw;
			if (temp_diff_cnt > 30000)
			{
				diff_cnt = temp_diff_cnt - 65535;
			}
			else if (temp_diff_cnt < -30000)
			{
				diff_cnt = temp_diff_cnt + 65535;
			}
			else
			{
				diff_cnt = temp_diff_cnt;
			}
			diff_accel = diff_cnt - pre_diff_cnt;
			pre_diff_cnt = 0;

			if (diff_accel > diff_accel_max)
			{
				diff_accel_max = diff_accel;
			}
			if (diff_accel < diff_accel_min)
			{
				diff_accel_min = diff_accel;
			}
		}
	}
}

uint32_t can_rx_cnt = 0;
uint8_t can_rx_data[8];
CAN_RxHeaderTypeDef   can_rx_header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_data) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
	can_rx_cnt++;
}

float max_speed_p = 0, max_speed_m = 0;
float max_offset_p = 0, max_offset_m = 0;
float motor_accel = 0;
float user_offet_radian = 0;


static uint16_t adc_raw_array[2] = {0};

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

	if (output_voltage > 0)
	{
		// 2.4
		offset_radian = 2.4 + user_offet_radian;
	}
	else
	{
		offset_radian = 0.0 + user_offet_radian;
	}
	// printf("raw = %4d %4d : %6d\n",raw_rx[0],raw_rx[1],(raw_rx[1]<<8)+raw_rx[0]);
	float spd_rps = (float)diff_cnt / 65535 * 26 * 50;

	if (max_speed_p < spd_rps)
	{
		max_speed_p = spd_rps;
		max_offset_p = offset_radian;
		max_speed_m = 0;
	}
	if (max_speed_m > spd_rps)
	{
		max_speed_m = spd_rps;
		max_offset_m = offset_radian;
		max_speed_p = 0;
	}

	// printf("spd %+7.3f diff %6d, enc %+7.3f\n",spd_rps,diff_cnt,(float)enc_raw/65556);
	// printf("rps = %+7.3f\n",spd_rps);
	// printf("spd %+6d rps = %+7.3f max+ = %+7.3f max- = %+7.3f raw = %6d elec = %6d out %4.3f offset %4.3f maped %3d masked %3d\n",diff_cnt,spd_rps,max_offset_p,max_offset_m,enc_raw,enc_elec,output_radian,offset_radian,print_mapped,print_mapped & 0xFF);
	printf("%8d %8d ",adc_raw_array[0],adc_raw_array[1]);
	printf("raw %6d max %+8d min %+8d rps = %+7.3f offset %4.3f, voltage %+6.3f, rx %8ld \n", enc_raw >> 2, diff_accel_max, diff_accel_min, spd_rps, offset_radian, output_voltage * 2.7,can_rx_cnt);
	diff_accel_max = -5000;
	diff_accel_min = 5000;
}

void calibrationMode(void)
{
	float spd_rps = (float)diff_cnt / 65535 * 26 * 50;

	printf("offset %+10.5f, spd %+10.5f\n", offset_radian, spd_rps);
	offset_radian += 0.05;

	if (offset_radian > M_PI * 2)
	{
		offset_radian = 0;
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

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw_array, 2);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

  printf("Orion VV driver V1 start! \n");

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

	HAL_TIM_PWM_Init(&htim8);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
	HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);

	CAN_Filter_Init(0);

	HAL_CAN_Start(&hcan);
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
				calibration_mode = true;
				user_offet_radian = 0;
				offset_radian = 0;
				output_voltage = 2.0;
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
				HAL_TIM_Base_Stop_IT(&htim1);
				HAL_TIM_Base_Stop_IT(&htim8);
				htim8.Instance->CCR1 = 0;
				htim8.Instance->CCR2 = 0;
				htim8.Instance->CCR3 = 0;
				htim1.Instance->CCR1 = 0;
				htim1.Instance->CCR2 = 0;
				htim1.Instance->CCR3 = 0;
				__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);
				__HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
				while (1)
					;
				break;
			}
		}

		if (calibration_mode)
		{
			calibrationMode();
			HAL_Delay(100);
		}
		else
		{
			runMode();
			HAL_Delay(1);

			can_header.StdId = 0x00;
			can_header.RTR = CAN_RTR_DATA;
			can_header.DLC = 8;
			can_header.TransmitGlobalTime = DISABLE;
			can_data[0] = 0;
			can_data[1] = 0;
			can_data[2] = 1;
			can_data[3] = 1;
			HAL_CAN_AddTxMessage(&hcan,&can_header,can_data,&can_mailbox);
		}
		// printf("spd %+6d rps = %+7.3f /offset max+ = %+7.3f max- = %+7.3f /speed max+ = %+7.3f max- = %+7.3f\n",diff_cnt,spd_rps,max_offset_p,max_offset_m,max_speed_p,max_speed_m);
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM8|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
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
