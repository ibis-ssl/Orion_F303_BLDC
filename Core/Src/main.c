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
int _write(int file, char *ptr, int len){
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

int diff_accel_min = 0, diff_accel_max = 0;

bool calibration_mode = false;

int adc_raw_cs_m0,adc_raw_cs_m1,adc_raw_batt_v,adc_raw_temp_m0,adc_raw_temp_m1;
int pre_cs_m0;

inline void updateADC_M0(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	adc_raw_cs_m0 = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);
	adc_raw_temp_m0 = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);
	adc_raw_temp_m1 = HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3);
	HAL_ADCEx_InjectedStart(&hadc1);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}


inline void updateADC_M1(void){
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	pre_cs_m0 = adc_raw_cs_m0;
	adc_raw_batt_v = HAL_ADCEx_InjectedGetValue(&hadc3,ADC_INJECTED_RANK_1);
	adc_raw_cs_m1 = HAL_ADCEx_InjectedGetValue(&hadc2,ADC_INJECTED_RANK_1);
	HAL_ADCEx_InjectedStart(&hadc2);
	HAL_ADCEx_InjectedStart(&hadc3);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
}

typedef struct
{
	int pre_angle_raw;
	float angle_ave;
	int ave_cnt;
	float result;
	bool result_setted;
} calib_point_t;

calib_point_t calib[2];

void checkAngle(int motor){
	if (getRadianM702(motor) > getPreRawM702(motor)){
		calib[motor].angle_ave = getRadianM702(motor);
		calib[motor].ave_cnt++;
		if(calib[motor].pre_angle_raw > getRawM702(motor)){

		}
		calib[motor].pre_angle_raw = getRawM702(motor);

	}
}

void calibrationProcess(int motor){
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

void motorProcess(int motor){

	if (motor)
	{

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
		updateADC_M0();
		// ->

		updateMA702_M0();

		// ->5us
		setOutputRadianM0(getRadianM702(0) + offset_radian, output_voltage, 24);

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	}
	else
	{

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
		updateADC_M1();

		updateMA702_M1();

		setOutputRadianM1(getRadianM702(1) + offset_radian, output_voltage, 24);

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);
	}
}

static float zero_angle_enc = 0,zero_angle_ave = 0;
static int pre_zero_angle_enc_raw = 0,zero_angle_cnt = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// TIM1 : M1
	// TIM8 : M0
	static bool control_toggle = false;
	static float dummy_output_radian = 0;

	if (htim == &htim1)
	{
	}else if(htim == &htim8){
		return;
	}

	control_toggle = !control_toggle;
	if (calibration_mode){
		calibrationProcess(control_toggle);
	}else{
		motorProcess(control_toggle);
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
		offset_radian = -2.4 + user_offet_radian;
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

	// ADC raw ALL
	//printf("CS M0 %6d M1 %6d / BV %6d Temp %6d %6d ", adc_raw_cs_m0, adc_raw_cs_m1, adc_raw_batt_v, adc_raw_temp_m0, adc_raw_temp_m1);
	
	printf("E0 %+4d E0 %+4d zero %6.3f", getPreRawM702(0) - getRawM702(0), getPreRawM702(1) - getRawM702(1), zero_angle_enc);
	printf("CS M0 %6d M1 %6d / BV %6d ", adc_raw_cs_m0, adc_raw_cs_m1, adc_raw_batt_v);
	printf("M0raw %8d M1raw %8d rps %+6.2f offset %4.3f, voltage %+6.3f rx %6ld\n", getRawM702(0),getRawM702(1), spd_rps, offset_radian, output_voltage * 2.7,can_rx_cnt);
	diff_accel_max = -5000;
	diff_accel_min = 5000;
}

void calibrationMode(void)
{
	offset_radian += 0.01;

	if (offset_radian > M_PI * 2)
	{
		offset_radian -= M_PI*2;
	}
	printf("E0 %+4d E0 %+4d zero %6.3f\n", getPreRawM702(0) - getRawM702(0), getPreRawM702(1) - getRawM702(1), zero_angle_enc);
}

void forceStop(void){
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

void trySendCanMsg(void){
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

  //HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  //HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_raw_array, 2);
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

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

	htim1.Instance->CNT = 0;
	htim8.Instance->CNT = 1000;

	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim8);
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

		if (adc_raw_cs_m0 > 3000 /* || adc_raw_cs_m1 > 3000*/)
		{
			forceStop();
			printf("over current!! : %d %d\n", adc_raw_cs_m0, adc_raw_cs_m1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
			while (1)
				;
		}
		HAL_Delay(1);
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
