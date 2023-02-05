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
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32f3xx_hal_spi.h"
#include "stm32f3xx_hal_can.h"
#include "stm32f3xx_hal_tim.h"
#include "stm32f3xx_hal_uart.h"
#include <math.h>
#include "flash.h"
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
#define UART_TEMP_BUF_SIZE (200)
char first_buf[UART_TEMP_BUF_SIZE];
char temp_buf[UART_TEMP_BUF_SIZE];
int re_queue_len = 0;
bool enable_buffer_mode = false;
int _write(int file, char *ptr, int len)
{	if (enable_buffer_mode){
		enable_buffer_mode = false;

		if (huart1.hdmatx->State == HAL_DMA_BURST_STATE_BUSY)
		{
			if (len >= UART_TEMP_BUF_SIZE)
				len = UART_TEMP_BUF_SIZE;
			memcpy(temp_buf, ptr, len);
			re_queue_len = len;
			return len;
		}
		memcpy(first_buf, ptr, len);						 // 8ms
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)first_buf, len); // 2ms
	}else{
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)ptr, len); // 2ms
	}
	return len;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (re_queue_len){
		
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *)temp_buf, re_queue_len);
		re_queue_len = 0;
	}
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
float manual_offset_radian = 0;
bool calibration_mode = false;

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
typedef struct{
	float final;
	float zero_calib;
}enc_offset_t;

typedef struct
{
	float speed;
	float limit;
	float out_v;
	int timeout_cnt;
}motor_control_cmd_t;
typedef struct
{
	int pre_enc_cnt_raw;
	float rps;
}motor_real_t;


float calib_rotation_speed = -0.005;
motor_control_cmd_t cmd[2];
calib_point_t calib[2];
enc_offset_t enc_offset[2];
motor_real_t motor_real[2];

#define ENC_CNT_MAX (65536)
#define HARF_OF_ENC_CNT_MAX (32768)

void calcMotorSpeed(int motor){
	int temp = motor_real[motor].pre_enc_cnt_raw - ma702[motor].enc_raw;
	if (temp < -HARF_OF_ENC_CNT_MAX){
		temp += ENC_CNT_MAX;
	}
	else if (temp > HARF_OF_ENC_CNT_MAX){
		temp -= ENC_CNT_MAX;
	}
	motor_real[motor].rps = (float)temp / ENC_CNT_MAX * 1000;	//rps
	motor_real[motor]
		.pre_enc_cnt_raw = ma702[motor].enc_raw;
}

void checkAngle(int motor)
{
	calib[motor].radian_ave += ma702[motor].output_radian;
	calib[motor].ave_cnt++;
	if (calib[motor].pre_raw > HARF_OF_ENC_CNT_MAX && ma702[motor].enc_raw < HARF_OF_ENC_CNT_MAX && calib_rotation_speed < 0)
	{
		// ccw
		calib[motor].result_ccw_cnt++;
		calib[motor].result_ccw = calib[motor].radian_ave / calib[motor].ave_cnt;
		calib[motor].radian_ave = 0;
		calib[motor].ave_cnt = 0;
	}
	if (calib[motor].pre_raw < HARF_OF_ENC_CNT_MAX && ma702[motor].enc_raw > HARF_OF_ENC_CNT_MAX && calib_rotation_speed > 0)
	{
		//cw
		calib[motor].result_cw_cnt++;
		calib[motor].result_cw = calib[motor].radian_ave / calib[motor].ave_cnt;
		calib[motor].radian_ave = 0;
		calib[motor].ave_cnt = 0;
	}
	calib[motor].pre_raw = ma702[motor].enc_raw;
}

// 1k -> 20k
inline void calibrationProcess(int motor)
{
	manual_offset_radian -= calib_rotation_speed;

	if (manual_offset_radian > M_PI * 2)
	{
		manual_offset_radian -= M_PI * 2;
		checkAngle(motor);
	}
	if (manual_offset_radian < 0)
	{
		manual_offset_radian += M_PI * 2;
		checkAngle(motor);
	}
	if (motor)
	{
		updateADC_M0();

		updateMA702_M0();

		setOutputRadianM0(manual_offset_radian, cmd[0].out_v, getBatteryVoltage());
	}
	else
	{
		updateADC_M1();

		updateMA702_M1();

		setOutputRadianM1(manual_offset_radian, cmd[1].out_v, getBatteryVoltage());
	}
}

inline void motorProcess(int motor)
{

	if (motor)
	{
		updateADC_M0();

		updateMA702_M0();

		// ->5us
		setOutputRadianM0(ma702[0].output_radian + enc_offset[0].final, cmd[0].out_v, 24);
	}
	else
	{
		updateADC_M1();

		updateMA702_M1();

		setOutputRadianM1(ma702[1].output_radian + enc_offset[1].final, cmd[1].out_v, 24);
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
	setLedBlue(false);
	if (calibration_mode)
	{
		calibrationProcess(motor_select_toggle);
	}
	else
	{
		motorProcess(motor_select_toggle);
	}

	setLedBlue(true);
}

uint32_t can_rx_cnt = 0;
can_rx_buf_t can_rx_buf;
uint8_t can_rx_data[8];
CAN_RxHeaderTypeDef can_rx_header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_buf.data) != HAL_OK)
	{
		/* Reception Error */
		Error_Handler();
	}
	if (calibration_mode){
		return;
	}
		can_rx_cnt++;
	switch (can_rx_header.StdId)
	{
	case 0x100:
		cmd[0].out_v = can_rx_buf.speed / 20;
		cmd[0].timeout_cnt = 0;
		break;

	case 0x101:
		cmd[1].out_v = can_rx_buf.speed / 20;
		cmd[1].timeout_cnt = 0;
		break;

	case 0x102:
		cmd[0].out_v = can_rx_buf.speed / 20;
		cmd[0].timeout_cnt = 0;
		break;
	case 0x103:
		cmd[1].out_v = can_rx_buf.speed / 20;
		cmd[1].timeout_cnt = 0;
		break;
	case 0x300:
		break;
	default:
		break;
	}
}

float motor_accel = 0;
#define ROTATION_OFFSET_RADIAN (2.0)
// by manual tuning

void runMode(void)
{

	if (manual_offset_radian > M_PI * 2)
	{
		manual_offset_radian = 0;
	}
	//- 0.5 max spd 0.75
	//+ 3.45 max spd 3.25


	for (int i = 0; i < 2;i++){

		// select Vq-offset angle
		if (cmd[i].out_v >= 0)
		{
			// 2.4
			enc_offset[i].final = -(ROTATION_OFFSET_RADIAN * 2) + enc_offset[i].zero_calib + manual_offset_radian;
		}
		else
		{
			enc_offset[i].final = enc_offset[i].zero_calib + manual_offset_radian;
		}

		cmd[i].timeout_cnt++;
		if (cmd[i].timeout_cnt > 100)
		{

			if (isPushedSW1())
			{
				cmd[i].out_v = 2.0;
			}
			else if (isPushedSW2())
			{
				cmd[i].out_v = -2.0;
			}
			else
			{
				cmd[i].out_v = 0;
			}
		}
	}


	// ADC raw ALL
	printf("CS M0 %+7.3f M1 %+7.3f / BV %6.3f ", getCurrentM0(), getCurrentM1(), getBatteryVoltage());
	printf("RPS M0 %+6.2f M1 %+6.2f M1offset %4.3f, voltageM0 %+6.3f M1 %6.3f rx %6ld speedM0 %+6.3f\n", motor_real[0].rps,motor_real[1].rps,ma702[0].enc_raw, ma702[1].enc_raw, manual_offset_radian, cmd[0].out_v, cmd[1].out_v, can_rx_cnt, cmd[0].speed);

	//	printf("M0raw %8d M1raw %8d offset %4.3f, voltageM0 %+6.3f M1 %6.3f rx %6ld speedM0 %+6.3f\n", ma702[0].enc_raw, ma702[1].enc_raw, manual_offset_radian, cmd[0].out_v, cmd[1].out_v, can_rx_cnt, cmd[0].speed);
	can_rx_cnt = 0;
}

/* 1ms cycle by 1ms delay*/
void calibrationMode(void)
{
	printf("M0 ave %6.3f cnt %3d M1 ave %6.3f cnt %3d ", calib[0].radian_ave, calib[0].ave_cnt, calib[1].radian_ave, calib[1].ave_cnt);
	printf("result M0 %6.4f M1 %6.4f ", calib[0].result_cw, calib[1].result_cw);
	printf("M0raw %6d M1raw %6d offset %4.3f\n", ma702[0].enc_raw, ma702[1].enc_raw, manual_offset_radian);

	// calib_rotation_speed is minus and CW rotation at 1st-calibration cycle
	if (calib[0].result_cw_cnt > 5 && calib[1].result_cw_cnt > 5 && calib_rotation_speed > 0)
	{
		calibration_mode = true;
		cmd[0].out_v = 2.0;
		cmd[1].out_v = 2.0;
		calib_rotation_speed = -calib_rotation_speed;
	}
	if (calib[0].result_ccw_cnt > 5 && calib[1].result_ccw_cnt > 5)
	{

		cmd[0].out_v = 0;
		cmd[1].out_v = 0;
		HAL_Delay(1); // write out uart buffer

		float temp_offset[2] = {0, 0};

		temp_offset[0] = (M_PI * 2) - ((calib[0].result_ccw + calib[0].result_cw) / 2);
		temp_offset[1] = (M_PI * 2) - ((calib[1].result_ccw + calib[1].result_cw) / 2);
		printf("elec-centor radian : M0 %6f M1 %6f\n", temp_offset[0], temp_offset[1]);
		HAL_Delay(1); // write out uart buffer

		temp_offset[0] += ROTATION_OFFSET_RADIAN;
		if (temp_offset[0] > M_PI * 2)
		{
			temp_offset[0] -= M_PI * 2;
		}
		if (temp_offset[0] < 0)
		{
			temp_offset[0] += M_PI * 2;
		}
		temp_offset[1] += ROTATION_OFFSET_RADIAN;
		if (temp_offset[1] > M_PI * 2)
		{
			temp_offset[1] -= M_PI * 2;
		}
		if (temp_offset[1] < 0)
		{
			temp_offset[1] += M_PI * 2;
		}
		enc_offset[0].zero_calib = temp_offset[0];
		enc_offset[1].zero_calib = temp_offset[1];
		printf("complete calibration!!\nccw %6f cw %6f result user offset M0 %6.3f M1 %6.3f\n", calib[0].result_ccw, calib[0].result_cw, temp_offset[0], temp_offset[1]);

		manual_offset_radian = 0;
		calibration_mode = false;

		cmd[0].out_v = 0;
		cmd[1].out_v = 0;

		calib[0].result_cw_cnt = 0;
		calib[1].result_cw_cnt = 0;
		calib[0].ave_cnt = 0;
		calib[1].ave_cnt = 0;
		calib[0].radian_ave = 0;
		calib[1].radian_ave = 0;

		writeCalibrationValue(enc_offset[0].zero_calib,enc_offset[1].zero_calib);

		HAL_Delay(1000);
	}
}


void startCalibrationMode(void)
{
	printf("calibration mode!\n");

	calibration_mode = true;
	manual_offset_radian = 0;

	cmd[0].out_v = 2.0;
	cmd[1].out_v = 2.0;
	calib_rotation_speed = -calib_rotation_speed;
}

void receiveUserSerialCommand(void){

	if (uart_rx_flag)
	{
		uart_rx_flag = false;
		HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);
		switch (uart_rx_buf[0])
		{
		case 'c':
			startCalibrationMode();
			break;
		case 'n':
			printf("run mode!\n");

			calibration_mode = false;
			manual_offset_radian = 0;

			cmd[0].out_v = 0;
			cmd[1].out_v = 0;
			break;
		case 'q':
			manual_offset_radian += 0.01;
			break;
		case 'a':
			manual_offset_radian -= 0.01;
			break;
		case 'w':
			cmd[0].out_v += 0.5;
			cmd[1].out_v += 0.5;
			break;
		case 's':
			cmd[0].out_v -= 0.5;
			cmd[1].out_v -= 0.5;
			break;
		case 'p':
			motor_accel = 0.5;
			printf("start auto speed!!\n");
			break;
		case 'l':
			motor_accel = 0;
			cmd[0].out_v = 0;
			cmd[1].out_v = 0;
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
	HAL_Delay(100);

	loadFlashData();
	enable_buffer_mode = true;
	printf("** Orion VV driver V1 start! **\n");
	enc_offset[0].zero_calib = flash.calib[0];
	enc_offset[1].zero_calib = flash.calib[1];
	printf("CAN ADDR 0x%03x, enc offset M0 %6.3f M1 %6.3f\n", flash.board_id, flash.calib[0], flash.calib[1]);

	if (isPushedSW1())
	{
		flash.board_id = 0;
		writeCanBoardID(flash.board_id);
		printf("sed board id %d\n", flash.board_id);
		HAL_Delay(1000);
	}
	else if (isPushedSW2())
	{
		flash.board_id = 1;
		writeCanBoardID(flash.board_id);
		printf("sed board id %d\n", flash.board_id);
		HAL_Delay(1000);
	}
	if (isPushedSW4())
	{
		startCalibrationMode();
		printf("calibration mode!!\n");
		while (isPushedSW4())
			;
	}

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

	CAN_Filter_Init(flash.board_id);

	HAL_CAN_Start(&hcan);
	printf("start main loop!\n");

	setLedRed(false);
	setLedGreen(false);
	setLedBlue(false);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		receiveUserSerialCommand();
		calcMotorSpeed(0);
		calcMotorSpeed(1);

		if (calibration_mode)
		{
			calibrationMode();
		}
		else
		{
			runMode();
		}

		if (getCurrentM0() > 3.0  || getCurrentM1() > 3.0)
		{
			forceStop();
			enable_buffer_mode = true;
			printf("over current!! : %d %d\n", adc_raw.cs_m0, adc_raw.cs_m1);
			setLedBlue(false);
			setLedGreen(true);
			setLedRed(true);
			while (1)
				;
		}
		if (getBatteryVoltage() < 20)
		{
			forceStop();
			enable_buffer_mode = true;
			printf("under operation voltaie!! %6.3f", getBatteryVoltage());
			setLedBlue(true);
			setLedGreen(false);
			setLedRed(true);
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
