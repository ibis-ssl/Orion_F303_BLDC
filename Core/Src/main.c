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
#include "stm32f3xx_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// 1kHz
#define START_UP_FREE_WHEEL_CNT (1000)
#define KICK_FREE_WHEEL_CNT (500)

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
extern uint32_t can_send_fail_cnt;

uint8_t uart_rx_buf[10] = {0};
bool uart_rx_flag = false;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart) { uart_rx_flag = true; }

uint32_t free_wheel_cnt = START_UP_FREE_WHEEL_CNT;

// 0 ~ M_PI*2 * 4
float manual_offset_radian = 0;
struct
{
  uint32_t enc_calib_cnt;
  uint32_t motor_calib_cnt;  // disable : 0, init : 5000, start : 3500~ , end : 1
  uint32_t motor_calib_mode;
  float motor_calib_voltage;
} calib_process;

float calib_force_rotation_speed = 0.005;
motor_control_cmd_t cmd[2];
calib_point_t calib[2];
enc_offset_t enc_offset[2];
motor_real_t motor_real[2];
motor_param_t motor_param[2];
volatile uint32_t power_enable_cnt = 0;

uint16_t error_id, error_info;
float error_value;

// 200kV -> 3.33rps/V -> 0.3 V/rps
//#define RPS_TO_MOTOR_EFF_VOLTAGE (0.15)
// 13rps : 2.4V

#define V_PER_RPS_DEFAULT (0.15)

#define ENC_CNT_MAX (65536)
#define HARF_OF_ENC_CNT_MAX (32768)

// by manual tuned
#define ROTATION_OFFSET_RADIAN (2.0)

#define MOTOR_OVER_LOAD_CNT_LIMIT (3000)

#define THR_BATTERY_UNVER_VOLTAGE (20.0)
#define THR_BATTERY_OVER_VOLTAGE (30.0)
#define THR_MOTOR_OVER_TEMPERATURE (70)  // 80 -> 70 deg (実機テストによる)

#define MOTOR_CALIB_INIT_CNT (2500)
#define MOTOR_CALIB_READY_CNT (2000)
#define MOTOR_CALIB_START_CNT (1500)
#define MOTOR_CALIB_VOLTAGE_LOW (3.0)
#define MOTOR_CALIB_VOLTAGE_HIGH (10.0)
#define MOTOR_CALIB_CYCLE (12)

#define MOTOR_CALIB_M0_M1_ERROR_TRERANCE (1.0)
#define MOTOR_CALIB_CW_CCW_ERROR_TRERANCE (1.0)
#define MOTOR_CALIB_UNDER_LIMIT (1.0)

#define SPEED_CMD_LIMIT_RPS (50)
// ヤバい速度指令を無視するしきい値
#define SPEED_REAL_LIMIT_GAIN (float)(1.5)
// エンコーダー飛んだ時に異常な速度を検出するしきい値

volatile bool calibration_print_flag = false;
volatile bool enc_over_speed_cnt_error_flag = false;
volatile int enc_over_speed_cnt_error_enc_idx = 0;
volatile int enc_over_speed_cnt_error_enc_cnt = 0;

void calcMotorSpeed(int motor)
{
  int temp = motor_real[motor].pre_enc_cnt_raw - ma702[motor].enc_raw;
  if (temp < -HARF_OF_ENC_CNT_MAX) {
    temp += ENC_CNT_MAX;
  } else if (temp > HARF_OF_ENC_CNT_MAX) {
    temp -= ENC_CNT_MAX;
  }

  if (abs(motor_real[motor].diff_cnt_max) < abs(temp)) {
    motor_real[motor].diff_cnt_max = temp;
  }

  // 異常な回転数の場合に無視
  if (abs((float)temp / ENC_CNT_MAX * 1000) > SPEED_CMD_LIMIT_RPS * 1.5 && free_wheel_cnt == 0) {
    setPwmOutPutFreeWheel();
    // forceStopAllPwmOutputAndTimerだとtim自体も止めるのでフリー回転のみ
    enc_over_speed_cnt_error_flag = true;
    enc_over_speed_cnt_error_enc_idx = 0;
    enc_over_speed_cnt_error_enc_cnt = temp;
    return;
  }

  // motor_real[motor].rps = ((float)temp / ENC_CNT_MAX * 1000) * motor_real[motor].k + (1-motor_real[motor].k) * motor_real[motor].pre_rps; // rps
  motor_real[motor].rps = (float)temp / ENC_CNT_MAX * 1000;
  motor_real[motor].pre_rps = motor_real[motor].rps;
  motor_real[motor].pre_enc_cnt_raw = ma702[motor].enc_raw;
}

void checkAngleCalibMode(int motor)
{
  calib[motor].xy_field.radian_ave_x += cos(ma702[motor].output_radian);
  calib[motor].xy_field.radian_ave_y += sin(ma702[motor].output_radian);
  calib[motor].ave_cnt++;
  if (calib[motor].pre_raw > HARF_OF_ENC_CNT_MAX && ma702[motor].enc_raw < HARF_OF_ENC_CNT_MAX && calib_force_rotation_speed > 0) {
    // ccw
    calibration_print_flag = true;
    calib[motor].result_ccw_cnt++;
    calib[motor].xy_field.result_cw_x = calib[motor].xy_field.radian_ave_x / calib[motor].ave_cnt;
    calib[motor].xy_field.result_cw_y = calib[motor].xy_field.radian_ave_y / calib[motor].ave_cnt;
    calib[motor].xy_field.radian_ave_x = 0;
    calib[motor].xy_field.radian_ave_y = 0;
    calib[motor].ave_cnt = 0;
  }
  if (calib[motor].pre_raw < HARF_OF_ENC_CNT_MAX && ma702[motor].enc_raw > HARF_OF_ENC_CNT_MAX && calib_force_rotation_speed < 0) {
    // cw
    calibration_print_flag = true;
    calib[motor].result_cw_cnt++;
    calib[motor].xy_field.result_ccw_x = calib[motor].xy_field.radian_ave_x / calib[motor].ave_cnt;
    calib[motor].xy_field.result_ccw_y = calib[motor].xy_field.radian_ave_y / calib[motor].ave_cnt;
    calib[motor].xy_field.radian_ave_x = 0;
    calib[motor].xy_field.radian_ave_y = 0;
    calib[motor].ave_cnt = 0;
  }
  calib[motor].pre_raw = ma702[motor].enc_raw;
}

inline void calibrationProcess(int motor)
{
  manual_offset_radian += calib_force_rotation_speed;

  if (manual_offset_radian > M_PI * 2) {
    manual_offset_radian -= M_PI * 2;
    checkAngleCalibMode(motor);
  }
  if (manual_offset_radian < 0) {
    manual_offset_radian += M_PI * 2;
    checkAngleCalibMode(motor);
  }
  if (motor) {
    updateADC_M0();

    updateMA702_M0();
    setOutputRadianM0(manual_offset_radian, cmd[0].out_v_final, getBatteryVoltage(), MOTOR_CALIB_VOLTAGE_HIGH);
  } else {
    updateADC_M1();

    updateMA702_M1();

    setOutputRadianM1(manual_offset_radian, cmd[1].out_v_final, getBatteryVoltage(), MOTOR_CALIB_VOLTAGE_HIGH);
  }
}

inline void motorProcess(int motor)
{
  if (motor) {
    updateADC_M0();

    updateMA702_M0();

    // ->5us
    setOutputRadianM0(ma702[0].output_radian + enc_offset[0].final, cmd[0].out_v_final, getBatteryVoltage(), motor_param[0].output_voltage_limit);
  } else {
    updateADC_M1();

    updateMA702_M1();

    setOutputRadianM1(ma702[1].output_radian + enc_offset[1].final, cmd[1].out_v_final, getBatteryVoltage(), motor_param[1].output_voltage_limit);
  }
}

// 7APB 36MHz / 1800 cnt -> 20kHz interrupt -> 1ms cycle
#define INTERRUPT_KHZ_1MS (20)
volatile uint32_t interrupt_timer_cnt = 0, main_loop_remain_counter = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim)
{
  // TIM1 : M1
  // TIM8 : M0
  static bool motor_select_toggle = false;
  interrupt_timer_cnt++;

  motor_select_toggle = !motor_select_toggle;
  setLedBlue(false);
  if (calib_process.enc_calib_cnt != 0) {
    calibrationProcess(motor_select_toggle);
  } else {
    motorProcess(motor_select_toggle);
  }

  setLedBlue(true);
}

void waitPowerOnTimeout()
{
  p("reset!!!");
  while (power_enable_cnt > 0) {
    power_enable_cnt--;
    sendCanData();
    sendError(0, error_id, error_info, error_value);
    HAL_Delay(2);
  }
  HAL_Delay(2);
  HAL_NVIC_SystemReset();
}

uint32_t can_rx_cnt = 0;
can_msg_buf_t can_rx_buf;
CAN_RxHeaderTypeDef can_rx_header;
// 50rps x 3.14 x 55mm = 8.635 m/s

void can_rx_callback(void)
{
  float tmp_speed = 0;
  if (calib_process.enc_calib_cnt != 0 || calib_process.motor_calib_cnt != 0) {
    return;
  }
  can_rx_cnt++;
  tmp_speed = can_rx_buf.value;
  if (tmp_speed > SPEED_CMD_LIMIT_RPS) {
    tmp_speed = SPEED_CMD_LIMIT_RPS;
  } else if (tmp_speed < -SPEED_CMD_LIMIT_RPS) {
    tmp_speed = -SPEED_CMD_LIMIT_RPS;
  }
  switch (can_rx_header.StdId) {
    case 0x010:
      if (can_rx_buf.data[0] == 0) {
        if (can_rx_buf.data[1] == 0) {
          HAL_NVIC_SystemReset();
        } else if (can_rx_buf.data[1] == 1) {
          power_enable_cnt = 100;
        }
      }
      break;
    case 0x100:
      cmd[0].speed = tmp_speed;
      cmd[0].timeout_cnt = 100;
      break;

    case 0x101:
      cmd[1].speed = tmp_speed;
      cmd[1].timeout_cnt = 100;
      break;

    case 0x102:
      cmd[0].speed = tmp_speed;
      cmd[0].timeout_cnt = 100;
      break;
    case 0x103:
      cmd[1].speed = tmp_speed;
      cmd[1].timeout_cnt = 100;
      break;
    case 0x310:
      startCalibrationMode();
      break;
    case 0x320:
      // not used
      break;
    case 0x110:
      if (can_rx_buf.data[0] == 3) {
        setPwmOutPutFreeWheel();
        free_wheel_cnt = KICK_FREE_WHEEL_CNT;
      }
      break;
    default:
      break;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &can_rx_header, can_rx_buf.data) != HAL_OK) {
    Error_Handler();
    return;
  }
  can_rx_callback();
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &can_rx_header, can_rx_buf.data) != HAL_OK) {
    Error_Handler();
    return;
  }
  can_rx_callback();
}

typedef struct
{
  float eff_voltage;
  float pid_kp, pid_kd, pid_ki;
  float error, error_integral, error_diff;
  float error_integral_limit;
  float pre_real_rps;
  float diff_voltage_limit;
  int load_limit_cnt;
} motor_pid_control_t;
motor_pid_control_t pid[2];

void speedToOutputVoltage(int motor)
{
  pid[motor].eff_voltage = motor_real[motor].rps * motor_param[motor].voltage_per_rps;
  /*pid[motor].error = cmd[motor].speed - motor_real[motor].rps;

  pid[motor].error_integral += pid[motor].error;
  if (pid[motor].error_integral > pid[motor].error_integral_limit) {
    pid[motor].error_integral = pid[motor].error_integral_limit;
  } else if (pid[motor].error_integral < -pid[motor].error_integral_limit) {
    pid[motor].error_integral = -pid[motor].error_integral_limit;
  }

  pid[motor].error_diff = motor_real[motor].rps - pid[motor].pre_real_rps;
  pid[motor].pre_real_rps = motor_real[motor].rps;*/

  cmd[motor].out_v = cmd[motor].speed * motor_param[motor].voltage_per_rps;

  // ローカルでの速度制御はしない(上位からトルクで制御したいため)
  //        +pid[motor].error_diff * pid[motor].pid_kp + pid[motor].error_integral * pid[motor].pid_ki + pid[motor].error_diff * pid[motor].pid_kd; // PID

  // 出力電圧リミット

  float output_voltage_diff = cmd[motor].out_v - pid[motor].eff_voltage;
  if (output_voltage_diff > +pid[motor].diff_voltage_limit) {
    cmd[motor].out_v = pid[motor].eff_voltage + pid[motor].diff_voltage_limit;
  } else if (output_voltage_diff < -pid[motor].diff_voltage_limit) {
    cmd[motor].out_v = pid[motor].eff_voltage - pid[motor].diff_voltage_limit;
  }

  // 過負荷カウント
  if (output_voltage_diff > +pid[motor].error_integral_limit) {
    if (fabs(motor_real[motor].rps) < fabs(cmd[motor].speed)) {
      pid[motor].load_limit_cnt++;
    }
  } else if (output_voltage_diff < -pid[motor].error_integral_limit) {
    if (fabs(motor_real[motor].rps) < fabs(cmd[motor].speed)) {
      pid[motor].load_limit_cnt++;
    }
  } else if (pid[motor].load_limit_cnt > 0) {
    pid[motor].load_limit_cnt--;
  }
}

void setFinalOutputVoltage(int motor)
{
  cmd[motor].out_v_final = cmd[motor].out_v;
  if (cmd[motor].out_v_final < 0) {
    // 2.4
    enc_offset[motor].final = -(ROTATION_OFFSET_RADIAN * 2) + enc_offset[motor].zero_calib + manual_offset_radian;
  } else {
    enc_offset[motor].final = enc_offset[motor].zero_calib + manual_offset_radian;
  }
}

// 1KHz
void runMode(void)
{
  if (free_wheel_cnt > 0) {
    free_wheel_cnt--;
    if (free_wheel_cnt == 0) {
      resumePwmOutput();
    }
  }

  if (manual_offset_radian > M_PI * 2) {
    manual_offset_radian = 0;
  }

  //- 0.5 max spd 0.75
  //+ 3.45 max spd 3.25

  for (int i = 0; i < 2; i++) {
    if (isPushedSW1()) {
      cmd[i].speed = 10.0;
    } else if (isPushedSW2()) {
      cmd[i].speed = -10.0;
    } else if (isPushedSW3()) {
      //
      cmd[i].speed = -20.0;
    } else if (isPushedSW4()) {
      cmd[i].speed = -40.0;
    }

    speedToOutputVoltage(i);

    // Output Voltage Override
    if (cmd[i].timeout_cnt > 0) {
      cmd[i].timeout_cnt--;
    }

    if (cmd[i].timeout_cnt == 0) {
      cmd[i].out_v = 0;
    }

    if (free_wheel_cnt > 0) {
      cmd[i].out_v = 0;
    }

    setFinalOutputVoltage(i);  // select Vq-offset angle
  }

  static uint8_t print_cnt = 0;
  // ここは1KHzでまわっている

  // 1サイクルごとの負荷を減らすために分割して送信
  print_cnt++;
  switch (print_cnt) {
    case 1:
      // p("M0raw %6d M1raw %6d ", ma702[0].enc_raw, ma702[1].enc_raw);
      p("CS %+5.2f %+5.2f / BV %4.1f ", getCurrentM0(), getCurrentM1(), getBatteryVoltage());
      // p("P %+3.1f I %+3.1f D %+3.1f ", pid[0].pid_kp, pid[0].pid_ki, pid[0].pid_kd);
      break;
    case 2:
      p("RPS %+6.1f %+6.1f Free %4d ", motor_real[0].rps, motor_real[1].rps, free_wheel_cnt);
      break;
    case 3:
      p("RAW %5d %5d Out_v %+5.1f %5.1f ", ma702[0].enc_raw, ma702[1].enc_raw, cmd[0].out_v, cmd[1].out_v);
      break;
    case 4:
      //p("p%+3.1f i%+3.1f d%+3.1f k%+3.1f ", pid[0].pid_kp, pid[0].pid_ki, pid[0].pid_kd, motor_real[0].k);
      p("Rx %4ld CPU %3d GD %4.1f ", can_rx_cnt, main_loop_remain_counter, getGateDriverDCDCVoltage());
      can_rx_cnt = 0;
      break;
    case 5:
      // FET温度はv4.2で取得できない
      // getTempFET0(), getTempFET1(),
      p("T %+4d %+4d ", getTempM0(), getTempM1());
      //p("SPD %+6.1f %+6.1f canErr 0x%04x ", cmd[0].speed, cmd[1].speed, getCanError());
      break;
    case 6:
      p("LoadV %+5.2f %+5.2f CanFail %4d ", cmd[0].out_v - pid[0].eff_voltage, cmd[1].out_v - pid[1].eff_voltage, can_send_fail_cnt);
      can_send_fail_cnt = 0;
      break;
    case 7:
      p("LoadCnt %3.2f %3.2f ", (float)pid[0].load_limit_cnt / MOTOR_OVER_LOAD_CNT_LIMIT, (float)pid[1].load_limit_cnt / MOTOR_OVER_LOAD_CNT_LIMIT);
      break;
    case 8:
      p("TO %4d %4d diff max M0 %+6d, M1 %+6d ", cmd[0].timeout_cnt, cmd[1].timeout_cnt, motor_real[0].diff_cnt_max, motor_real[1].diff_cnt_max);
      // p("min %+6d cnt %6d / max %+6d cnt %6d ", ma702[0].diff_min, ma702[0].diff_min_cnt, ma702[0].diff_max, ma702[0].diff_max_cnt);
      motor_real[0].diff_cnt_max = 0;
      motor_real[1].diff_cnt_max = 0;
      ma702[0].diff_max = 0;
      ma702[0].diff_min = 65535;
      ma702[1].diff_max = 0;
      ma702[1].diff_min = 65535;
      break;
    default:
      p("\n");
      print_cnt = 0;
      break;
  }
  // ADC raw ALL
  //	p("M0raw %8d M1raw %8d offset %4.3f, voltageM0 %+6.3f M1 %6.3f rx %6ld speedM0 %+6.3f\n", ma702[0].enc_raw, ma702[1].enc_raw, manual_offset_radian, cmd[0].out_v, cmd[1].out_v, can_rx_cnt, cmd[0].speed);
}

/* Can't running 1kHz */
void encoderCalibrationMode(void)
{
  // 角度0のときにprint
  if (calibration_print_flag) {
    calibration_print_flag = false;
    p("enc = %+5.2f %+5.2f  / M0 X %+5.2f Y %+5.2f / M1 X %+5.2f Y %+5.2f / Rad %+5.2f %+5.2f\n", ma702[0].output_radian, ma702[1].output_radian, cos(ma702[0].output_radian),
      sin(ma702[0].output_radian), cos(ma702[1].output_radian), sin(ma702[1].output_radian), atan2(sin(ma702[0].output_radian), cos(ma702[0].output_radian)),
      atan2(sin(ma702[1].output_radian), cos(ma702[1].output_radian)));
  }

  // calib_force_rotation_speedが+でCCW
  // calib_force_rotation_speedが-でCW回転する。
  // 初期値はcalib_force_rotation_speedが+でCCWからキャリブレーション。

  // END of 1st-calibration cycle (CCW)
  if (calib[0].result_ccw_cnt > MOTOR_CALIB_CYCLE && calib[1].result_ccw_cnt > MOTOR_CALIB_CYCLE && calib_force_rotation_speed > 0) {
    calib_force_rotation_speed = -calib_force_rotation_speed;  //CCW方向終わったので、回転方向反転
    HAL_Delay(1);                                              // write out uart buffer
  }

  // END of 2nd-calibration cycle (CW)
  if (calib[0].result_cw_cnt > MOTOR_CALIB_CYCLE && calib[1].result_cw_cnt > MOTOR_CALIB_CYCLE) {
    // 強制転流モード完了
    cmd[0].out_v_final = 0;
    cmd[1].out_v_final = 0;
    HAL_Delay(1);  // write out uart buffer

    float xy_field_ave_x[2] = {0}, xy_field_ave_y[2] = {0}, xy_field_offset_radian[2] = {0, 0};

    for (int i = 0; i < 2; i++) {
      p("Motor : %d\n", i);
      p("CW X %+5.2f Y %+5.2f\n", calib[i].xy_field.result_cw_x, calib[i].xy_field.result_cw_y);
      p("CCW X %+5.2f Y %+5.2f\n", calib[i].xy_field.result_ccw_x, calib[i].xy_field.result_ccw_y);
      p("CW rad %+5.2f\n", atan2(calib[i].xy_field.result_cw_y, calib[i].xy_field.result_cw_x));
      p("CCW rad %+5.2f\n", atan2(calib[i].xy_field.result_ccw_y, calib[i].xy_field.result_ccw_x));

      xy_field_ave_x[i] = calib[i].xy_field.result_cw_x + calib[i].xy_field.result_ccw_x;
      xy_field_ave_y[i] = calib[i].xy_field.result_cw_y + calib[i].xy_field.result_ccw_y;
      xy_field_offset_radian[i] = (2 * M_PI) - atan2(xy_field_ave_y[i], xy_field_ave_x[i]);
      p("CW+CCW X %+5.2f Y %+5.2f\n", xy_field_ave_x[i], xy_field_ave_y[i]);

      xy_field_offset_radian[i] += ROTATION_OFFSET_RADIAN;
      if (xy_field_offset_radian[i] > M_PI * 2) {
        xy_field_offset_radian[i] -= M_PI * 2;
      }
      if (xy_field_offset_radian[i] < 0) {
        xy_field_offset_radian[i] += M_PI * 2;
      }

      p("Rad M0 %+5.2f\n\n", xy_field_offset_radian[i]);

      // モーターキャリブレーション前に更新
      enc_offset[i].zero_calib = xy_field_offset_radian[i];

      // モーターキャリブレーション結果を書き込むときに、メモリ上のエンコーダーキャリブレーション値も更新しないと戻ってしまう??
      flash.calib[i] = enc_offset[i].zero_calib;

      calib[i].result_cw_cnt = 0;
      calib[i].ave_cnt = 0;
      cmd[i].out_v_final = 0;
    }

    writeEncCalibrationValue(enc_offset[0].zero_calib, enc_offset[1].zero_calib);

    for (int i = 0; i < 2; i++) {
      // モーターキャリブレーション前に更新
      enc_offset[i].zero_calib = xy_field_offset_radian[i];

      // モーターキャリブレーション結果を書き込むときに、メモリ上のエンコーダーキャリブレーション値も更新しないと戻ってしまう??
      flash.calib[i] = xy_field_offset_radian[i];
    }

    // エンコーダキャリブレーション完了
    calib_process.enc_calib_cnt = 0;
    manual_offset_radian = 0;  // 割り込みの中で加算してしまうので,enc_calib_cnt = 0にしてからでないといけない

    // モーターキャリブレーションに切り替え
    calib_process.motor_calib_mode = 0;
    calib_process.motor_calib_cnt = 1;

    p("calib %+5.2f %+5.2f", enc_offset[0].zero_calib, enc_offset[1].zero_calib);
    HAL_Delay(900);
  }
}

bool checkMotorRpsError(float m0, float m1)
{
  if (fabs(m0 - m1) > MOTOR_CALIB_M0_M1_ERROR_TRERANCE) {
    p("\n\nCALIBRATION ERROR!!!\n\n");
    calib_process.motor_calib_cnt = 0;
    return true;
  }
  if (m0 < MOTOR_CALIB_UNDER_LIMIT && m1 < MOTOR_CALIB_UNDER_LIMIT) {
    p("\n\nCALIBRATION ERROR!!!\n\n");
    calib_process.motor_calib_cnt = 0;
    return true;
  }
  return false;
}

bool checkMotorRpsHighLowError(float m0_cw, float m1_cw, float m0_ccw, float m1_ccw)
{
  if (fabs(m0_cw - m0_ccw) > MOTOR_CALIB_CW_CCW_ERROR_TRERANCE || fabs(m1_cw - m1_ccw) > MOTOR_CALIB_CW_CCW_ERROR_TRERANCE) {
    p("\n\nCALIBRATION ERROR!!! CW-CCW PARAM UNMATCH\n\n");
    calib_process.motor_calib_cnt = 0;
    return true;
  }
  return false;
}

void motorCalibrationMode(void)
{
  if (calib_process.motor_calib_cnt > 1) {
    calib_process.motor_calib_cnt--;
  }

  // Output Voltage Override
  for (int i = 0; i < 2; i++) {
    if (calib_process.motor_calib_cnt > 1) {
      if (calib_process.motor_calib_cnt < MOTOR_CALIB_START_CNT) {
        calib[i].rps_integral += motor_real[i].rps;
      }

      // そのまま逆転させるとエラいことになるので一度電圧0にする
      if (calib_process.motor_calib_cnt < MOTOR_CALIB_READY_CNT) {
        cmd[i].out_v = calib_process.motor_calib_voltage;
      } else {
        cmd[i].out_v = 0;
      }
    } else if (calib_process.motor_calib_cnt == 1) {
      cmd[i].out_v = 0;
    }

    setFinalOutputVoltage(i);  // select Vq-offset angle
  }

  if (calib_process.motor_calib_cnt == 1) {
    static float rps_per_v_cw_l[2], rps_per_v_ccw_l[2], rps_per_v_cw_h[2], rps_per_v_ccw_h[2];

    switch (calib_process.motor_calib_mode) {
      case 0:
        p("\n\nstart motor calib!!\n\n");
        calib_process.motor_calib_cnt = MOTOR_CALIB_INIT_CNT;

        calib_process.motor_calib_mode = 1;
        calib_process.motor_calib_voltage = MOTOR_CALIB_VOLTAGE_LOW;
        calib[0].rps_integral = 0;
        calib[1].rps_integral = 0;
        p("set output V = %f\n", calib_process.motor_calib_voltage);
        break;

      case 1:
        rps_per_v_cw_l[0] = calib[0].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        rps_per_v_cw_l[1] = calib[1].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        calib[0].rps_integral = 0;
        calib[1].rps_integral = 0;
        p("\n\nMotor Calib rps/v \n M0 %6.2f\n M1 %6.2f\n\n", rps_per_v_cw_l[0], rps_per_v_cw_l[1]);

        if (checkMotorRpsError(rps_per_v_cw_l[0], rps_per_v_cw_l[1])) {
          return;
        }

        calib_process.motor_calib_cnt = MOTOR_CALIB_INIT_CNT;
        calib_process.motor_calib_mode++;
        calib_process.motor_calib_voltage = -MOTOR_CALIB_VOLTAGE_LOW;
        p("set output V = %f\n", calib_process.motor_calib_voltage);
        break;

      case 2:
        rps_per_v_ccw_l[0] = calib[0].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        rps_per_v_ccw_l[1] = calib[1].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        calib[0].rps_integral = 0;
        calib[1].rps_integral = 0;
        p("\n\nMotor Calib rps/v \n M0 %6.2f\n M1 %6.2f\n\n", rps_per_v_ccw_l[0], rps_per_v_ccw_l[1]);

        if (checkMotorRpsError(rps_per_v_ccw_l[0], rps_per_v_ccw_l[1])) {
          return;
        } else if (checkMotorRpsHighLowError(rps_per_v_cw_l[0], rps_per_v_cw_l[1], rps_per_v_ccw_l[0], rps_per_v_ccw_l[1])) {
          return;
        }

        calib_process.motor_calib_cnt = MOTOR_CALIB_INIT_CNT;
        calib_process.motor_calib_mode++;
        if (rps_per_v_cw_l[0] < 4.0) {
          calib_process.motor_calib_voltage = MOTOR_CALIB_VOLTAGE_HIGH;
        } else {
          calib_process.motor_calib_voltage = MOTOR_CALIB_VOLTAGE_HIGH / 2;
        }
        p("set output V = %f\n", calib_process.motor_calib_voltage);
        break;

      case 3:
        rps_per_v_cw_h[0] = calib[0].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        rps_per_v_cw_h[1] = calib[1].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        calib[0].rps_integral = 0;
        calib[1].rps_integral = 0;
        p("\n\nMotor Calib rps/v \n M0 %6.2f\n M1 %6.2f\n\n", rps_per_v_cw_h[0], rps_per_v_cw_h[1]);

        if (checkMotorRpsError(rps_per_v_cw_h[0], rps_per_v_cw_h[1])) {
          return;
        }

        calib_process.motor_calib_cnt = MOTOR_CALIB_INIT_CNT;
        calib_process.motor_calib_mode++;

        calib_process.motor_calib_mode++;
        if (rps_per_v_cw_l[0] < 4.0) {
          calib_process.motor_calib_voltage = -MOTOR_CALIB_VOLTAGE_HIGH;
        } else {
          calib_process.motor_calib_voltage = -MOTOR_CALIB_VOLTAGE_HIGH / 2;
        }
        p("set output V = %f\n", calib_process.motor_calib_voltage);
        break;

      default:

        rps_per_v_ccw_h[0] = calib[0].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        rps_per_v_ccw_h[1] = calib[1].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        calib[0].rps_integral = 0;
        calib[1].rps_integral = 0;
        p("\n\nMotor Calib rps/v \n M0 %6.2f\n M1 %6.2f\n\n", rps_per_v_ccw_h[0], rps_per_v_ccw_h[1]);
        p("\n\n!!!!!!FINISH!!!!!!!!\n\n");

        if (checkMotorRpsError(rps_per_v_ccw_h[0], rps_per_v_ccw_h[1])) {
          return;
        } else if (checkMotorRpsHighLowError(rps_per_v_cw_h[0], rps_per_v_cw_h[1], rps_per_v_ccw_h[0], rps_per_v_ccw_h[1])) {
          return;
        }

        p("save calib result...\n");
        writeMotorCalibrationValue(rps_per_v_cw_l[0], rps_per_v_cw_l[1]);

        HAL_Delay(10);
        p("enc data : %4.1f %4.1f\n", flash.calib[0], flash.calib[1]);
        p("motor data : %4.1f %4.1f\n", flash.rps_per_v_cw[0], flash.rps_per_v_cw[1]);

        HAL_Delay(1000);

        NVIC_SystemReset();

        break;
    }
  }
}

void startCalibrationMode(void)
{
  p("calibration mode!\n");

  calib_process.enc_calib_cnt = MOTOR_CALIB_INIT_CNT;
  calib_process.motor_calib_cnt = 0;
  manual_offset_radian = 0;

  cmd[0].speed = 0;
  cmd[1].speed = 0;

  cmd[0].out_v_final = 2.0;
  cmd[1].out_v_final = 2.0;

  // キャリブレーションモード中の動作

  // エンコーダーキャリブレーション
  // エンコーダー : 通常動作と同じ
  // 出力角度 : 低速で強制転流
  // CAN受信 : 早期returnで全部無視
  // 出力電圧 : 固定
  // main : encoderCalibrationMode()

  // モーターキャリブレーション
  // エンコーダー : 通常動作と同じ
  // 出力角度 : 通常動作と同じ
  // CAN受信 : 早期returnで全部無視
  // 出力電圧 : CW,CCWで各2数段階,2段階めは1段階めの回転数に応じて変更
  // main : runMode()
}

void receiveUserSerialCommand(void)
{
  if (uart_rx_flag) {
    uart_rx_flag = false;
    HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);
    switch (uart_rx_buf[0]) {
      case 'c':
        p("\n\nstart calib mode!\n\n");
        startCalibrationMode();
        break;
      case 'n':
        p("run mode!\n");

        calib_process.enc_calib_cnt = 0;
        calib_process.motor_calib_cnt = 0;
        manual_offset_radian = 0;

        cmd[0].out_v = 0;
        cmd[1].out_v = 0;
        break;
      case 'q':
        manual_offset_radian += 0.01;
        p("offset %+4.2f\n", manual_offset_radian);
        break;
      case 'a':
        manual_offset_radian -= 0.01;
        p("offset %+4.2f\n", manual_offset_radian);
        break;
      case 'w':
        cmd[0].speed += 0.5;
        cmd[0].timeout_cnt = -1;
        cmd[1].speed += 0.5;
        cmd[1].timeout_cnt = -1;
        break;
      case 's':
        cmd[0].speed -= 0.5;
        cmd[0].timeout_cnt = -1;
        cmd[1].speed -= 0.5;
        cmd[1].timeout_cnt = -1;
        break;
      case 'e':
        pid[0].pid_kp += 0.1;
        pid[1].pid_kp += 0.1;
        p("\nKP %+5.2f\n", pid[0].pid_kp);
        break;
      case 'd':
        pid[0].pid_kp -= 0.1;
        pid[1].pid_kp -= 0.1;
        p("\nKP %+5.2f\n", pid[0].pid_kp);
        break;
      case 'r':
        pid[0].pid_ki += 0.1;
        pid[1].pid_ki += 0.1;
        p("\nKI %+5.2f\n", pid[0].pid_ki);
        break;
      case 'f':
        pid[0].pid_ki -= 0.1;
        pid[1].pid_ki -= 0.1;
        p("\nKI %+5.2f\n", pid[0].pid_ki);
        break;
      case 't':
        pid[0].pid_kd += 0.1;
        pid[1].pid_kd += 0.1;
        p("\nKD %+5.2f\n", pid[0].pid_kd);
        break;
      case 'g':
        pid[0].pid_kd -= 0.1;
        pid[1].pid_kd -= 0.1;
        p("\nKD %+5.2f\n", pid[0].pid_kd);
        break;
      case 'y':
        motor_real[0].k += 0.1;
        motor_real[1].k += 0.1;
        break;
      case 'h':
        motor_real[0].k -= 0.1;
        motor_real[1].k -= 0.1;
        break;
      case '0':
        p("enter sleep!\n");
        forceStopAllPwmOutputAndTimer();
        while (1)
          ;
        break;
    }
  }
}

// 2ms cycle
void sendCanData(void)
{
  static int transfer_cnt;

  sendSpeed(flash.board_id, 0, motor_real[0].rps, (float)ma702[0].enc_raw * 2 * M_PI / 65535);
  sendSpeed(flash.board_id, 1, motor_real[1].rps, (float)ma702[1].enc_raw * 2 * M_PI / 65535);

  switch (transfer_cnt) {
    case 0:
      sendVoltage(flash.board_id, 0, getBatteryVoltage());
      break;
    case 2:
      sendVoltage(flash.board_id, 1, getBatteryVoltage());
      break;
    case 4:
      sendCurrent(flash.board_id, 0, getCurrentM0());
      break;
    case 6:
      sendCurrent(flash.board_id, 1, getCurrentM1());
      break;
    case 8:
      // 本当はFETとモーター温度をまとめてint x4で送ったほうがいいかもしれないが、実用上のメリットが少ないので後回し
      sendTemperature(flash.board_id, 0, getTempM0());
      break;
    case 10:
      sendTemperature(flash.board_id, 1, getTempM1());
      break;
    case 12:
      // 拡張
      sendFloat(0x500 + flash.board_id * 2, flash.rps_per_v_cw[0]);
      break;
    case 14:
      // 拡張
      sendFloat(0x501 + flash.board_id * 2, flash.rps_per_v_cw[1]);
      break;
    case 50:
      transfer_cnt = -1;
      break;
    default:
      break;
  }
  transfer_cnt++;
}

void protect(void)
{
  if (getCurrentM0() > 6.0 || getCurrentM1() > 6.0) {
    forceStopAllPwmOutputAndTimer();
    p("over current!! : %+6.2f %+6.2f\n", getCurrentM0(), getCurrentM1());
    setLedBlue(false);
    setLedGreen(true);
    setLedRed(true);
    error_id = OVER_CURRENT;
    if (getCurrentM0() > getCurrentM1()) {
      error_info = 0;
      error_value = getCurrentM0();
    } else {
      error_info = 1;
      error_value = getCurrentM1();
    }
    waitPowerOnTimeout();
  }  //*/

  if (enc_over_speed_cnt_error_flag) {
    forceStopAllPwmOutputAndTimer();
    p("encoder error!!! ENC M%d diff %5d", enc_over_speed_cnt_error_enc_idx, enc_over_speed_cnt_error_enc_cnt);
    setLedBlue(true);
    setLedGreen(false);
    setLedRed(true);
    error_id = ENC_ERROR;
    error_info = enc_over_speed_cnt_error_enc_idx;
    error_value = motor_real[enc_over_speed_cnt_error_enc_idx].diff_cnt_max;
    waitPowerOnTimeout();
  }

  if (getBatteryVoltage() < THR_BATTERY_UNVER_VOLTAGE) {
    forceStopAllPwmOutputAndTimer();
    p("UNDER voltage!! %6.3f", getBatteryVoltage());
    setLedBlue(true);
    setLedGreen(false);
    setLedRed(true);
    error_id = UNDER_VOLTAGE;
    error_info = 0;
    error_value = getBatteryVoltage();
    waitPowerOnTimeout();
  }

  if (getBatteryVoltage() > THR_BATTERY_OVER_VOLTAGE) {
    setPwmAll(0);
    //stopTimerInterrupt();
    p("OVER voltage!! %6.3f", getBatteryVoltage());
    setLedBlue(true);
    setLedGreen(false);
    setLedRed(true);
    error_id = OVER_VOLTAGE;
    error_info = 0;
    error_value = getBatteryVoltage();
    waitPowerOnTimeout();
  }

  if (getTempM0() > THR_MOTOR_OVER_TEMPERATURE || getTempM1() > THR_MOTOR_OVER_TEMPERATURE) {
    forceStopAllPwmOutputAndTimer();
    p("OVER Motor temperature!! M0 : %3d M1 : %3d", getTempM0(), getTempM1());
    setLedBlue(true);
    setLedGreen(true);
    setLedRed(true);

    error_id = MOTOR_OVER_HEAT;
    if (getTempM0() > getTempM1()) {
      error_info = 0;
      error_value = (float)getTempM0();
    } else {
      error_info = 1;
      error_value = (float)getTempM1();
    }
    waitPowerOnTimeout();
  }
  /*if (getTempFET0() > THR_MOTOR_OVER_TEMPERATURE || getTempFET1() > THR_MOTOR_OVER_TEMPERATURE) {
    forceStopAllPwmOutputAndTimer();
    p("OVER FET temperature!! M0 : %3df M1 : %3d", getTempFET0(), getTempFET1());
    setLedBlue(true);
    setLedGreen(true);
    setLedRed(true);

    error_id = MOTOR_OVER_HEAT;
    if (getTempFET0() > getTempFET1()) {
      error_info = 0;
      error_value = (float)getTempFET0();
    } else {
      error_info = 1;
      error_value = (float)getTempFET1();
    }
    waitPowerOnTimeout();
  }*/
  if (pid[0].load_limit_cnt > MOTOR_OVER_LOAD_CNT_LIMIT || pid[1].load_limit_cnt > MOTOR_OVER_LOAD_CNT_LIMIT) {
    forceStopAllPwmOutputAndTimer();
    p("over load!! %d %d", pid[0].load_limit_cnt, pid[1].load_limit_cnt);
    setLedBlue(false);
    setLedGreen(false);
    setLedRed(true);

    error_id = OVER_LOAD;
    if (pid[0].load_limit_cnt > pid[1].load_limit_cnt) {
      error_info = 0;
      error_value = pid[0].load_limit_cnt;
    } else {
      error_info = 1;
      error_value = pid[1].load_limit_cnt;
    }

    waitPowerOnTimeout();
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
  p("\n\n** Orion VV driver V4 start! **\n");

  for (int i = 0; i < 2; i++) {
    pid[i].pid_kp = 0.2;
    pid[i].pid_ki = 0.3;
    pid[i].pid_kd = 0.0;
    pid[i].error_integral_limit = 4.0;
    pid[i].diff_voltage_limit = 5.0;  // 2.0 -> 4.0 -> 6.0

    cmd[i].speed = 0;
    cmd[i].timeout_cnt = -1;

    cmd[i].out_v = 0;
    cmd[i].out_v_final = 0;

    // set calibration params
    enc_offset[i].zero_calib = flash.calib[i];

    // rps/v
    // 200kV : 3.33 rps/v
    // 100kV : 1.67 rps/v
    // M0 5.7, M1 2.88 rps/v

    // v/rps
    // 200kV : 0.3 rps/v
    // 100kV : 0.6 rps/v
    // M0 0.175, M1 0.347 v/rps

    if (1 < flash.rps_per_v_cw[i] && flash.rps_per_v_cw[i] < 10) {
      motor_param[i].voltage_per_rps = 1 / flash.rps_per_v_cw[i];
    } else {
      motor_param[i].voltage_per_rps = V_PER_RPS_DEFAULT;
    }
  }
  p("CAN ADDR 0x%03x\nenc offset M0 %6.3f M1 %6.3f\nRPS/V M0 %6.3f M1 %6.3f\n", flash.board_id, flash.calib[0], flash.calib[1], flash.rps_per_v_cw[0], flash.rps_per_v_cw[1]);
  HAL_Delay(1);
  p("Kv M0 %6.3f M1 %6.3f rpm/V\n", flash.rps_per_v_cw[0] * 60, flash.rps_per_v_cw[1] * 60);

  __HAL_SPI_ENABLE(&hspi1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  // 50 / 5 : 10ぐらい
  //
  HAL_Delay(1);

  motor_param[0].output_voltage_limit = SPEED_CMD_LIMIT_RPS / flash.rps_per_v_cw[0] * SPEED_REAL_LIMIT_GAIN;
  motor_param[1].output_voltage_limit = SPEED_CMD_LIMIT_RPS / flash.rps_per_v_cw[1] * SPEED_REAL_LIMIT_GAIN;
  p("output voltage limit : %5.2f %5.2f\n", motor_param[0].output_voltage_limit, motor_param[1].output_voltage_limit);
  HAL_Delay(1);
  p("0x00 0x01 0x02 0x03 0x04 0x05 0x06 0x09 0x0E 0x10 0x1B\n");
  for (int i = 0; i < 2; i++) {
    HAL_Delay(10);
    writeRegisterMA702(i, 5, 0xFF);
    HAL_Delay(10);
    writeRegisterMA702(i, 6, 0x1C);
    HAL_Delay(10);
    writeRegisterMA702(i, 0x10, 0x9C);
    HAL_Delay(10);
    writeRegisterMA702(i, 0x1B, 0x43);
    HAL_Delay(10);

    // id = 0xE, filter window
    // default : 119, f-cut = 370Hz

    // 187(0xBB) = 23Hz (MA730,NG)
    // 119(0x77) = 370Hz (default)
    // 390Hz = MA702
    // 136(0x88) = 185Hz
    writeRegisterMA702(i, 0x0E, 0x77);
    HAL_Delay(10);
    p("reg = 0x%02x", readRegisterMA702(i, 0));  // Z offset-L
    HAL_Delay(1);
    p("0x%02x ", readRegisterMA702(i, 1));  // Z offset-H
    HAL_Delay(1);
    p("0x%02x ", readRegisterMA702(i, 2));  // BCT (off-axis param)
    HAL_Delay(1);
    p("0x%02x ", readRegisterMA702(i, 3));  // ETY,ETX
    HAL_Delay(1);
    p("0x%02x ", readRegisterMA702(i, 4));  // PPT-L/ILIP
    HAL_Delay(1);
    p("0x%02x ", readRegisterMA702(i, 5));  // PPT-H
    HAL_Delay(1);
    p("0x%02x ", readRegisterMA702(i, 6));  // MGLT/MGHT
    HAL_Delay(1);
    p("0x%02x ", readRegisterMA702(i, 9));  // RD
    HAL_Delay(1);
    p("0x%02x ", readRegisterMA702(i, 0xE));  // FW
    HAL_Delay(1);
    p("0x%02x ", readRegisterMA702(i, 0x10));  // HYS
    HAL_Delay(1);
    p("0x%02x ", readRegisterMA702(i, 0x1B));  // MGH&L
    HAL_Delay(1);
    p("\n");
  }

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);
  HAL_ADC_Start(&hadc3);

  htim1.Instance->CNT = 0;
  htim8.Instance->CNT = 10;

  HAL_TIM_Base_Start_IT(&htim1);
  HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);

  uint32_t over_startup_voltage = 0;
  p("waiting startup voltage.... : %3.1fV\n", THR_BATTERY_UNVER_VOLTAGE + 2);
  while (over_startup_voltage < 500) {
    HAL_Delay(1);
    if (getBatteryVoltage() > THR_BATTERY_UNVER_VOLTAGE + 2.0) {
      over_startup_voltage++;
    } else {
      over_startup_voltage = 0;
    }
  }

  p("ADC : %5d %5d GD %4.2f Batt %4.2f\n", adc_raw.cs_m0, adc_raw.cs_m1, getGateDriverDCDCVoltage(), getBatteryVoltage());
  if (adc_raw.cs_m0 < 100 && adc_raw.cs_m1 < 100) {
    // 正方向電流のみモデル
    // ZXCT1084
    adc_raw.cs_adc_offset = 0;
  } else {
    // 正負電流
    // INA199
    adc_raw.cs_adc_offset = 2048;
  }

  // M0 : tim1
  // M1 : tim8

  HAL_TIM_PWM_Init(&htim8);
  HAL_TIM_PWM_Init(&htim1);
  setPwmAll(TIM_PWM_CENTER);

  // PWM出力をHIGH/LOW順番に1chずつONにして短絡チェック
  int turn_on_channel = 0;
  while (turn_on_channel < 6) {
    switch (turn_on_channel) {
      case 0:
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);  // M0 low
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);  // M1 low
        break;
      case 1:
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);  // M0 low
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);  // M1 low
        break;
      case 2:
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);  // M0 low
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);  // M1 low
        break;
      case 3:
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);  // M1 high & low
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // M0 high & low
        break;
      case 4:
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);  // M1 high & low
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);  // M0 high & low
        break;
      case 5:
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);  // M1 high & low
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);  // M0 high & low
        break;
      default:
        break;
    }

    interrupt_timer_cnt = 0;
    while (interrupt_timer_cnt < INTERRUPT_KHZ_1MS * 50) {
      if (isNotZeroCurrent() || getBatteryVoltage() < THR_BATTERY_UNVER_VOLTAGE) {
        power_enable_cnt = 500;
        forceStopAllPwmOutputAndTimer();
        p("Current MM0 %+6.3f M1 %+6.3f ch:%d\n", getCurrentM0(), getCurrentM1(), turn_on_channel);
        waitPowerOnTimeout();
      }
    }
    p("ch:%2d CurrentCheck OK!! M0 %+6.3f M1 %+6.3f Battery %5.2f GD %5.2f\n", turn_on_channel, getCurrentM0(), getCurrentM1(), getBatteryVoltage(), getGateDriverDCDCVoltage());
    turn_on_channel++;
  }
  // 全部ONにする
  resumePwmOutput();

  //*/

  if (isPushedSW1()) {
    flash.board_id = 0;
    writeCanBoardID(flash.board_id);
    p("sed board id %d\n", flash.board_id);
    HAL_Delay(1000);
  } else if (isPushedSW2()) {
    flash.board_id = 1;
    writeCanBoardID(flash.board_id);
    p("sed board id %d\n", flash.board_id);
    HAL_Delay(1000);
  }
  if (isPushedSW4()) {
    startCalibrationMode();
    p("enc calibration mode!!\n");
    while (isPushedSW4())
      ;
  }

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
    receiveUserSerialCommand();
    calcMotorSpeed(0);
    calcMotorSpeed(1);
    sendCanData();

    if (calib_process.enc_calib_cnt != 0) {
      encoderCalibrationMode();
    } else if (calib_process.motor_calib_cnt != 0) {
      motorCalibrationMode();
    } else {
      runMode();
    }
    protect();

    setLedRed(true);

    // 周期固定するために待つ
    main_loop_remain_counter = INTERRUPT_KHZ_1MS - interrupt_timer_cnt;
    while (interrupt_timer_cnt <= INTERRUPT_KHZ_1MS)
      ;
    interrupt_timer_cnt = 0;

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
