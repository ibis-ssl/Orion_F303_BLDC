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

// 200kV -> 3.33rps/V -> 0.3 V/rps
//#define RPS_TO_MOTOR_EFF_VOLTAGE (0.15)
// 13rps : 2.4V

#define V_PER_RPS_DEFAULT (0.15)

#define MOTOR_OVER_LOAD_CNT_LIMIT (3000)

#define THR_MOTOR_OVER_CURRENT (10)
#define THR_BATTERY_UNVER_VOLTAGE (18.0)
#define THR_BATTERY_OVER_VOLTAGE (35.0)
#define THR_MOTOR_OVER_TEMPERATURE (70)    // 80 -> 70 deg (実機テストによる)
#define THR_FET_OVER_TEMPERATURE (80)      // 70 -> 80 deg (実機テストによる)
#define THR_NO_CONNECTED_TEPERATURE (120)  // V4.1用、無接続時130度程度になるので

#define MOTOR_CALIB_INIT_CNT (2500)
#define MOTOR_CALIB_READY_CNT (2000)
#define MOTOR_CALIB_START_CNT (1500)
#define MOTOR_CALIB_VOLTAGE_LOW (3.0)
#define MOTOR_CALIB_VOLTAGE_HIGH (5.0)
#define MOTOR_CALIB_CYCLE (12)

#define MOTOR_CALIB_M0_M1_ERROR_TRERANCE (1.0)
#define MOTOR_CALIB_CW_CCW_ERROR_TRERANCE (1.0)
#define MOTOR_CALIB_UNDER_LIMIT (1.0)

// ヤバい速度指令を無視するしきい値
#define SPEED_REAL_LIMIT_GAIN (float)(1.5)
// エンコーダー飛んだ時に異常な速度を検出するしきい値

void checkAngleCalibMode(bool motor)
{
  calib[motor].xy_field.radian_ave_x += cos(as5047p[motor].output_radian);
  calib[motor].xy_field.radian_ave_y += sin(as5047p[motor].output_radian);
  calib[motor].ave_cnt++;
  if (calib[motor].pre_raw > HARF_OF_ENC_CNT_MAX && as5047p[motor].enc_raw < HARF_OF_ENC_CNT_MAX && calib_process.force_rotation_speed > 0) {
    // ccw
    calib_process.print_flag = true;
    calib[motor].result_ccw_cnt++;
    calib[motor].xy_field.result_cw_x = calib[motor].xy_field.radian_ave_x / calib[motor].ave_cnt;
    calib[motor].xy_field.result_cw_y = calib[motor].xy_field.radian_ave_y / calib[motor].ave_cnt;
    calib[motor].xy_field.radian_ave_x = 0;
    calib[motor].xy_field.radian_ave_y = 0;
    calib[motor].ave_cnt = 0;
  }
  if (calib[motor].pre_raw < HARF_OF_ENC_CNT_MAX && as5047p[motor].enc_raw > HARF_OF_ENC_CNT_MAX && calib_process.force_rotation_speed < 0) {
    // cw
    calib_process.print_flag = true;
    calib[motor].result_cw_cnt++;
    calib[motor].xy_field.result_ccw_x = calib[motor].xy_field.radian_ave_x / calib[motor].ave_cnt;
    calib[motor].xy_field.result_ccw_y = calib[motor].xy_field.radian_ave_y / calib[motor].ave_cnt;
    calib[motor].xy_field.radian_ave_x = 0;
    calib[motor].xy_field.radian_ave_y = 0;
    calib[motor].ave_cnt = 0;
  }
  calib[motor].pre_raw = as5047p[motor].enc_raw;
}

inline void calibrationProcess_itr(bool motor)
{
  sys.manual_offset_radian += calib_process.force_rotation_speed;

  // 出力電気角度 = 0 のときに、エンコーダー角度を計測
  if (sys.manual_offset_radian > M_PI * 2) {
    sys.manual_offset_radian -= M_PI * 2;
    checkAngleCalibMode(!motor);
  }
  if (sys.manual_offset_radian < 0) {
    sys.manual_offset_radian += M_PI * 2;
    checkAngleCalibMode(!motor);
  }

  updateADC(motor);
  updateAS5047P(motor);
  setOutputRadianMotor(motor, sys.manual_offset_radian, cmd[motor].out_v_final, getBatteryVoltage(), MOTOR_CALIB_VOLTAGE_HIGH);
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
volatile static float main_loop_remain_counter_ave = 0, task_complete_timer_cnt_ave = 0;
volatile static uint32_t system_exec_time_stamp[10] = {0};
volatile static float system_exec_time_stamp_ave[10] = {0};
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
  if (calib_process.enc_calib_cnt != 0) {
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

void can_rx_callback(void)
{
  // モーターキャリブレーション中は無視
  if (calib_process.enc_calib_cnt != 0 || calib_process.motor_calib_cnt != 0) {
    return;
  }

  can_rx_cnt++;
  switch (can_rx_header.StdId) {
    case 0x010:
      if (can_rx_buf.data[0] == 0) {
        if (can_rx_buf.data[1] == 0) {
          HAL_NVIC_SystemReset();
        } else if (can_rx_buf.data[1] == 1) {
          sys.power_enable_cnt = 100;
        }
      }
      break;

      // IDによる振り分け
    case 0x100:
    case 0x102:
      cmd[0].speed = clampSize(can_rx_buf.value[0], SPEED_CMD_LIMIT_RPS);
      cmd[0].timeout_cnt = 100;
      break;

    case 0x101:
    case 0x103:
      cmd[1].speed = clampSize(can_rx_buf.value[0], SPEED_CMD_LIMIT_RPS);
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
        p("stop!!! CAN cmd");
        sys.free_wheel_cnt = can_rx_buf.data[2];
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
  // ここが速度によって処理負荷変わってる？
  // 負のほうが処理時間かかってる
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
  // ここは1KHzでまわっている

  // 1サイクルごとの負荷を減らすために分割して送信
  // 1ms程度かかっている
  sys.print_cnt++;

  switch (sys.print_cnt) {
    case 1:
      // p("M0raw %6d M1raw %6d ", as5047p[0].enc_raw, as5047p[1].enc_raw);
      p("\e[0mCS %+5.2f %+5.2f / BV %4.1f ", getCurrentMotor(0), getCurrentMotor(1), getBatteryVoltage());
      // p("P %+3.1f I %+3.1f D %+3.1f ", pid[0].pid_kp, pid[0].pid_ki, pid[0].pid_kd);
      break;
    case 2:
      p("RPS %+6.1f %+6.1f Free %4d ", motor_real[0].rps, motor_real[1].rps, sys.free_wheel_cnt);
      break;
    case 3:
      p("RAW %5d %5d Out_v %+5.1f %+5.1f ", as5047p[0].enc_raw, as5047p[1].enc_raw, cmd[0].out_v, cmd[1].out_v);
      break;
    case 4:
      //p("p%+3.1f i%+3.1f d%+3.1f k%+3.1f ", pid[0].pid_kp, pid[0].pid_ki, pid[0].pid_kd, motor_real[0].k);
      p("Rx %4ld CPU %3d GD %4.1f ", can_rx_cnt, main_loop_remain_counter, getGateDriverDCDCVoltage());
      can_rx_cnt = 0;
      break;
    case 5:
      if (sys.print_idx == 0) {
        p("Eff %+6.2f %+6.2f %d %d ", pid[0].eff_voltage, pid[1].eff_voltage, pid[0].output_voltage_limitting, pid[1].output_voltage_limitting);
      } else if (sys.print_idx == 1) {
        // FET温度はv4.2で取得できない
        p("FET-T %+4d %+4d Motor-T %+4d %+4d", getTempFET(0), getTempFET(1), getTempMotor(0), getTempMotor(1));
      } else {
        p("SPD %+6.1f %+6.1f canErr 0x%04x ", cmd[0].speed, cmd[1].speed, getCanError());
      }
      break;
    case 6:
      task_complete_timer_cnt_ave = task_complete_timer_cnt_ave * 0.99 + (float)sys.task_complete_timer_cnt * 0.01;
      main_loop_remain_counter_ave = main_loop_remain_counter_ave * 0.99 + (float)main_loop_remain_counter * 0.01;
      p("CPUA %6.4f CNT %6.2f ", main_loop_remain_counter_ave, task_complete_timer_cnt_ave);
      //p("LoadV %+5.2f %+5.2f CanFail %4d ", cmd[0].out_v - pid[0].eff_voltage, cmd[1].out_v - pid[1].eff_voltage, ex_can_send_fail_cnt);
      ex_can_send_fail_cnt = 0;
      break;
    case 7:
      p("Offset %+4.2f RPS %+6.1f %+6.1f ", sys.manual_offset_radian, motor_real[0].rps_ave, motor_real[1].rps_ave);
      //p("LoadCnt %4.3f %4.3f ", (float)pid[0].load_limit_cnt / MOTOR_OVER_LOAD_CNT_LIMIT, (float)pid[1].load_limit_cnt / MOTOR_OVER_LOAD_CNT_LIMIT);
      break;
    case 8:
      for (int i = 0; i < 4; i++) {
        system_exec_time_stamp_ave[i] = system_exec_time_stamp_ave[i] * 0.99 + (float)system_exec_time_stamp[i] * 0.01;
      }
      p("Ave %6.4f %6.4f %6.4f %6.4f ", system_exec_time_stamp_ave[0], system_exec_time_stamp_ave[1], system_exec_time_stamp_ave[2], system_exec_time_stamp_ave[3]);
      //p("TO %4d %4d diff max M0 %+6d, M1 %+6d %d", cmd[0].timeout_cnt, cmd[1].timeout_cnt, motor_real[0].diff_cnt_max, motor_real[1].diff_cnt_max, enc_error_watcher.detect_flag);
      // p("min %+6d cnt %6d / max %+6d cnt %6d ", as5047p[0].diff_min, as5047p[0].diff_min_cnt, as5047p[0].diff_max, as5047p[0].diff_max_cnt);
      motor_real[0].diff_cnt_max = 0;
      motor_real[1].diff_cnt_max = 0;
      as5047p[0].diff_max = 0;
      as5047p[0].diff_min = 65535;
      as5047p[1].diff_max = 0;
      as5047p[1].diff_min = 65535;
      break;
    default:
      p("\n");
      sys.print_cnt = 0;
      break;
  }

  // ADC raw ALL
  //	p("M0raw %8d M1raw %8d offset %4.3f, voltageM0 %+6.3f M1 %6.3f rx %6ld speedM0 %+6.3f\n", as5047p[0].enc_raw, as5047p[1].enc_raw, sys.manual_offset_radian, cmd[0].out_v, cmd[1].out_v, can_rx_cnt, cmd[0].speed);
}

/* Can't running 1kHz */
void encoderCalibrationMode(void)
{
  // 角度0のときにprint
  if (calib_process.print_flag) {
    calib_process.print_flag = false;
    p("enc = %+5.2f %+5.2f  / M0 X %+5.2f Y %+5.2f / M1 X %+5.2f Y %+5.2f / Rad %+5.2f %+5.2f\n", as5047p[0].output_radian, as5047p[1].output_radian, cos(as5047p[0].output_radian),
      sin(as5047p[0].output_radian), cos(as5047p[1].output_radian), sin(as5047p[1].output_radian), atan2(sin(as5047p[0].output_radian), cos(as5047p[0].output_radian)),
      atan2(sin(as5047p[1].output_radian), cos(as5047p[1].output_radian)));
  }

  // calib_process.force_rotation_speedが+でCCW
  // calib_process.force_rotation_speedが-でCW回転する。
  // 初期値はcalib_process.force_rotation_speedが+でCCWからキャリブレーション。

  // END of 1st-calibration cycle (CCW)
  if (calib[0].result_ccw_cnt > MOTOR_CALIB_CYCLE && calib[1].result_ccw_cnt > MOTOR_CALIB_CYCLE && calib_process.force_rotation_speed > 0) {
    calib_process.force_rotation_speed = -calib_process.force_rotation_speed;  //CCW方向終わったので、回転方向反転
    p("END of 1st-calibration cycle (CCW)\n");
    HAL_Delay(1);  // write out uart buffer
  }

  // END of 2nd-calibration cycle (CW)
  if (calib[0].result_cw_cnt > MOTOR_CALIB_CYCLE && calib[1].result_cw_cnt > MOTOR_CALIB_CYCLE) {
    // 強制転流モード完了
    cmd[0].out_v_final = 0;
    cmd[1].out_v_final = 0;
    p("END of 2nd-calibration cycle (CW)\n");
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

      /*       xy_field_offset_radian[i] += ROTATION_OFFSET_RADIAN;
      if (xy_field_offset_radian[i] > M_PI * 2) {
        xy_field_offset_radian[i] -= M_PI * 2;
      }
      if (xy_field_offset_radian[i] < 0) {
        xy_field_offset_radian[i] += M_PI * 2;
      } */

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
    sys.manual_offset_radian = 0;  // 割り込みの中で加算してしまうので,enc_calib_cnt = 0にしてからでないといけない

    // モーターキャリブレーションに切り替え
    calib_process.motor_calib_mode = 0;
    calib_process.motor_calib_cnt = 1;

    p("calib %+5.2f %+5.2f", enc_offset[0].zero_calib, enc_offset[1].zero_calib);
    HAL_Delay(900);
  }

  static int print_cnt = 0;
  print_cnt++;
  if (print_cnt > 1000) {
    print_cnt = 0;
    p("%4.1f CNT %4d %4d %4d / %4d %4.1f\n", cmd[0].out_v_final, htim1.Instance->CCR1, htim1.Instance->CCR2, htim1.Instance->CCR3, calib_process.enc_calib_cnt, getBatteryVoltage());
  }
}

bool checkMotorRpsError(float m0, float m1)
{
  if (fabsf(m0 - m1) > MOTOR_CALIB_M0_M1_ERROR_TRERANCE) {
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
  if (fabsf(m0_cw - m0_ccw) > MOTOR_CALIB_CW_CCW_ERROR_TRERANCE || fabsf(m1_cw - m1_ccw) > MOTOR_CALIB_CW_CCW_ERROR_TRERANCE) {
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

    setFinalOutputVoltage(&cmd[i], &enc_offset[i], sys.manual_offset_radian);  // select Vq-offset angle
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
        calib_process.motor_calib_voltage = MOTOR_CALIB_VOLTAGE_HIGH;
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
        calib_process.motor_calib_voltage = -MOTOR_CALIB_VOLTAGE_HIGH;
        p("set output V = %f\n", calib_process.motor_calib_voltage);
        break;

      default:

        rps_per_v_ccw_h[0] = calib[0].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        rps_per_v_ccw_h[1] = calib[1].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        calib[0].rps_integral = 0;
        calib[1].rps_integral = 0;
        p("\n\nMotor Calib rps/v \n ");
        p("M0 -%6.2f +%6.2f Diff %+6.2f\n ", rps_per_v_ccw_h[0], rps_per_v_cw_h[0], rps_per_v_ccw_h[0] - rps_per_v_cw_h[0]);
        p("M0 -%6.2f +%6.2f Diff %+6.2f \n", rps_per_v_ccw_h[1], rps_per_v_cw_h[1], rps_per_v_ccw_h[1] - rps_per_v_cw_h[1]);
        p("\n\n!!!!!!FINISH!!!!!!!!\n\n");

        if (checkMotorRpsError(rps_per_v_ccw_h[0], rps_per_v_ccw_h[1])) {
          return;
        } else if (checkMotorRpsHighLowError(rps_per_v_cw_h[0], rps_per_v_cw_h[1], rps_per_v_ccw_h[0], rps_per_v_ccw_h[1])) {
          return;
        }

        p("save calib result...\n");
        // 高回転時のパラメーターのみ使用(低回転から切り替え)
        writeMotorCalibrationValue(rps_per_v_cw_h[0], rps_per_v_cw_h[1]);

        HAL_Delay(10);
        p("enc data : %4.2f %4.2f\n", flash.calib[0], flash.calib[1]);
        p("motor data : %4.2f %4.2f\n", flash.rps_per_v_cw[0], flash.rps_per_v_cw[1]);

        HAL_Delay(1000);

        NVIC_SystemReset();

        break;
    }
  }

  static int print_cnt = 0;
  print_cnt++;
  if (print_cnt > 100) {
    print_cnt = 0;
    p("%4.1f %4.1f\n", cmd[0].out_v, cmd[1].out_v);
  }
}

void startCalibrationMode(void)
{
  p("calibration mode!\n");

  calib_process.enc_calib_cnt = MOTOR_CALIB_INIT_CNT;
  calib_process.motor_calib_cnt = 0;
  sys.manual_offset_radian = 0;

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
        sys.manual_offset_radian = 0;

        cmd[0].out_v = 0;
        cmd[1].out_v = 0;
        break;
      case 'q':
        sys.manual_offset_radian += 0.01;
        p("offset %+4.2f\n", sys.manual_offset_radian);
        break;
      case 'a':
        sys.manual_offset_radian -= 0.01;
        p("offset %+4.2f\n", sys.manual_offset_radian);
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
        while (1);
        break;
      case '\n':
        sys.print_idx++;
        if (sys.print_idx > 2) {
          sys.print_idx = 0;
        }
        p("\nprint idx : %d\n");
        break;
    }
  }
}

// 2ms cycle
void sendCanData(void)
{
  static int transfer_cnt;

  sendSpeed(flash.board_id, 0, motor_real[0].rps, (float)as5047p[0].enc_raw * 2 * M_PI / 65535);
  sendSpeed(flash.board_id, 1, motor_real[1].rps, (float)as5047p[1].enc_raw * 2 * M_PI / 65535);

  switch (transfer_cnt) {
    case 0:
      sendVoltage(flash.board_id, 0, getBatteryVoltage());
      break;
    case 2:
      sendVoltage(flash.board_id, 1, getBatteryVoltage());
      break;
    case 4:
      sendCurrent(flash.board_id, 0, getCurrentMotor(0));
      break;
    case 6:
      sendCurrent(flash.board_id, 1, getCurrentMotor(1));
      break;
    case 8:
      // 本当はFETとモーター温度をまとめてint x4で送ったほうがいいかもしれないが、実用上のメリットが少ないので後回し
      sendTemperature(flash.board_id, 0, getTempMotor(0), getTempFET(0));
      break;
    case 10:
      sendTemperature(flash.board_id, 1, getTempMotor(1), getTempFET(1));
      break;
    case 12:
      // 拡張
      sendFloatDual(0x500 + flash.board_id * 2, flash.rps_per_v_cw[0], 0);
      break;
    case 14:
      // 拡張
      sendFloatDual(0x501 + flash.board_id * 2, flash.rps_per_v_cw[1], 0);
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
  for (int i = 0; i < 2; i++) {
    if (getCurrentMotor(i) > THR_MOTOR_OVER_CURRENT) {
      forceStopAllPwmOutputAndTimer();

      p("M%d over current!! : %+6.2f / out_v %+6.2f\n", i, getCurrentMotor(i), cmd[i].out_v_final, pid[i].output_voltage_limitting);
      setLedBlue(false);
      setLedGreen(true);
      setLedRed(true);

      error.id = flash.board_id * 2 + i;
      error.info = BLDC_OVER_CURRENT;
      error.value = getCurrentMotor(i);
      waitPowerOnTimeout();
    }
  }

  // エンコーダー値飛んだときにエラーにする
  // 処理的には対策を入れているが、canの受信タイミングの影響で、キッカーの影響でエラーになってしまうかもしれないので今はエラーにしない。
  /* if (enc_error_watcher.detect_flag) {
    forceStopAllPwmOutputAndTimer();
    p("encoder error!!! ENC M%d diff %5d", enc_error_watcher.idx, enc_error_watcher.cnt);
    setLedBlue(true);
    setLedGreen(false);
    setLedRed(true);
    
    error.id = flash.board_id * 2 + enc_error_watcher.idx;
    error.info = BLDC_ENC_ERROR;
    error.value = motor_real[enc_error_watcher.idx].diff_cnt_max;
    waitPowerOnTimeout();
  }
  */

  if (getBatteryVoltage() < THR_BATTERY_UNVER_VOLTAGE) {
    forceStopAllPwmOutputAndTimer();
    p("UNDER voltage!! %6.3f", getBatteryVoltage());
    setLedBlue(true);
    setLedGreen(false);
    setLedRed(true);

    error.id = flash.board_id * 2;
    error.info = BLDC_UNDER_VOLTAGE;
    error.value = getBatteryVoltage();
    waitPowerOnTimeout();
  }

  if (getBatteryVoltage() > THR_BATTERY_OVER_VOLTAGE) {
    setPwmAll(0);
    //stopTimerInterrupt();
    p("OVER voltage!! %6.3f", getBatteryVoltage());
    setLedBlue(true);
    setLedGreen(false);
    setLedRed(true);

    error.id = flash.board_id * 2;
    error.info = BLDC_OVER_VOLTAGE;
    error.value = getBatteryVoltage();
    waitPowerOnTimeout();
  }

  if (getTempMotor(0) > THR_MOTOR_OVER_TEMPERATURE || getTempMotor(1) > THR_MOTOR_OVER_TEMPERATURE) {
    forceStopAllPwmOutputAndTimer();
    p("OVER Motor temperature!! M0 : %3d M1 : %3d", getTempMotor(0), getTempMotor(1));
    setLedBlue(true);
    setLedGreen(true);
    setLedRed(true);

    error.info = BLDC_MOTOR_OVER_HEAT;
    if (getTempMotor(0) > getTempMotor(1)) {
      error.id = flash.board_id * 2;
      error.value = (float)getTempMotor(0);
    } else {
      error.id = flash.board_id * 2 + 1;
      error.value = (float)getTempMotor(1);
    }
    waitPowerOnTimeout();
  }
  if ((getTempFET(0) > THR_FET_OVER_TEMPERATURE && getTempFET(0) < THR_NO_CONNECTED_TEPERATURE) || (getTempFET(1) > THR_FET_OVER_TEMPERATURE && getTempFET(1) < THR_NO_CONNECTED_TEPERATURE)) {
    forceStopAllPwmOutputAndTimer();
    p("OVER FET temperature!! M0 : %3df M1 : %3d", getTempFET(0), getTempFET(1));
    setLedBlue(true);
    setLedGreen(true);
    setLedRed(true);

    error.info = BLDC_FET_OVER_HEAT;
    if (getTempFET(0) > getTempFET(1)) {
      error.id = flash.board_id * 2 + 0;
      error.value = (float)getTempFET(0);
    } else {
      error.id = flash.board_id * 2 + 1;
      error.value = (float)getTempFET(1);
    }
    waitPowerOnTimeout();
  }

  if (pid[0].load_limit_cnt > MOTOR_OVER_LOAD_CNT_LIMIT || pid[1].load_limit_cnt > MOTOR_OVER_LOAD_CNT_LIMIT) {
    forceStopAllPwmOutputAndTimer();
    p("over load!! %d %d", pid[0].load_limit_cnt, pid[1].load_limit_cnt);
    setLedBlue(false);
    setLedGreen(false);
    setLedRed(true);

    error.info = BLDC_OVER_LOAD;
    if (pid[0].load_limit_cnt > pid[1].load_limit_cnt) {
      error.id = flash.board_id * 2 + 0;
      error.value = pid[0].load_limit_cnt;
    } else {
      error.id = flash.board_id * 2 + 1;
      error.value = pid[1].load_limit_cnt;
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

  sys.is_starting_mode = true;

  loadFlashData();
  p("\n\n** Orion VV driver V4 start! %s %s **\n", __DATE__, __TIME__);

  /*   for (int i = 0; i < 1025; i++) {
    float radian = (float)i / 0x400 * M_PI * 2;
    uint16_t rad_to_cnt = (uint16_t)(((float)(radian + M_PI * 4) / (M_PI * 2) * 0x400)) & 0x3FF;

    float output_sin[3];

    output_sin[0] = get_sin_table(rad_to_cnt);
    output_sin[1] = get_sin_table(rad_to_cnt + 341);
    output_sin[2] = get_sin_table(rad_to_cnt + 683);
    p("U-W %+8.5f U-V %+8.5f V-W %+8.5f\n", output_sin[0], output_sin[1], output_sin[2]);

    //p("idx:%4d sin_arr %+8.5f rad %+8.5f fsin %+8.5f asin %+8.5f\n", i, get_sin_table(i), (float)i / 0x400 * M_PI * 2, fast_sin((float)i / 0x400 * M_PI * 2), asinf(get_sin_table(i)));

    HAL_Delay(1);
  } */

  calib_process.force_rotation_speed = 0.005;
  sys.free_wheel_cnt = START_UP_FREE_WHEEL_CNT;

  for (int i = 0; i < 2; i++) {
    pid[i].pid_kp = 0.2;
    pid[i].pid_ki = 0.3;
    pid[i].pid_kd = 0.0;
    pid[i].error_integral_limit = 4.0;

    cmd[i].speed = 0;
    cmd[i].timeout_cnt = -1;

    cmd[i].out_v = 0;
    cmd[i].out_v_final = 0;

    // 手動調整による
    // 40rps/80rpsでも同等の効果のため、処理時間ではなく電気角度のオフセットと思われるので暫定的対処
    sys.manual_offset_radian = 0.00;

    // set calibration params
    enc_offset[i].zero_calib = flash.calib[i];

    // rps/v
    // 400kV : 6.66 rps/v
    // 200kV : 3.33 rps/v

    // v/rps
    // 400kV : 0.15 rps/v
    // 200kV : 0.3  rps/v

    if (1 < flash.rps_per_v_cw[i] && flash.rps_per_v_cw[i] < 10) {
      motor_param[i].voltage_per_rps = 1 / flash.rps_per_v_cw[i];
    } else {
      motor_param[i].voltage_per_rps = V_PER_RPS_DEFAULT;
    }

    // 2.0 -> 4.0 -> 6.0
    if (motor_param[i].voltage_per_rps < 0.25) {
      // 400kV
      // 4.0でも問題なかったけど一応6.0にしておく
      pid[i].diff_voltage_limit = 4.0;

    } else {
      // 200kV
      pid[i].diff_voltage_limit = 6.0;
    }
  }
  p("CAN ADDR 0x%03x\nenc offset M0 %6.3f M1 %6.3f\nRPS/V M0 %6.3f M1 %6.3f\n", flash.board_id, flash.calib[0], flash.calib[1], flash.rps_per_v_cw[0], flash.rps_per_v_cw[1]);
  HAL_Delay(1);
  p("Kv M0 %6.3f M1 %6.3f rpm/V\n", flash.rps_per_v_cw[0] * 60, flash.rps_per_v_cw[1] * 60);
  HAL_Delay(1);
  p("Diff voltage limit M0 %3.1fV M1 %3.1fV\n", pid[0].diff_voltage_limit, pid[1].diff_voltage_limit);

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

  motor_param[0].output_voltage_limit = SPEED_CMD_LIMIT_RPS / flash.rps_per_v_cw[0] * SPEED_REAL_LIMIT_GAIN;
  motor_param[1].output_voltage_limit = SPEED_CMD_LIMIT_RPS / flash.rps_per_v_cw[1] * SPEED_REAL_LIMIT_GAIN;
  p("output voltage limit : %5.2f %5.2f\n", motor_param[0].output_voltage_limit, motor_param[1].output_voltage_limit);
  HAL_Delay(1);

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

  p("ADC : %5d %5d GD %4.2f Batt %4.2f\n", adc_raw.cs_motor[0], adc_raw.cs_motor[1], getGateDriverDCDCVoltage(), getBatteryVoltage());
  if (adc_raw.cs_motor[0] < 100 && adc_raw.cs_motor[1] < 100) {
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
  // なぜか LOW→LOW→LOW→HIGH→HIGH→HIGHとやると止まるのでHIGH&LOWでやっている
  int turn_on_channel = 0;
  while (turn_on_channel < 3) {
    switch (turn_on_channel) {
      case 0:
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);     // M0 low
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);     // M1 low
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);  // M1 high & low
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);  // M0 high & low
        break;
      case 1:
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);     // M0 low
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);     // M1 low
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2);  // M1 high & low
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);  // M0 high & low
        break;
      case 2:
        HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);     // M0 low
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);     // M1 low
        HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3);  // M1 high & low
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);  // M0 high & low
        break;
      default:
        break;
    }

    interrupt_timer_cnt = 0;
    while (interrupt_timer_cnt < INTERRUPT_KHZ_1MS * 50) {
      if (isNotZeroCurrent() || getBatteryVoltage() < THR_BATTERY_UNVER_VOLTAGE) {
        forceStopAllPwmOutputAndTimer();
        p("fail check!! Current M0 %+6.3f M1 %+6.3f ch:%d\n", getCurrentMotor(0), getCurrentMotor(1), turn_on_channel);
        sys.power_enable_cnt = 500;
        waitPowerOnTimeout();
      }
    }
    p("ch:%2d CurrentCheck OK!! M0 %+6.3f M1 %+6.3f Battery %5.2f GD %5.2f\n", turn_on_channel, getCurrentMotor(0), getCurrentMotor(1), getBatteryVoltage(), getGateDriverDCDCVoltage());
    HAL_Delay(10);
    turn_on_channel++;
  }
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
    while (isPushedSW4());
  }

  CAN_Filter_Init(flash.board_id);

  HAL_CAN_Start(&hcan);
  p("start main loop!\n");

  setLedRed(false);
  setLedGreen(false);
  setLedBlue(false);

  sys.is_starting_mode = false;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    receiveUserSerialCommand();

    for (int i = 0; i < 2; i++) {
      int ret = calcMotorSpeed(&motor_real[i], &as5047p[i], &sys, &enc_error_watcher);
      if (ret < 0) {
        p("stop!! speed error");
      }
    }
    sendCanData();

    system_exec_time_stamp[0] = interrupt_timer_cnt;
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
    // 無回転時50%、回転時80%
    main_loop_remain_counter = INTERRUPT_KHZ_1MS - interrupt_timer_cnt;
    while (interrupt_timer_cnt <= INTERRUPT_KHZ_1MS);
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
