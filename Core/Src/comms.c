/**
 * @file comms.c
 * @brief CAN/UART command handling and telemetry transfer.
 */

#include "comms.h"

#include <math.h>
#include <stdbool.h>

#include "adc.h"
#include "app_context.h"
#include "calibration.h"
#include "can.h"
#include "control_mode.h"
#include "flash.h"
#include "motor.h"
#include "tim.h"
#include "usart.h"

static uint8_t uart_rx_buf[10] = {0};
static bool uart_rx_flag = false;

static uint32_t can_rx_cnt = 0;
static can_msg_buf_t can_rx_buf;
static CAN_RxHeaderTypeDef can_rx_header;

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

uint32_t getCanRxCount(void)
{
  return can_rx_cnt;
}

void clearCanRxCount(void)
{
  can_rx_cnt = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  (void)huart;
  uart_rx_flag = true;
}

void initComms(void)
{
  HAL_UART_Receive_IT(&huart1, uart_rx_buf, 1);
}

static void can_rx_callback(void)
{
  // Ignore command updates while any calibration is active.
  if (isAnyCalibrationActive()) {
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

      // Speed command IDs
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
      // Send motor and FET temperatures.
      sendTemperature(flash.board_id, 0, getTempMotor(0), getTempFET(0));
      break;
    case 10:
      sendTemperature(flash.board_id, 1, getTempMotor(1), getTempFET(1));
      break;
    case 12:
      // Calibration parameter
      sendFloatDual(0x500 + flash.board_id * 2, flash.rps_per_v_cw[0], 0);
      break;
    case 14:
      // Calibration parameter
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
