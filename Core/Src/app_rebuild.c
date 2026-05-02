/**
 * @file app_rebuild.c
 * @brief Rebuilt BLDC application layer. CubeMX/HAL peripheral code remains in each peripheral file.
 */

#include "app_rebuild.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "adc.h"
#include "can.h"
#include "control_limits.h"
#include "flash.h"
#include "foc_driver_hal.h"
#include "foc_math.h"
#include "gpio.h"
#include "io_check.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#define APP_MOTOR_COUNT (2U)
#define APP_PWM_ISR_PER_1MS (20U)
#define APP_POLE_PAIRS (6)
#define APP_SENSOR_DIRECTION (-1)
#define APP_RPS_LIMIT (80.0f)
#define APP_MIN_VOLTAGE_PER_RPS (0.02f)
#define APP_DEFAULT_VOLTAGE_PER_RPS (0.08f)
#define APP_OUTPUT_VOLTAGE_LIMIT (6.0f)
#define APP_ZERO_SLEEP_MS (5000U)
#define APP_SERIAL_SPEED_STEP_RPS (0.5f)
#define APP_CAN_TIMEOUT_MS (100U)

typedef struct
{
  app_mode_t mode;
  app_motor_state_t motor[APP_MOTOR_COUNT];
  volatile uint32_t pwm_irq_count;
  uint32_t freewheel_ms;
  uint32_t zero_output_ms;
  uint32_t print_ms;
  uint8_t print_page;
  uint16_t fault_id;
  uint16_t fault_info;
  float fault_value;
  uint32_t can_rx_count;
  bool io_check_request;
  bool can_started;
  uint8_t uart_rx_byte;
  volatile bool uart_rx_ready;
} app_state_t;

static app_state_t app;

static float clampFloat(float value, float limit)
{
  if (value > limit) {
    return limit;
  }
  if (value < -limit) {
    return -limit;
  }
  return value;
}

static bool validCalibFloat(float value)
{
  return isfinite(value) && fabsf(value) < 1000.0f;
}

static float encoderRawToMechanicalRad(int raw)
{
  return (float)raw * (2.0f * (float)M_PI / (float)ENC_CNT_MAX);
}

static void startPwmOutputsFreewheel(void)
{
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  setPwmAll(TIM_PWM_CENTER);

  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }

  setPwmOutPutFreeWheel();
}

static void startAdcInjected(void)
{
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ADCEx_InjectedStart(&hadc1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ADCEx_InjectedStart(&hadc2) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_ADCEx_InjectedStart(&hadc3) != HAL_OK) {
    Error_Handler();
  }
}

static void primeSensors(void)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  __HAL_SPI_ENABLE(&hspi1);

  for (int i = 0; i < 8; i++) {
    updateADC(0);
    updateADC(1);
    updateAS5047P(0);
    updateAS5047P(1);
  }

  uint32_t offset = 0;
  for (int i = 0; i < 32; i++) {
    updateADC(0);
    updateADC(1);
    offset += (uint32_t)adc_raw.cs_motor[0];
    offset += (uint32_t)adc_raw.cs_motor[1];
  }
  adc_raw.cs_adc_offset = (int)(offset / 64U);

  for (uint8_t m = 0; m < APP_MOTOR_COUNT; m++) {
    app.motor[m].pre_raw = as5047p[m].enc_raw;
  }
}

static void initMotorParameters(void)
{
  for (uint8_t m = 0; m < APP_MOTOR_COUNT; m++) {
    float rps_per_v = flash.rps_per_v_cw[m];
    float voltage_per_rps = APP_DEFAULT_VOLTAGE_PER_RPS;
    if (validCalibFloat(rps_per_v) && fabsf(rps_per_v) > APP_MIN_VOLTAGE_PER_RPS) {
      voltage_per_rps = 1.0f / fabsf(rps_per_v);
    }

    app.motor[m].target_rps = 0.0f;
    app.motor[m].measured_rps = 0.0f;
    app.motor[m].measured_rps_ave = 0.0f;
    app.motor[m].voltage_q = 0.0f;
    app.motor[m].voltage_per_rps = voltage_per_rps;
    app.motor[m].voltage_limit = APP_OUTPUT_VOLTAGE_LIMIT;
    app.motor[m].zero_electric_angle = validCalibFloat(flash.calib[m]) ? flash.calib[m] : 0.0f;
    app.motor[m].command_timeout_ms = 0;
    app.motor[m].output_limited = false;
  }
}

static void updateSpeed(uint8_t motor)
{
  int diff = app.motor[motor].pre_raw - as5047p[motor].enc_raw;
  if (diff < -HARF_OF_ENC_CNT_MAX) {
    diff += ENC_CNT_MAX;
  } else if (diff > HARF_OF_ENC_CNT_MAX) {
    diff -= ENC_CNT_MAX;
  }

  app.motor[motor].measured_rps = (float)diff / (float)ENC_CNT_MAX * 1000.0f;
  app.motor[motor].measured_rps_ave = app.motor[motor].measured_rps_ave * 0.98f + app.motor[motor].measured_rps * 0.02f;
  app.motor[motor].pre_raw = as5047p[motor].enc_raw;

  if (fabsf(app.motor[motor].measured_rps) > APP_RPS_LIMIT * 2.0f && app.mode == APP_MODE_RUN) {
    appForceFault(BLDC_ENC_ERROR, motor, app.motor[motor].measured_rps);
  }
}

static void updateOutputVoltage(void)
{
  if (app.mode != APP_MODE_RUN) {
    for (uint8_t m = 0; m < APP_MOTOR_COUNT; m++) {
      app.motor[m].voltage_q = 0.0f;
    }
    return;
  }

  for (uint8_t m = 0; m < APP_MOTOR_COUNT; m++) {
    if (app.motor[m].command_timeout_ms > 0U) {
      app.motor[m].command_timeout_ms--;
    } else {
      app.motor[m].target_rps = 0.0f;
    }

    float command_v = app.motor[m].target_rps * app.motor[m].voltage_per_rps;
    app.motor[m].voltage_q = clampFloat(command_v, app.motor[m].voltage_limit);
    app.motor[m].output_limited = (app.motor[m].voltage_q != command_v);
  }
}

static void applyProtection(void)
{
  const float batt = getBatteryVoltage();
  if (batt < THR_BATTERY_UNVER_VOLTAGE) {
    appForceFault(BLDC_UNDER_VOLTAGE, 0, batt);
    return;
  }
  if (batt > THR_BATTERY_OVER_VOLTAGE) {
    appForceFault(BLDC_OVER_VOLTAGE, 0, batt);
    return;
  }

  for (uint8_t m = 0; m < APP_MOTOR_COUNT; m++) {
    const float current = fabsf(getCurrentMotor(m));
    if (current > THR_MOTOR_OVER_CURRENT) {
      appForceFault(BLDC_OVER_CURRENT, m, current);
      return;
    }
    const int motor_temp = getTempMotor(m);
    const int fet_temp = getTempFET(m);
    if (motor_temp > THR_MOTOR_OVER_TEMPERATURE && motor_temp < THR_NO_CONNECTED_TEPERATURE) {
      appForceFault(BLDC_MOTOR_OVER_HEAT, m, (float)motor_temp);
      return;
    }
    if (fet_temp > THR_FET_OVER_TEMPERATURE && fet_temp < THR_NO_CONNECTED_TEPERATURE) {
      appForceFault(BLDC_FET_OVER_HEAT, m, (float)fet_temp);
      return;
    }
  }
}

static void applyPwmInIsr(uint8_t motor)
{
  updateADC(motor);
  updateAS5047P(motor);

  if (app.mode != APP_MODE_RUN || app.freewheel_ms > 0U) {
    foc_pwm_compare_t center = {TIM_PWM_CENTER, TIM_PWM_CENTER, TIM_PWM_CENTER, false};
    focDriverSetPwmCompare(motor, center);
    return;
  }

  const float batt = getBatteryVoltage();
  const float mech = encoderRawToMechanicalRad(as5047p[motor].enc_raw);
  const float electrical = focElectricalAngle(mech, APP_POLE_PAIRS, app.motor[motor].zero_electric_angle, APP_SENSOR_DIRECTION);
  focDriverApplySineVoltage(motor, app.motor[motor].voltage_q, 0.0f, electrical, batt);
}

static void handleFreewheelTimer(void)
{
  if (app.freewheel_ms > 0U) {
    app.freewheel_ms--;
    if (app.freewheel_ms == 0U && app.mode == APP_MODE_RUN) {
      resumePwmOutput();
    } else if (app.freewheel_ms == 0U && app.mode == APP_MODE_FREEWHEEL) {
      app.mode = APP_MODE_READY;
    }
  }
}

static void updateZeroOutputSleep(void)
{
  if (app.mode != APP_MODE_RUN) {
    return;
  }
  if (app.motor[0].target_rps == 0.0f && app.motor[1].target_rps == 0.0f) {
    if (app.zero_output_ms < APP_ZERO_SLEEP_MS) {
      app.zero_output_ms++;
    } else {
      setPwmOutPutFreeWheel();
    }
  } else {
    if (app.zero_output_ms >= APP_ZERO_SLEEP_MS) {
      resumePwmOutput();
    }
    app.zero_output_ms = 0;
  }
}

static void printDiagnostics(void)
{
  if (app.print_ms++ < 100U) {
    return;
  }
  app.print_ms = 0;

  switch (app.print_page) {
    case 0:
      p("Mode %d Batt %5.2f GD %5.2f Free %lu Fault 0x%04x/%u %+6.2f CAN %lu\n",
        app.mode,
        getBatteryVoltage(),
        getGateDriverDCDCVoltage(),
        app.freewheel_ms,
        app.fault_id,
        app.fault_info,
        app.fault_value,
        app.can_rx_count);
      break;
    case 1:
      p("M0 tgt %+6.2f rps %+6.2f vq %+5.2f enc %5d cur %+5.2f tmp %3d/%3d\n",
        app.motor[0].target_rps,
        app.motor[0].measured_rps_ave,
        app.motor[0].voltage_q,
        as5047p[0].enc_raw,
        getCurrentMotor(0),
        getTempMotor(0),
        getTempFET(0));
      break;
    default:
      p("M1 tgt %+6.2f rps %+6.2f vq %+5.2f enc %5d cur %+5.2f tmp %3d/%3d\n",
        app.motor[1].target_rps,
        app.motor[1].measured_rps_ave,
        app.motor[1].voltage_q,
        as5047p[1].enc_raw,
        getCurrentMotor(1),
        getTempMotor(1),
        getTempFET(1));
      break;
  }
}

static void sendTelemetry(void)
{
  static uint8_t transfer_cnt;

  if (!app.can_started) {
    return;
  }

  sendSpeed(flash.board_id, 0, app.motor[0].measured_rps_ave, encoderRawToMechanicalRad(as5047p[0].enc_raw));
  sendSpeed(flash.board_id, 1, app.motor[1].measured_rps_ave, encoderRawToMechanicalRad(as5047p[1].enc_raw));

  switch (transfer_cnt) {
    case 0:
      sendVoltage(flash.board_id, 0, getBatteryVoltage());
      break;
    case 2:
      sendVoltage(flash.board_id, 1, getGateDriverDCDCVoltage());
      break;
    case 4:
      sendCurrent(flash.board_id, 0, getCurrentMotor(0));
      break;
    case 6:
      sendCurrent(flash.board_id, 1, getCurrentMotor(1));
      break;
    case 8:
      sendTemperature(flash.board_id, 0, getTempMotor(0), getTempFET(0));
      break;
    case 10:
      sendTemperature(flash.board_id, 1, getTempMotor(1), getTempFET(1));
      break;
    case 12:
      sendFloatDual(0x500 + flash.board_id * 2, flash.rps_per_v_cw[0], flash.calib[0]);
      break;
    case 14:
      sendFloatDual(0x501 + flash.board_id * 2, flash.rps_per_v_cw[1], flash.calib[1]);
      break;
    default:
      break;
  }

  transfer_cnt++;
  if (transfer_cnt >= 50U) {
    transfer_cnt = 0;
  }
}

static void setTarget(uint8_t motor, float rps, uint16_t timeout_ms)
{
  if (motor >= APP_MOTOR_COUNT) {
    return;
  }
  app.motor[motor].target_rps = clampFloat(rps, APP_RPS_LIMIT);
  app.motor[motor].command_timeout_ms = timeout_ms;
  if (app.mode == APP_MODE_READY || (app.mode == APP_MODE_FREEWHEEL && app.freewheel_ms == 0U)) {
    appEnableRun();
  }
}

static void handleUartCommand(uint8_t cmd)
{
  switch (cmd) {
    case 'i':
      runIoCheckOnce();
      break;
    case 'm':
      runFocMathCheckOnce();
      break;
    case 'n':
      appEnableRun();
      p("run enabled, targets remain zero\n");
      break;
    case 'x':
    case '0':
      appSetFreewheelMs(60000U);
      p("freewheel 60s\n");
      break;
    case 'w':
      setTarget(0, app.motor[0].target_rps + APP_SERIAL_SPEED_STEP_RPS, 0xFFFFU);
      setTarget(1, app.motor[1].target_rps + APP_SERIAL_SPEED_STEP_RPS, 0xFFFFU);
      break;
    case 's':
      setTarget(0, app.motor[0].target_rps - APP_SERIAL_SPEED_STEP_RPS, 0xFFFFU);
      setTarget(1, app.motor[1].target_rps - APP_SERIAL_SPEED_STEP_RPS, 0xFFFFU);
      break;
    case ' ':
      setTarget(0, 0.0f, 0U);
      setTarget(1, 0.0f, 0U);
      appSetFreewheelMs(60000U);
      break;
    case '\n':
      app.print_page++;
      if (app.print_page > 2U) {
        app.print_page = 0;
      }
      p("print page %u\n", app.print_page);
      break;
    default:
      break;
  }
}

static void pollUart(void)
{
  if (!app.uart_rx_ready) {
    return;
  }
  app.uart_rx_ready = false;
  const uint8_t cmd = app.uart_rx_byte;
  HAL_UART_Receive_IT(&huart1, &app.uart_rx_byte, 1);
  handleUartCommand(cmd);
}

static void handleCanMessage(CAN_RxHeaderTypeDef * header, can_msg_buf_t * msg)
{
  app.can_rx_count++;

  switch (header->StdId) {
    case 0x010:
      if (msg->data[0] == 0U && msg->data[1] == 0U) {
        HAL_NVIC_SystemReset();
      }
      break;
    case 0x100:
    case 0x102:
      setTarget(0, msg->value[0], APP_CAN_TIMEOUT_MS);
      break;
    case 0x101:
    case 0x103:
      setTarget(1, msg->value[0], APP_CAN_TIMEOUT_MS);
      break;
    case 0x110:
      if (msg->data[0] == 3U) {
        appSetFreewheelMs((uint32_t)msg->data[2]);
      }
      break;
    case 0x320:
      app.io_check_request = true;
      break;
    default:
      break;
  }
}

static void waitForNextMainCycle(void)
{
  while (app.pwm_irq_count <= APP_PWM_ISR_PER_1MS) {
  }
  app.pwm_irq_count = 0;
}

void appInit(void)
{
  app.mode = APP_MODE_BOOT;
  setLedRed(true);
  setLedGreen(false);
  setLedBlue(false);

  loadFlashData();
  if (flash.board_id > 0x7FU) {
    flash.board_id = 1U;
  }

  startAdcInjected();
  primeSensors();
  initMotorParameters();
  startPwmOutputsFreewheel();

  CAN_Filter_Init((uint16_t)flash.board_id);
  if (HAL_CAN_Start(&hcan) == HAL_OK) {
    app.can_started = true;
  }

  HAL_UART_Receive_IT(&huart1, &app.uart_rx_byte, 1);

  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
    Error_Handler();
  }

  app.mode = APP_MODE_READY;
  appSetFreewheelMs(60000U);
  p("\n[APP] rebuild boot board 0x%03lx offset %+6.3f %+6.3f v/rps %+6.3f %+6.3f\n",
    flash.board_id,
    app.motor[0].zero_electric_angle,
    app.motor[1].zero_electric_angle,
    app.motor[0].voltage_per_rps,
    app.motor[1].voltage_per_rps);
}

void appTick1kHz(void)
{
  pollUart();

  if (app.io_check_request) {
    app.io_check_request = false;
    runIoCheckOnce();
  }

  updateSpeed(0);
  updateSpeed(1);
  handleFreewheelTimer();
  updateOutputVoltage();
  applyProtection();
  updateZeroOutputSleep();
  sendTelemetry();
  printDiagnostics();

  setLedRed(true);
  waitForNextMainCycle();
  setLedRed(false);
}

void appOnTimerElapsed(TIM_HandleTypeDef * htim)
{
  if (htim->Instance != TIM1) {
    return;
  }

  static uint8_t motor_select = 0U;
  app.pwm_irq_count++;
  motor_select ^= 1U;

  setLedBlue(false);
  applyPwmInIsr(motor_select);
  setLedBlue(true);
}

void appSetFreewheelMs(uint32_t ms)
{
  app.freewheel_ms = ms;
  app.motor[0].target_rps = 0.0f;
  app.motor[1].target_rps = 0.0f;
  app.motor[0].command_timeout_ms = 0U;
  app.motor[1].command_timeout_ms = 0U;
  app.motor[0].voltage_q = 0.0f;
  app.motor[1].voltage_q = 0.0f;
  setPwmAll(TIM_PWM_CENTER);
  setPwmOutPutFreeWheel();
  if (app.mode != APP_MODE_FAULT) {
    app.mode = APP_MODE_FREEWHEEL;
  }
}

void appEnableRun(void)
{
  if (app.mode == APP_MODE_FAULT) {
    p("cannot run: fault 0x%04x info %u value %+6.2f\n", app.fault_id, app.fault_info, app.fault_value);
    return;
  }
  app.freewheel_ms = 0U;
  app.zero_output_ms = 0U;
  setPwmAll(TIM_PWM_CENTER);
  resumePwmOutput();
  app.mode = APP_MODE_RUN;
}

void appForceFault(uint16_t id, uint16_t info, float value)
{
  if (app.mode == APP_MODE_FAULT) {
    return;
  }
  app.fault_id = id;
  app.fault_info = info;
  app.fault_value = value;
  app.mode = APP_MODE_FAULT;
  setTarget(0, 0.0f, 0U);
  setTarget(1, 0.0f, 0U);
  setPwmAll(TIM_PWM_CENTER);
  setPwmOutPutFreeWheel();
  sendError(id, info, value);
  p("FAULT 0x%04x info %u value %+6.2f\n", id, info, value);
}

uint32_t appGetCanRxCount(void)
{
  return app.can_rx_count;
}

app_mode_t appGetMode(void)
{
  return app.mode;
}

const app_motor_state_t * appGetMotorState(uint8_t motor)
{
  if (motor >= APP_MOTOR_COUNT) {
    return 0;
  }
  return &app.motor[motor];
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  if (huart->Instance == USART1) {
    app.uart_rx_ready = true;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan_arg)
{
  CAN_RxHeaderTypeDef header;
  can_msg_buf_t msg;
  if (HAL_CAN_GetRxMessage(hcan_arg, CAN_RX_FIFO0, &header, msg.data) == HAL_OK) {
    handleCanMessage(&header, &msg);
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef * hcan_arg)
{
  CAN_RxHeaderTypeDef header;
  can_msg_buf_t msg;
  if (HAL_CAN_GetRxMessage(hcan_arg, CAN_RX_FIFO1, &header, msg.data) == HAL_OK) {
    handleCanMessage(&header, &msg);
  }
}
