/**
 * @file bldc_app.c
 * @brief BLDC application layer. CubeMX/HAL peripheral code remains in each peripheral file.
 */

#include "bldc_app.h"

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
#define APP_PWM_ISR_PER_1MS (23U)
#define APP_POLE_PAIRS (12)
#define APP_SENSOR_DIRECTION (1)
#define APP_OPEN_LOOP_DIRECTION (-1)
#define APP_RPS_LIMIT (80.0f)
#define APP_MIN_VOLTAGE_PER_RPS (0.02f)
#define APP_DEFAULT_VOLTAGE_PER_RPS (0.08f)
#define APP_OUTPUT_VOLTAGE_LIMIT (12.0f)
#define APP_RPS_SLEW_PER_MS (0.01f)
#define APP_ZERO_SLEEP_MS (5000U)
#define APP_SERIAL_SPEED_STEP_RPS (0.5f)
#define APP_UART_RX_QUEUE_SIZE (32U)
#define APP_CAN_TIMEOUT_MS (100U)
#define APP_ALIGN_VOLTAGE (2.0f)
#define APP_ALIGN_MS (1000U)
#define APP_ALIGN_ELECTRICAL_ANGLE (4.71238898038f)
#define APP_SENSOR_DIAG_SCAN_MS (1200U)
#define APP_CAL_VOLTAGE_START (0.60f)
#define APP_CAL_VOLTAGE_STEP (0.20f)
#define APP_CAL_VOLTAGE_LIMIT (2.00f)
#define APP_CAL_SETTLE_MS (600U)
#define APP_CAL_SAMPLE_MS (900U)
#define APP_CAL_MIN_FIT_RPS (0.30f)
#define APP_CAL_MODEL_MARGIN (1.10f)
#define APP_CAL_MAX_OFFSET_V (3.00f)
#define APP_PROTECT_DEBOUNCE_MS (20U)
#define APP_SPEED_GLITCH_LIMIT_RPS (160.0f)
#define APP_SPEED_GLITCH_FAULT_COUNT (5U)

typedef enum
{
  APP_PARAM_CAL_STAGE_IDLE = 0,
  APP_PARAM_CAL_STAGE_ALIGN,
  APP_PARAM_CAL_STAGE_SAMPLE
} app_param_calib_stage_t;

typedef enum
{
  APP_SENSOR_DIAG_STAGE_IDLE = 0,
  APP_SENSOR_DIAG_STAGE_FORWARD,
  APP_SENSOR_DIAG_STAGE_REVERSE,
  APP_SENSOR_DIAG_STAGE_ALIGN
} app_sensor_diag_stage_t;

typedef struct
{
  bool active;
  bool save_to_flash;
  app_param_calib_stage_t stage;
  uint8_t motor;
  uint16_t elapsed_ms;
  float test_voltage;
  float speed_sum;
  uint16_t speed_count;
  float fit_sum_x;
  float fit_sum_y;
  float fit_sum_xx;
  float fit_sum_xy;
  uint8_t fit_count;
  float voltage_per_rps[APP_MOTOR_COUNT];
  float voltage_offset[APP_MOTOR_COUNT];
} app_param_calib_t;

typedef struct
{
  bool active;
  uint8_t motor;
  app_sensor_diag_stage_t stage;
  uint16_t elapsed_ms;
  int start_raw;
  int forward_raw;
  int reverse_raw;
  int forward_diff;
  int reverse_diff;
  float electrical_angle;
} app_sensor_diag_t;

typedef struct
{
  bldc_app_mode_t mode;
  bldc_app_motor_state_t motor[APP_MOTOR_COUNT];
  volatile uint32_t pwm_irq_count;
  uint32_t freewheel_ms;
  uint32_t zero_output_ms;
  uint8_t print_page;
  bool debug_print_request;
  bool foc_math_request;
  uint16_t fault_id;
  uint16_t fault_info;
  float fault_value;
  uint32_t can_rx_count;
  uint16_t align_ms;
  uint8_t align_motor;
  volatile bool align_active;
  bool open_loop_velocity;
  app_param_calib_t param_calib;
  app_sensor_diag_t sensor_diag;
  bool io_check_request;
  bool can_started;
  uint8_t uart_rx_byte;
  uint8_t uart_rx_queue[APP_UART_RX_QUEUE_SIZE];
  volatile uint8_t uart_rx_head;
  volatile uint8_t uart_rx_tail;
  volatile uint32_t uart_rx_overrun;
  uint32_t loop_busy_cycles;
  uint32_t loop_busy_cycles_max;
  uint32_t isr_cycles;
  uint32_t isr_cycles_max;
  uint32_t open_loop_cycle[APP_MOTOR_COUNT];
  uint32_t speed_cycle[APP_MOTOR_COUNT];
  uint32_t speed_glitch_count[APP_MOTOR_COUNT];
  uint16_t speed_glitch_streak[APP_MOTOR_COUNT];
  float speed_glitch_rps[APP_MOTOR_COUNT];
  int speed_glitch_diff[APP_MOTOR_COUNT];
  uint16_t under_voltage_ms;
  uint16_t over_voltage_ms;
  uint16_t over_current_ms[APP_MOTOR_COUNT];
  float cycle_to_s;
} app_state_t;

static app_state_t app;

static uint32_t cycleNow(void)
{
  return DWT->CYCCNT;
}

static uint32_t cycleDelta(uint32_t start, uint32_t end)
{
  return end - start;
}

static uint32_t cyclesToUs(uint32_t cycles)
{
  const uint32_t cycles_per_us = SystemCoreClock / 1000000U;
  if (cycles_per_us == 0U) {
    return 0U;
  }
  return cycles / cycles_per_us;
}

static void enableCycleCounter(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

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

static bool validVoltageOffset(float value)
{
  return isfinite(value) && value >= 0.0f && value <= APP_CAL_MAX_OFFSET_V;
}

static float encoderRawToMechanicalRad(int raw)
{
  return (float)raw * (2.0f * (float)M_PI / (float)ENC_CNT_MAX);
}

static int encoderRawDiff(int from_raw, int to_raw)
{
  int diff = to_raw - from_raw;
  if (diff < -HARF_OF_ENC_CNT_MAX) {
    diff += ENC_CNT_MAX;
  } else if (diff > HARF_OF_ENC_CNT_MAX) {
    diff -= ENC_CNT_MAX;
  }
  return diff;
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
    app.speed_cycle[m] = cycleNow();
  }
}

static void calibrateCurrentOffset(void)
{
  int64_t offset = 0;
  for (int i = 0; i < 128; i++) {
    updateADC(0);
    updateADC(1);
    offset += (int64_t)adc_raw.cs_motor[0];
    offset += (int64_t)adc_raw.cs_motor[1];
  }
  adc_raw.cs_adc_offset = (int)(offset / 256);
  p("current offset raw %d cur %+5.2f %+5.2f\n",
    adc_raw.cs_adc_offset,
    getCurrentMotor(0),
    getCurrentMotor(1));
}

static void initMotorParameters(void)
{
  for (uint8_t m = 0; m < APP_MOTOR_COUNT; m++) {
    float rps_per_v = flash.rps_per_v_cw[m];
    float voltage_per_rps = APP_DEFAULT_VOLTAGE_PER_RPS;
    if (validCalibFloat(rps_per_v) && fabsf(rps_per_v) > APP_MIN_VOLTAGE_PER_RPS) {
      voltage_per_rps = 1.0f / fabsf(rps_per_v);
    }
    const float voltage_offset = validVoltageOffset(flash.voltage_offset[m]) ? flash.voltage_offset[m] : 0.0f;

    app.motor[m].target_rps = 0.0f;
    app.motor[m].command_rps = 0.0f;
    app.motor[m].measured_rps = 0.0f;
    app.motor[m].measured_rps_ave = 0.0f;
    app.motor[m].voltage_q = 0.0f;
    app.motor[m].voltage_per_rps = voltage_per_rps;
    app.motor[m].voltage_offset = voltage_offset;
    app.motor[m].voltage_limit = APP_OUTPUT_VOLTAGE_LIMIT;
    app.motor[m].zero_electric_angle = validCalibFloat(flash.calib[m]) ? flash.calib[m] : 0.0f;
    app.motor[m].open_loop_electrical_angle = 0.0f;
    app.motor[m].command_timeout_ms = 0;
    app.motor[m].output_limited = false;
  }
}

static void updateSpeed(uint8_t motor)
{
  const uint32_t now = cycleNow();
  const uint32_t elapsed_cycles = cycleDelta(app.speed_cycle[motor], now);
  app.speed_cycle[motor] = now;
  const float elapsed_s = (float)elapsed_cycles * app.cycle_to_s;
  int diff = encoderRawDiff(as5047p[motor].enc_raw, app.motor[motor].pre_raw);

  if (elapsed_s > 0.0f) {
    app.motor[motor].measured_rps = ((float)diff / (float)ENC_CNT_MAX) / elapsed_s;
  }
  app.motor[motor].measured_rps_ave = app.motor[motor].measured_rps_ave * 0.98f + app.motor[motor].measured_rps * 0.02f;
  app.motor[motor].pre_raw = as5047p[motor].enc_raw;

  if (fabsf(app.motor[motor].measured_rps) > APP_RPS_LIMIT * 2.0f && app.mode == BLDC_APP_MODE_RUN) {
    bldcAppForceFault(BLDC_ENC_ERROR, motor, app.motor[motor].measured_rps);
  }
}

static void updateOutputVoltage(void)
{
  if (app.mode != BLDC_APP_MODE_RUN) {
    for (uint8_t m = 0; m < APP_MOTOR_COUNT; m++) {
      app.motor[m].voltage_q = 0.0f;
    }
    return;
  }

  for (uint8_t m = 0; m < APP_MOTOR_COUNT; m++) {
    if (app.param_calib.active) {
      if (m == app.param_calib.motor) {
        app.motor[m].command_rps = 0.0f;
        app.motor[m].target_rps = 0.0f;
        app.motor[m].voltage_q = app.param_calib.test_voltage;
      } else {
        app.motor[m].command_rps = 0.0f;
        app.motor[m].target_rps = 0.0f;
        app.motor[m].voltage_q = 0.0f;
      }
      app.motor[m].command_timeout_ms = 0U;
      app.motor[m].output_limited = false;
      continue;
    }

    if (app.motor[m].command_timeout_ms == 0xFFFFU) {
      /* UART manual command is held until another UART command changes it. */
    } else if (app.motor[m].command_timeout_ms > 0U) {
      app.motor[m].command_timeout_ms--;
    } else {
      app.motor[m].command_rps = 0.0f;
      app.motor[m].target_rps = 0.0f;
    }

    const float error = app.motor[m].command_rps - app.motor[m].target_rps;
    if (error > APP_RPS_SLEW_PER_MS) {
      app.motor[m].target_rps += APP_RPS_SLEW_PER_MS;
    } else if (error < -APP_RPS_SLEW_PER_MS) {
      app.motor[m].target_rps -= APP_RPS_SLEW_PER_MS;
    } else {
      app.motor[m].target_rps = app.motor[m].command_rps;
    }

    float command_v = app.motor[m].target_rps * app.motor[m].voltage_per_rps;
    if (app.motor[m].target_rps > 0.0f) {
      command_v += app.motor[m].voltage_offset;
    } else if (app.motor[m].target_rps < 0.0f) {
      command_v -= app.motor[m].voltage_offset;
    }
    app.motor[m].voltage_q = clampFloat(command_v, app.motor[m].voltage_limit);
    app.motor[m].output_limited = (app.motor[m].voltage_q != command_v);
  }
}

static void updateOpenLoopVelocity(void)
{
  if (app.mode != BLDC_APP_MODE_RUN || !app.open_loop_velocity) {
    return;
  }
}

static void updateOpenLoopVelocityInIsr(uint8_t motor)
{
  if (app.mode != BLDC_APP_MODE_RUN || !app.open_loop_velocity) {
    app.open_loop_cycle[motor] = cycleNow();
    return;
  }
  const uint32_t now = cycleNow();
  const uint32_t elapsed_cycles = cycleDelta(app.open_loop_cycle[motor], now);
  app.open_loop_cycle[motor] = now;

  const float direction = (APP_OPEN_LOOP_DIRECTION < 0) ? -1.0f : 1.0f;
  const float motor_period_s = (float)elapsed_cycles * app.cycle_to_s;
  const float delta =
    direction * app.motor[motor].target_rps * (2.0f * (float)M_PI) * (float)APP_POLE_PAIRS * motor_period_s;
  app.motor[motor].open_loop_electrical_angle = focNormalizeAngle(app.motor[motor].open_loop_electrical_angle + delta);
}

static void applyProtection(void)
{
  const float batt = getBatteryVoltage();
  if (batt < THR_BATTERY_UNVER_VOLTAGE) {
    if (++app.under_voltage_ms >= APP_PROTECT_DEBOUNCE_MS) {
      bldcAppForceFault(BLDC_UNDER_VOLTAGE, 0, batt);
      return;
    }
  } else {
    app.under_voltage_ms = 0U;
  }
  if (batt > THR_BATTERY_OVER_VOLTAGE) {
    if (++app.over_voltage_ms >= APP_PROTECT_DEBOUNCE_MS) {
      bldcAppForceFault(BLDC_OVER_VOLTAGE, 0, batt);
      return;
    }
  } else {
    app.over_voltage_ms = 0U;
  }

  for (uint8_t m = 0; m < APP_MOTOR_COUNT; m++) {
    const float current = fabsf(getCurrentMotor(m));
    if (current > THR_MOTOR_OVER_CURRENT) {
      if (++app.over_current_ms[m] >= APP_PROTECT_DEBOUNCE_MS) {
        bldcAppForceFault(BLDC_OVER_CURRENT, m, current);
        return;
      }
    } else {
      app.over_current_ms[m] = 0U;
    }
    const int motor_temp = getTempMotor(m);
    const int fet_temp = getTempFET(m);
    if (motor_temp > THR_MOTOR_OVER_TEMPERATURE && motor_temp < THR_NO_CONNECTED_TEPERATURE) {
      bldcAppForceFault(BLDC_MOTOR_OVER_HEAT, m, (float)motor_temp);
      return;
    }
    if (fet_temp > THR_FET_OVER_TEMPERATURE && fet_temp < THR_NO_CONNECTED_TEPERATURE) {
      bldcAppForceFault(BLDC_FET_OVER_HEAT, m, (float)fet_temp);
      return;
    }
  }
}

static void applyPwmInIsr(uint8_t motor)
{
  updateADC(motor);
  updateAS5047P(motor);

  if (app.sensor_diag.active) {
    if (motor == app.sensor_diag.motor) {
      focDriverApplySineVoltage(motor, APP_ALIGN_VOLTAGE, 0.0f, app.sensor_diag.electrical_angle, getBatteryVoltage());
    } else {
      foc_pwm_compare_t center = {TIM_PWM_CENTER, TIM_PWM_CENTER, TIM_PWM_CENTER, false};
      focDriverSetPwmCompare(motor, center);
    }
    return;
  }

  if (app.align_active || (app.param_calib.active && app.param_calib.stage == APP_PARAM_CAL_STAGE_ALIGN)) {
    const uint8_t align_motor = app.align_active ? app.align_motor : app.param_calib.motor;
    if (motor == align_motor) {
      focDriverApplySineVoltage(motor, APP_ALIGN_VOLTAGE, 0.0f, APP_ALIGN_ELECTRICAL_ANGLE, getBatteryVoltage());
    } else {
      foc_pwm_compare_t center = {TIM_PWM_CENTER, TIM_PWM_CENTER, TIM_PWM_CENTER, false};
      focDriverSetPwmCompare(motor, center);
    }
    return;
  }

  if (app.mode != BLDC_APP_MODE_RUN || app.freewheel_ms > 0U) {
    foc_pwm_compare_t center = {TIM_PWM_CENTER, TIM_PWM_CENTER, TIM_PWM_CENTER, false};
    focDriverSetPwmCompare(motor, center);
    return;
  }

  const float batt = getBatteryVoltage();
  updateOpenLoopVelocityInIsr(motor);
  float electrical = app.motor[motor].open_loop_electrical_angle;
  if (!app.open_loop_velocity) {
    const float mech = encoderRawToMechanicalRad(as5047p[motor].enc_raw);
    electrical = focElectricalAngle(mech, APP_POLE_PAIRS, app.motor[motor].zero_electric_angle, APP_SENSOR_DIRECTION);
  }
  focDriverApplySineVoltage(motor, app.motor[motor].voltage_q, 0.0f, electrical, batt);
}

static void handleFreewheelTimer(void)
{
  if (app.freewheel_ms > 0U) {
    app.freewheel_ms--;
    if (app.freewheel_ms == 0U && app.mode == BLDC_APP_MODE_RUN) {
      resumePwmOutput();
    } else if (app.freewheel_ms == 0U && app.mode == BLDC_APP_MODE_FREEWHEEL) {
      app.mode = BLDC_APP_MODE_READY;
    }
  }
}

static void updateZeroOutputSleep(void)
{
  if (app.align_active) {
    return;
  }
  if (app.sensor_diag.active) {
    return;
  }
  if (app.param_calib.active) {
    return;
  }
  if (app.mode != BLDC_APP_MODE_RUN) {
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

static void requestDebugPrint(uint8_t page)
{
  app.print_page = page;
  app.debug_print_request = true;
}

static void printDiagnostics(void)
{
  switch (app.print_page) {
    case 0:
      p("Mode %d OL %u Batt %5.2f GD %5.2f Free %lu Fault 0x%04x/%u %+6.2f CAN %lu loop %lu/%lu us slack %ld us isr %lu/%lu us\n",
        app.mode,
        app.open_loop_velocity ? 1U : 0U,
        getBatteryVoltage(),
        getGateDriverDCDCVoltage(),
        app.freewheel_ms,
        app.fault_id,
        app.fault_info,
        app.fault_value,
        app.can_rx_count,
        cyclesToUs(app.loop_busy_cycles),
        cyclesToUs(app.loop_busy_cycles_max),
        1000L - (long)cyclesToUs(app.loop_busy_cycles_max),
        cyclesToUs(app.isr_cycles),
        cyclesToUs(app.isr_cycles_max));
      app.loop_busy_cycles_max = app.loop_busy_cycles;
      app.isr_cycles_max = app.isr_cycles;
      break;
    case 1:
      p("M0 cmd %+6.2f tgt %+6.2f rps %+6.2f vq %+5.2f enc %5d cur %+5.2f tmp %3d/%3d\n",
        app.motor[0].command_rps,
        app.motor[0].target_rps,
        app.motor[0].measured_rps_ave,
        app.motor[0].voltage_q,
        as5047p[0].enc_raw,
        getCurrentMotor(0),
        getTempMotor(0),
        getTempFET(0));
      break;
    default:
      p("M1 cmd %+6.2f tgt %+6.2f rps %+6.2f vq %+5.2f enc %5d cur %+5.2f tmp %3d/%3d\n",
        app.motor[1].command_rps,
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

static void processDeferredDebugOutput(void)
{
  if (app.io_check_request) {
    app.io_check_request = false;
    runIoCheckOnce();
    return;
  }
  if (app.foc_math_request) {
    app.foc_math_request = false;
    runFocMathCheckOnce();
    return;
  }
  if (app.debug_print_request) {
    app.debug_print_request = false;
    printDiagnostics();
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
  app.motor[motor].command_rps = clampFloat(rps, APP_RPS_LIMIT);
  app.motor[motor].command_timeout_ms = timeout_ms;
  if (timeout_ms == 0U) {
    app.motor[motor].command_rps = 0.0f;
    app.motor[motor].target_rps = 0.0f;
  }
  if (app.motor[motor].command_rps != 0.0f &&
      (app.mode == BLDC_APP_MODE_READY || app.mode == BLDC_APP_MODE_FREEWHEEL)) {
    bldcAppEnableRun();
  }
}

static void startSensorAlignment(uint8_t motor)
{
  if (motor >= APP_MOTOR_COUNT || app.mode == BLDC_APP_MODE_FAULT) {
    return;
  }

  app.motor[0].target_rps = 0.0f;
  app.motor[1].target_rps = 0.0f;
  app.motor[0].command_rps = 0.0f;
  app.motor[1].command_rps = 0.0f;
  app.motor[0].command_timeout_ms = 0U;
  app.motor[1].command_timeout_ms = 0U;
  app.motor[0].voltage_q = 0.0f;
  app.motor[1].voltage_q = 0.0f;
  app.align_motor = motor;
  app.align_ms = APP_ALIGN_MS;
  app.freewheel_ms = 0U;
  app.zero_output_ms = 0U;
  setPwmAll(TIM_PWM_CENTER);
  resumePwmOutput();
  app.mode = BLDC_APP_MODE_RUN;
  app.align_active = true;
  p("align M%u start: uq %+4.1f angle %+4.2f\n", motor, APP_ALIGN_VOLTAGE, APP_ALIGN_ELECTRICAL_ANGLE);
}

static void updateSensorAlignment(void)
{
  if (!app.align_active) {
    return;
  }

  if (app.align_ms > 0U) {
    app.align_ms--;
    return;
  }

  const uint8_t motor = app.align_motor;
  const float shaft = encoderRawToMechanicalRad(as5047p[motor].enc_raw);
  const float direction = (APP_SENSOR_DIRECTION < 0) ? -1.0f : 1.0f;
  app.motor[motor].zero_electric_angle = focNormalizeAngle(direction * (float)APP_POLE_PAIRS * shaft);
  app.align_active = false;
  bldcAppSetFreewheelMs(60000U);
  p("align M%u done: raw %5d zero %+6.3f\n", motor, as5047p[motor].enc_raw, app.motor[motor].zero_electric_angle);
}

static void initOpenLoopAnglesFromSensor(void)
{
  const uint32_t now = cycleNow();
  for (uint8_t m = 0; m < APP_MOTOR_COUNT; m++) {
    const float mech = encoderRawToMechanicalRad(as5047p[m].enc_raw);
    app.motor[m].open_loop_electrical_angle = focElectricalAngle(mech, APP_POLE_PAIRS, app.motor[m].zero_electric_angle, APP_SENSOR_DIRECTION);
    app.open_loop_cycle[m] = now;
    app.speed_cycle[m] = now;
  }
}

static void startSensorDiagStage(app_sensor_diag_stage_t stage)
{
  app.sensor_diag.stage = stage;
  app.sensor_diag.elapsed_ms = 0U;
  if (stage == APP_SENSOR_DIAG_STAGE_FORWARD) {
    app.sensor_diag.start_raw = as5047p[app.sensor_diag.motor].enc_raw;
    app.sensor_diag.electrical_angle = APP_ALIGN_ELECTRICAL_ANGLE;
    p("sensor diag M%u forward start raw %5d\n", app.sensor_diag.motor, app.sensor_diag.start_raw);
  } else if (stage == APP_SENSOR_DIAG_STAGE_REVERSE) {
    app.sensor_diag.electrical_angle = focNormalizeAngle(APP_ALIGN_ELECTRICAL_ANGLE + 2.0f * (float)M_PI);
    p("sensor diag M%u reverse start raw %5d\n", app.sensor_diag.motor, app.sensor_diag.forward_raw);
  } else if (stage == APP_SENSOR_DIAG_STAGE_ALIGN) {
    app.sensor_diag.electrical_angle = APP_ALIGN_ELECTRICAL_ANGLE;
    p("sensor diag M%u zero align raw %5d\n", app.sensor_diag.motor, as5047p[app.sensor_diag.motor].enc_raw);
  }
}

static void finishSensorDiagMotor(void)
{
  const uint8_t motor = app.sensor_diag.motor;
  const float shaft = encoderRawToMechanicalRad(as5047p[motor].enc_raw);
  const float configured_direction = (APP_SENSOR_DIRECTION < 0) ? -1.0f : 1.0f;
  const int inferred_direction = (app.sensor_diag.forward_diff >= 0) ? 1 : -1;
  const bool direction_matches = (inferred_direction == APP_SENSOR_DIRECTION);

  app.motor[motor].zero_electric_angle =
    focNormalizeAngle(configured_direction * (float)APP_POLE_PAIRS * shaft);

  p("sensor diag M%u result fwd %+6d rev %+6d inferred %+d config %+d match %u zero %+6.3f\n",
    motor,
    app.sensor_diag.forward_diff,
    app.sensor_diag.reverse_diff,
    inferred_direction,
    APP_SENSOR_DIRECTION,
    direction_matches ? 1U : 0U,
    app.motor[motor].zero_electric_angle);

  app.sensor_diag.motor++;
  if (app.sensor_diag.motor < APP_MOTOR_COUNT) {
    startSensorDiagStage(APP_SENSOR_DIAG_STAGE_FORWARD);
    return;
  }

  app.sensor_diag.active = false;
  app.sensor_diag.stage = APP_SENSOR_DIAG_STAGE_IDLE;
  app.open_loop_velocity = true;
  initOpenLoopAnglesFromSensor();
  bldcAppSetFreewheelMs(60000U);
  p("sensor diag done\n");
}

static void updateSensorDiag(void)
{
  if (!app.sensor_diag.active) {
    return;
  }

  const uint8_t motor = app.sensor_diag.motor;
  if (app.sensor_diag.stage == APP_SENSOR_DIAG_STAGE_FORWARD) {
    if (app.sensor_diag.elapsed_ms < APP_SENSOR_DIAG_SCAN_MS) {
      const float ratio = (float)app.sensor_diag.elapsed_ms / (float)APP_SENSOR_DIAG_SCAN_MS;
      app.sensor_diag.electrical_angle =
        focNormalizeAngle(APP_ALIGN_ELECTRICAL_ANGLE + ratio * 2.0f * (float)M_PI);
      app.sensor_diag.elapsed_ms++;
      return;
    }

    app.sensor_diag.forward_raw = as5047p[motor].enc_raw;
    app.sensor_diag.forward_diff = encoderRawDiff(app.sensor_diag.start_raw, app.sensor_diag.forward_raw);
    p("sensor diag M%u forward end raw %5d diff %+6d\n", motor, app.sensor_diag.forward_raw, app.sensor_diag.forward_diff);
    startSensorDiagStage(APP_SENSOR_DIAG_STAGE_REVERSE);
    return;
  }

  if (app.sensor_diag.stage == APP_SENSOR_DIAG_STAGE_REVERSE) {
    if (app.sensor_diag.elapsed_ms < APP_SENSOR_DIAG_SCAN_MS) {
      const float ratio = (float)app.sensor_diag.elapsed_ms / (float)APP_SENSOR_DIAG_SCAN_MS;
      app.sensor_diag.electrical_angle =
        focNormalizeAngle(APP_ALIGN_ELECTRICAL_ANGLE + (1.0f - ratio) * 2.0f * (float)M_PI);
      app.sensor_diag.elapsed_ms++;
      return;
    }

    app.sensor_diag.reverse_raw = as5047p[motor].enc_raw;
    app.sensor_diag.reverse_diff = encoderRawDiff(app.sensor_diag.forward_raw, app.sensor_diag.reverse_raw);
    p("sensor diag M%u reverse end raw %5d diff %+6d\n", motor, app.sensor_diag.reverse_raw, app.sensor_diag.reverse_diff);
    startSensorDiagStage(APP_SENSOR_DIAG_STAGE_ALIGN);
    return;
  }

  if (app.sensor_diag.stage == APP_SENSOR_DIAG_STAGE_ALIGN) {
    if (app.sensor_diag.elapsed_ms++ < APP_ALIGN_MS) {
      return;
    }
    finishSensorDiagMotor();
  }
}

static void startSensorDiag(void)
{
  if (app.mode == BLDC_APP_MODE_FAULT) {
    p("cannot run sensor diag: fault 0x%04x\n", app.fault_id);
    return;
  }

  app.align_active = false;
  app.param_calib.active = false;
  app.sensor_diag.active = true;
  app.sensor_diag.motor = 0U;
  app.open_loop_velocity = false;
  app.freewheel_ms = 0U;
  app.zero_output_ms = 0U;
  app.motor[0].target_rps = 0.0f;
  app.motor[1].target_rps = 0.0f;
  app.motor[0].command_rps = 0.0f;
  app.motor[1].command_rps = 0.0f;
  setPwmAll(TIM_PWM_CENTER);
  resumePwmOutput();
  app.mode = BLDC_APP_MODE_RUN;
  p("sensor diag start scan_ms %u uq %+4.1f\n", APP_SENSOR_DIAG_SCAN_MS, APP_ALIGN_VOLTAGE);
  startSensorDiagStage(APP_SENSOR_DIAG_STAGE_FORWARD);
}

static void resetParamFit(uint8_t motor)
{
  app.param_calib.fit_sum_x = 0.0f;
  app.param_calib.fit_sum_y = 0.0f;
  app.param_calib.fit_sum_xx = 0.0f;
  app.param_calib.fit_sum_xy = 0.0f;
  app.param_calib.fit_count = 0U;
  app.motor[motor].measured_rps_ave = 0.0f;
}

static void startParamCalibAlign(void)
{
  app.param_calib.stage = APP_PARAM_CAL_STAGE_ALIGN;
  app.param_calib.elapsed_ms = 0U;
  app.param_calib.test_voltage = 0.0f;
  resetParamFit(app.param_calib.motor);
  p("param cal M%u align uq %+4.1f\n", app.param_calib.motor, APP_ALIGN_VOLTAGE);
}

static void startParamCalibSample(float voltage)
{
  app.param_calib.stage = APP_PARAM_CAL_STAGE_SAMPLE;
  app.param_calib.elapsed_ms = 0U;
  app.param_calib.test_voltage = voltage;
  app.param_calib.speed_sum = 0.0f;
  app.param_calib.speed_count = 0U;
  app.motor[app.param_calib.motor].measured_rps_ave = 0.0f;
  p("param cal M%u vq %+4.2f\n", app.param_calib.motor, voltage);
}

static bool fitParamCalibMotor(uint8_t motor)
{
  const float n = (float)app.param_calib.fit_count;
  const float denom = n * app.param_calib.fit_sum_xx - app.param_calib.fit_sum_x * app.param_calib.fit_sum_x;
  if (app.param_calib.fit_count < 2U || fabsf(denom) < 0.0001f) {
    p("param cal M%u fit NG count %u\n", motor, app.param_calib.fit_count);
    return false;
  }

  float voltage_per_rps =
    (n * app.param_calib.fit_sum_xy - app.param_calib.fit_sum_x * app.param_calib.fit_sum_y) / denom;
  float voltage_offset =
    (app.param_calib.fit_sum_y - voltage_per_rps * app.param_calib.fit_sum_x) / n;

  if (!isfinite(voltage_per_rps) || voltage_per_rps < APP_MIN_VOLTAGE_PER_RPS) {
    p("param cal M%u slope NG %+6.3f\n", motor, voltage_per_rps);
    return false;
  }
  if (!isfinite(voltage_offset) || voltage_offset < 0.0f) {
    voltage_offset = 0.0f;
  }
  if (voltage_offset > APP_CAL_MAX_OFFSET_V) {
    voltage_offset = APP_CAL_MAX_OFFSET_V;
  }

  app.param_calib.voltage_per_rps[motor] = voltage_per_rps * APP_CAL_MODEL_MARGIN;
  app.param_calib.voltage_offset[motor] = voltage_offset * APP_CAL_MODEL_MARGIN;
  p("param cal M%u model off %+5.2f v/rps %+6.3f points %u\n",
    motor,
    app.param_calib.voltage_offset[motor],
    app.param_calib.voltage_per_rps[motor],
    app.param_calib.fit_count);
  return true;
}

static void finishParamCalibration(void)
{
  const float rps_per_v0 = 1.0f / app.param_calib.voltage_per_rps[0];
  const float rps_per_v1 = 1.0f / app.param_calib.voltage_per_rps[1];
  const bool save = app.param_calib.save_to_flash;

  app.motor[0].voltage_per_rps = app.param_calib.voltage_per_rps[0];
  app.motor[1].voltage_per_rps = app.param_calib.voltage_per_rps[1];
  app.motor[0].voltage_offset = app.param_calib.voltage_offset[0];
  app.motor[1].voltage_offset = app.param_calib.voltage_offset[1];
  app.param_calib.active = false;
  app.param_calib.stage = APP_PARAM_CAL_STAGE_IDLE;
  app.open_loop_velocity = true;
  initOpenLoopAnglesFromSensor();
  bldcAppSetFreewheelMs(60000U);

  if (save) {
    writeMotorModelValue(rps_per_v0, rps_per_v1, app.motor[0].voltage_offset, app.motor[1].voltage_offset);
  }

  p("param cal done off %+5.2f %+5.2f v/rps %+6.3f %+6.3f rps/v %+6.3f %+6.3f save %u\n",
    app.motor[0].voltage_offset,
    app.motor[1].voltage_offset,
    app.motor[0].voltage_per_rps,
    app.motor[1].voltage_per_rps,
    rps_per_v0,
    rps_per_v1,
    save ? 1U : 0U);
}

static void advanceParamCalibrationMotor(void)
{
  const uint8_t motor = app.param_calib.motor;
  if (!fitParamCalibMotor(motor)) {
    app.param_calib.voltage_per_rps[motor] = app.motor[motor].voltage_per_rps;
    app.param_calib.voltage_offset[motor] = app.motor[motor].voltage_offset;
  }

  app.param_calib.motor++;
  if (app.param_calib.motor < APP_MOTOR_COUNT) {
    startParamCalibAlign();
    return;
  }

  finishParamCalibration();
}

static void updateParamCalibration(void)
{
  if (!app.param_calib.active) {
    return;
  }

  const uint8_t motor = app.param_calib.motor;
  if (app.param_calib.stage == APP_PARAM_CAL_STAGE_ALIGN) {
    if (app.param_calib.elapsed_ms++ < APP_ALIGN_MS) {
      return;
    }

    const float shaft = encoderRawToMechanicalRad(as5047p[motor].enc_raw);
    const float direction = (APP_SENSOR_DIRECTION < 0) ? -1.0f : 1.0f;
    app.motor[motor].zero_electric_angle = focNormalizeAngle(direction * (float)APP_POLE_PAIRS * shaft);
    p("param cal M%u align done raw %5d zero %+6.3f\n", motor, as5047p[motor].enc_raw, app.motor[motor].zero_electric_angle);
    startParamCalibSample(APP_CAL_VOLTAGE_START);
    return;
  }

  if (app.param_calib.stage != APP_PARAM_CAL_STAGE_SAMPLE) {
    return;
  }

  if (app.param_calib.elapsed_ms >= APP_CAL_SETTLE_MS) {
    app.param_calib.speed_sum += fabsf(app.motor[motor].measured_rps);
    if (app.param_calib.speed_count < 0xFFFFU) {
      app.param_calib.speed_count++;
    }
  }

  app.param_calib.elapsed_ms++;
  if (app.param_calib.elapsed_ms < (APP_CAL_SETTLE_MS + APP_CAL_SAMPLE_MS)) {
    return;
  }

  const float count = (app.param_calib.speed_count == 0U) ? 1.0f : (float)app.param_calib.speed_count;
  const float measured = app.param_calib.speed_sum / count;
  p("param cal M%u point vq %+4.2f rps %+6.3f\n", motor, app.param_calib.test_voltage, measured);

  if (measured >= APP_CAL_MIN_FIT_RPS) {
    app.param_calib.fit_sum_x += measured;
    app.param_calib.fit_sum_y += app.param_calib.test_voltage;
    app.param_calib.fit_sum_xx += measured * measured;
    app.param_calib.fit_sum_xy += measured * app.param_calib.test_voltage;
    app.param_calib.fit_count++;
  }

  const float next_voltage = app.param_calib.test_voltage + APP_CAL_VOLTAGE_STEP;
  if (next_voltage <= APP_CAL_VOLTAGE_LIMIT + 0.001f) {
    startParamCalibSample(next_voltage);
    return;
  }

  advanceParamCalibrationMotor();
}

static void startParamCalibration(bool save_to_flash)
{
  app.param_calib.active = true;
  app.param_calib.save_to_flash = save_to_flash;
  app.param_calib.motor = 0U;
  app.param_calib.voltage_per_rps[0] = app.motor[0].voltage_per_rps;
  app.param_calib.voltage_per_rps[1] = app.motor[1].voltage_per_rps;
  app.param_calib.voltage_offset[0] = app.motor[0].voltage_offset;
  app.param_calib.voltage_offset[1] = app.motor[1].voltage_offset;
  app.open_loop_velocity = false;
  app.freewheel_ms = 0U;
  app.zero_output_ms = 0U;
  setPwmAll(TIM_PWM_CENTER);
  resumePwmOutput();
  app.mode = BLDC_APP_MODE_RUN;
  p("param cal start vq sweep save %u\n", save_to_flash ? 1U : 0U);
  startParamCalibAlign();
}

static void handleUartCommand(uint8_t cmd)
{
  switch (cmd) {
    case 'i':
      bldcAppSetFreewheelMs(60000U);
      app.io_check_request = true;
      break;
    case 'm':
      app.foc_math_request = true;
      break;
    case 'R':
      p("system reset\n");
      HAL_NVIC_SystemReset();
      break;
    case 'u':
      bldcAppSetFreewheelMs(60000U);
      calibrateCurrentOffset();
      break;
    case 'n':
      bldcAppEnableRun();
      p("run enabled, targets remain zero\n");
      break;
    case 'x':
    case '0':
      bldcAppSetFreewheelMs(60000U);
      p("freewheel 60s\n");
      break;
    case '1':
      requestDebugPrint(0U);
      break;
    case '2':
      requestDebugPrint(1U);
      break;
    case '3':
      requestDebugPrint(2U);
      break;
    case 'w':
      setTarget(0, app.motor[0].command_rps + APP_SERIAL_SPEED_STEP_RPS, 0xFFFFU);
      setTarget(1, app.motor[1].command_rps + APP_SERIAL_SPEED_STEP_RPS, 0xFFFFU);
      break;
    case 's':
      setTarget(0, app.motor[0].command_rps - APP_SERIAL_SPEED_STEP_RPS, 0xFFFFU);
      setTarget(1, app.motor[1].command_rps - APP_SERIAL_SPEED_STEP_RPS, 0xFFFFU);
      break;
    case 'q':
      setTarget(0, app.motor[0].command_rps + APP_SERIAL_SPEED_STEP_RPS, 0xFFFFU);
      break;
    case 'a':
      setTarget(0, app.motor[0].command_rps - APP_SERIAL_SPEED_STEP_RPS, 0xFFFFU);
      break;
    case 'e':
      setTarget(1, app.motor[1].command_rps + APP_SERIAL_SPEED_STEP_RPS, 0xFFFFU);
      break;
    case 'd':
      setTarget(1, app.motor[1].command_rps - APP_SERIAL_SPEED_STEP_RPS, 0xFFFFU);
      break;
    case 'y':
      startSensorAlignment(0);
      break;
    case 'h':
      startSensorAlignment(1);
      break;
    case 'v':
      startSensorDiag();
      break;
    case 'o':
      app.open_loop_velocity = true;
      initOpenLoopAnglesFromSensor();
      p("open-loop velocity enabled\n");
      break;
    case 'c':
      app.open_loop_velocity = false;
      p("sensor-angle voltage diagnostic enabled\n");
      break;
    case 'k':
      startParamCalibration(false);
      break;
    case 'K':
      startParamCalibration(true);
      break;
    case ' ':
      setTarget(0, 0.0f, 0U);
      setTarget(1, 0.0f, 0U);
      bldcAppSetFreewheelMs(60000U);
      break;
    case '\n':
      app.print_page++;
      if (app.print_page > 2U) {
        app.print_page = 0;
      }
      requestDebugPrint(app.print_page);
      break;
    default:
      break;
  }
}

static void pollUart(void)
{
  if (app.uart_rx_tail == app.uart_rx_head) {
    return;
  }
  const uint8_t cmd = app.uart_rx_queue[app.uart_rx_tail];
  app.uart_rx_tail = (uint8_t)((app.uart_rx_tail + 1U) % APP_UART_RX_QUEUE_SIZE);
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
        bldcAppSetFreewheelMs((uint32_t)msg->data[2]);
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
  while (app.pwm_irq_count < APP_PWM_ISR_PER_1MS) {
  }
  app.pwm_irq_count = 0;
}

void bldcAppInit(void)
{
  app.mode = BLDC_APP_MODE_BOOT;
  enableCycleCounter();
  focMathInit();
  app.cycle_to_s = 1.0f / (float)SystemCoreClock;
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
  app.open_loop_velocity = true;
  startPwmOutputsFreewheel();

  CAN_Filter_Init((uint16_t)flash.board_id);
  if (HAL_CAN_Start(&hcan) == HAL_OK) {
    app.can_started = true;
  }

  HAL_UART_Receive_IT(&huart1, &app.uart_rx_byte, 1);

  setPwmTimerSyncPhase();
  if (HAL_TIM_Base_Start_IT(&htim1) != HAL_OK) {
    Error_Handler();
  }

  app.mode = BLDC_APP_MODE_READY;
  bldcAppSetFreewheelMs(60000U);
  p("\n[BLDC_APP] boot board 0x%03lx pp %d ol %u zero %+6.3f %+6.3f off %+5.2f %+5.2f v/rps %+6.3f %+6.3f\n",
    flash.board_id,
    APP_POLE_PAIRS,
    app.open_loop_velocity ? 1U : 0U,
    app.motor[0].zero_electric_angle,
    app.motor[1].zero_electric_angle,
    app.motor[0].voltage_offset,
    app.motor[1].voltage_offset,
    app.motor[0].voltage_per_rps,
    app.motor[1].voltage_per_rps);
}

void bldcAppTick1kHz(void)
{
  const uint32_t loop_start = cycleNow();

  pollUart();

  updateSpeed(0);
  updateSpeed(1);
  handleFreewheelTimer();
  updateSensorAlignment();
  updateSensorDiag();
  updateParamCalibration();
  updateOutputVoltage();
  updateOpenLoopVelocity();
  applyProtection();
  updateZeroOutputSleep();
  sendTelemetry();

  app.loop_busy_cycles = cycleDelta(loop_start, cycleNow());
  if (app.loop_busy_cycles > app.loop_busy_cycles_max) {
    app.loop_busy_cycles_max = app.loop_busy_cycles;
  }

  processDeferredDebugOutput();

  setLedRed(true);
  waitForNextMainCycle();
  setLedRed(false);
}

void bldcAppOnTimerElapsed(TIM_HandleTypeDef * htim)
{
  if (htim->Instance != TIM1) {
    return;
  }

  static uint8_t motor_select = 0U;
  const uint32_t isr_start = cycleNow();
  app.pwm_irq_count++;
  motor_select ^= 1U;

  setLedBlue(false);
  applyPwmInIsr(motor_select);
  setLedBlue(true);
  app.isr_cycles = cycleDelta(isr_start, cycleNow());
  if (app.isr_cycles > app.isr_cycles_max) {
    app.isr_cycles_max = app.isr_cycles;
  }
}

void bldcAppSetFreewheelMs(uint32_t ms)
{
  app.align_active = false;
  app.sensor_diag.active = false;
  app.param_calib.active = false;
  app.freewheel_ms = ms;
  app.motor[0].target_rps = 0.0f;
  app.motor[1].target_rps = 0.0f;
  app.motor[0].command_rps = 0.0f;
  app.motor[1].command_rps = 0.0f;
  app.motor[0].command_timeout_ms = 0U;
  app.motor[1].command_timeout_ms = 0U;
  app.motor[0].voltage_q = 0.0f;
  app.motor[1].voltage_q = 0.0f;
  setPwmAll(TIM_PWM_CENTER);
  setPwmOutPutFreeWheel();
  if (app.mode != BLDC_APP_MODE_FAULT) {
    app.mode = BLDC_APP_MODE_FREEWHEEL;
  }
}

void bldcAppEnableRun(void)
{
  if (app.mode == BLDC_APP_MODE_FAULT) {
    p("cannot run: fault 0x%04x info %u value %+6.2f\n", app.fault_id, app.fault_info, app.fault_value);
    return;
  }
  app.freewheel_ms = 0U;
  app.zero_output_ms = 0U;
  initOpenLoopAnglesFromSensor();
  setPwmAll(TIM_PWM_CENTER);
  resumePwmOutput();
  app.mode = BLDC_APP_MODE_RUN;
}

void bldcAppForceFault(uint16_t id, uint16_t info, float value)
{
  if (app.mode == BLDC_APP_MODE_FAULT) {
    return;
  }
  app.fault_id = id;
  app.fault_info = info;
  app.fault_value = value;
  app.mode = BLDC_APP_MODE_FAULT;
  app.align_active = false;
  app.sensor_diag.active = false;
  app.param_calib.active = false;
  setTarget(0, 0.0f, 0U);
  setTarget(1, 0.0f, 0U);
  setPwmAll(TIM_PWM_CENTER);
  setPwmOutPutFreeWheel();
  sendError(id, info, value);
  p("FAULT 0x%04x info %u value %+6.2f\n", id, info, value);
}

uint32_t bldcAppGetCanRxCount(void)
{
  return app.can_rx_count;
}

bldc_app_mode_t bldcAppGetMode(void)
{
  return app.mode;
}

const bldc_app_motor_state_t * bldcAppGetMotorState(uint8_t motor)
{
  if (motor >= APP_MOTOR_COUNT) {
    return 0;
  }
  return &app.motor[motor];
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
{
  if (huart->Instance == USART1) {
    const uint8_t next_head = (uint8_t)((app.uart_rx_head + 1U) % APP_UART_RX_QUEUE_SIZE);
    if (next_head != app.uart_rx_tail) {
      app.uart_rx_queue[app.uart_rx_head] = app.uart_rx_byte;
      app.uart_rx_head = next_head;
    } else {
      app.uart_rx_overrun++;
    }
    HAL_UART_Receive_IT(&huart1, &app.uart_rx_byte, 1);
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

