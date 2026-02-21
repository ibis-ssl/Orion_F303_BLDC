/**
 * @file startup_sequence.c
 * @brief Startup initialization and self-check sequence.
 */

#include "startup_sequence.h"

#include <math.h>

#include "adc.h"
#include "app_context.h"
#include "calibration.h"
#include "can.h"
#include "comms.h"
#include "control_mode.h"
#include "control_limits.h"
#include "flash.h"
#include "gpio.h"
#include "motor.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#define START_UP_FREE_WHEEL_CNT (1000)
#define V_PER_RPS_DEFAULT (0.15)
#define SPEED_REAL_LIMIT_GAIN (float)(1.5)

void waitPowerOnTimeout(void);

void runStartupSequence(void)
{
  initFirstSin();
  clearFaultMode();

  // LED
  setLedRed(true);
  setLedGreen(true);
  setLedBlue(true);
  HAL_Delay(100);

  sys.is_starting_mode = true;

  loadFlashData();
  p("\n\n** Orion VV driver V4 start! %s %s **\n", __DATE__, __TIME__);

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

    // Manual electrical angle offset.
    // Keep zero for normal operation; used only during tuning.
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
      // Lower diff limit for high-kV motors.
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
  // Short delay before sensor register access.
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
    // Zero-offset current sensor model
    // ZXCT1084
    adc_raw.cs_adc_offset = 0;
  } else {
    // Mid-point offset current sensor model
    // INA199
    adc_raw.cs_adc_offset = 2048;
  }

  // M0 : tim1
  // M1 : tim8

  HAL_TIM_PWM_Init(&htim8);
  HAL_TIM_PWM_Init(&htim1);
  setPwmAll(TIM_PWM_CENTER);

  // Turn on each PWM channel pair and perform current safety check.
  // This validates gate output path before normal control loop.
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
    while (interrupt_timer_cnt < 20U * 50U) {
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

  initComms();

  sys.is_starting_mode = false;
}
