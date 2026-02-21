/**
 * @file protect.c
 * @brief Protection logic and fault shutdown handling.
 */

#include "protect.h"

#include "adc.h"
#include "app_context.h"
#include "control_limits.h"
#include "control_mode.h"
#include "flash.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

void waitPowerOnTimeout(void);

static inline void protectOverCurrent(void)
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
      setFaultMode();
      waitPowerOnTimeout();
    }
  }
}

static inline void protectBatteryVoltage(void)
{
  if (getBatteryVoltage() < THR_BATTERY_UNVER_VOLTAGE) {
    forceStopAllPwmOutputAndTimer();
    p("UNDER voltage!! %6.3f", getBatteryVoltage());
    setLedBlue(true);
    setLedGreen(false);
    setLedRed(true);

    error.id = flash.board_id * 2;
    error.info = BLDC_UNDER_VOLTAGE;
    error.value = getBatteryVoltage();
    setFaultMode();
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
    setFaultMode();
    waitPowerOnTimeout();
  }
}

static inline void protectMotorTemperature(void)
{
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
    setFaultMode();
    waitPowerOnTimeout();
  }
}

static inline void protectFetTemperature(void)
{
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
    setFaultMode();
    waitPowerOnTimeout();
  }
}

static inline void protectOverLoad(void)
{
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

    setFaultMode();
    waitPowerOnTimeout();
  }
}

void protect(void)
{
  protectOverCurrent();

  // Encoder error hook is disabled for now.
  (void)enc_error_watcher;
  (void)motor_real;

  protectBatteryVoltage();
  protectMotorTemperature();
  protectFetTemperature();
  protectOverLoad();
}
