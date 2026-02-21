/**
 * @file control_mode.c
 * @brief Control mode state judgment and dispatch.
 */

#include "control_mode.h"

#include "main.h"

extern calib_process_t calib_process;

void runMode(void);
void encoderCalibrationMode(void);
void motorCalibrationMode(void);

bool isEncoderCalibrationActive(void)
{
  return calib_process.enc_calib_cnt != 0U;
}

bool isAnyCalibrationActive(void)
{
  return (calib_process.enc_calib_cnt != 0U) || (calib_process.motor_calib_cnt != 0U);
}

control_mode_t getControlMode(void)
{
  if (isEncoderCalibrationActive()) {
    return CONTROL_MODE_ENCODER_CALIB;
  }
  if (calib_process.motor_calib_cnt != 0U) {
    return CONTROL_MODE_MOTOR_CALIB;
  }
  return CONTROL_MODE_RUN;
}

void runControlMode(void)
{
  static void (* const mode_handler[])(void) = {
    runMode,
    encoderCalibrationMode,
    motorCalibrationMode,
  };

  control_mode_t mode = getControlMode();
  if ((unsigned)mode < (sizeof(mode_handler) / sizeof(mode_handler[0]))) {
    mode_handler[mode]();
    return;
  }

  runMode();
}
