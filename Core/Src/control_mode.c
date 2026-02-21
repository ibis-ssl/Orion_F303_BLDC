/**
 * @file control_mode.c
 * @brief Control mode state judgment and dispatch.
 */

#include "control_mode.h"

#include "app_context.h"

static control_mode_t current_mode = CONTROL_MODE_STARTUP;
static bool fault_mode = false;

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
  if (fault_mode) {
    return CONTROL_MODE_FAULT;
  }
  if (sys.is_starting_mode) {
    return CONTROL_MODE_STARTUP;
  }
  if (isEncoderCalibrationActive()) {
    return CONTROL_MODE_ENCODER_CALIB;
  }
  if (calib_process.motor_calib_cnt != 0U) {
    return CONTROL_MODE_MOTOR_CALIB;
  }
  if (sys.free_wheel_cnt > 0U) {
    return CONTROL_MODE_FREEWHEEL;
  }
  return CONTROL_MODE_RUN;
}

control_mode_t getCurrentControlMode(void)
{
  return current_mode;
}

const char * getControlModeName(control_mode_t mode)
{
  switch (mode) {
    case CONTROL_MODE_STARTUP:
      return "startup";
    case CONTROL_MODE_RUN:
      return "run";
    case CONTROL_MODE_ENCODER_CALIB:
      return "enc_calib";
    case CONTROL_MODE_MOTOR_CALIB:
      return "motor_calib";
    case CONTROL_MODE_FREEWHEEL:
      return "freewheel";
    case CONTROL_MODE_FAULT:
      return "fault";
    default:
      return "unknown";
  }
}

void setFaultMode(void)
{
  fault_mode = true;
}

void clearFaultMode(void)
{
  fault_mode = false;
}

void runControlMode(void)
{
  current_mode = getControlMode();

  switch (current_mode) {
    case CONTROL_MODE_ENCODER_CALIB:
      encoderCalibrationMode();
      break;
    case CONTROL_MODE_MOTOR_CALIB:
      motorCalibrationMode();
      break;
    case CONTROL_MODE_RUN:
    case CONTROL_MODE_FREEWHEEL:
    case CONTROL_MODE_STARTUP:
    case CONTROL_MODE_FAULT:
    default:
      runMode();
      break;
  }
}
