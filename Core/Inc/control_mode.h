#ifndef INC_CONTROL_MODE_H_
#define INC_CONTROL_MODE_H_

#include <stdbool.h>

typedef enum
{
  CONTROL_MODE_STARTUP = 0,
  CONTROL_MODE_RUN,
  CONTROL_MODE_ENCODER_CALIB,
  CONTROL_MODE_MOTOR_CALIB,
  CONTROL_MODE_FREEWHEEL,
  CONTROL_MODE_FAULT
} control_mode_t;

bool isEncoderCalibrationActive(void);
bool isAnyCalibrationActive(void);
control_mode_t getControlMode(void);
control_mode_t getCurrentControlMode(void);
const char * getControlModeName(control_mode_t mode);
void setFaultMode(void);
void clearFaultMode(void);
void runControlMode(void);

#endif /* INC_CONTROL_MODE_H_ */
