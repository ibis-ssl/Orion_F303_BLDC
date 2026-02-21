#ifndef INC_CONTROL_MODE_H_
#define INC_CONTROL_MODE_H_

#include <stdbool.h>

typedef enum
{
  CONTROL_MODE_RUN = 0,
  CONTROL_MODE_ENCODER_CALIB,
  CONTROL_MODE_MOTOR_CALIB
} control_mode_t;

bool isEncoderCalibrationActive(void);
bool isAnyCalibrationActive(void);
control_mode_t getControlMode(void);
void runControlMode(void);

#endif /* INC_CONTROL_MODE_H_ */
