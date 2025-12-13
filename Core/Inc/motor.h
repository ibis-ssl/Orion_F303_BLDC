/*
 * motor.h
 *
 *  Created on: Apr 17, 2024
 *      Author: hiroyuki
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_
#include "main.h"
#include "spi.h"

void speedToOutputVoltage(motor_pid_control_t * pid, motor_real_t * real, motor_param_t * param, motor_control_cmd_t * cmd);
void setFinalOutputVoltage(motor_control_cmd_t * cmd, enc_offset_t * enc_offset, float manual_offset);
int calcMotorSpeed(motor_real_t * real, as5047p_t * enc, system_t * sys, enc_error_watcher_t * enc_error);

#endif /* INC_MOTOR_H_ */
