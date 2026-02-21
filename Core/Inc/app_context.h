#ifndef INC_APP_CONTEXT_H_
#define INC_APP_CONTEXT_H_

#include "main.h"

extern motor_control_cmd_t cmd[2];
extern calib_point_t calib[2];
extern enc_offset_t enc_offset[2];
extern motor_real_t motor_real[2];
extern motor_param_t motor_param[2];
extern motor_pid_control_t pid[2];

extern error_t error;
extern system_t sys;
extern enc_error_watcher_t enc_error_watcher;
extern calib_process_t calib_process;

extern volatile uint32_t interrupt_timer_cnt;
extern volatile uint32_t main_loop_remain_counter;
extern volatile uint32_t system_exec_time_stamp[10];

#endif /* INC_APP_CONTEXT_H_ */
