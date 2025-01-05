#include "motor.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "tim.h"

void speedToOutputVoltage(motor_pid_control_t * pid, motor_real_t * real, motor_param_t * param, motor_control_cmd_t * cmd)
{
  pid->eff_voltage = real->rps * param->voltage_per_rps;
  /*pid[motor].error = cmd[motor].speed - motor_real[motor].rps;

  pid[motor].error_integral += pid[motor].error;
  if (pid[motor].error_integral > pid[motor].error_integral_limit) {
    pid[motor].error_integral = pid[motor].error_integral_limit;
  } else if (pid[motor].error_integral < -pid[motor].error_integral_limit) {
    pid[motor].error_integral = -pid[motor].error_integral_limit;
  }

  pid[motor].error_diff = motor_real[motor].rps - pid[motor].pre_real_rps;
  pid[motor].pre_real_rps = motor_real[motor].rps;*/

  cmd->out_v = cmd->speed * param->voltage_per_rps;

  // ローカルでの速度制御はしない(上位からトルクで制御したいため)
  //        +pid[motor].error_diff * pid[motor].pid_kp + pid[motor].error_integral * pid[motor].pid_ki + pid[motor].error_diff * pid[motor].pid_kd; // PID

  // 出力電圧リミット

  float output_voltage_diff = cmd->out_v - pid->eff_voltage;
  if (output_voltage_diff > +pid->diff_voltage_limit) {
    cmd->out_v = pid->eff_voltage + pid->diff_voltage_limit;
    pid->output_voltage_limitting = true;

  } else if (output_voltage_diff < -pid->diff_voltage_limit) {
    cmd->out_v = pid->eff_voltage - pid->diff_voltage_limit;
    pid->output_voltage_limitting = true;

  } else {
    pid->output_voltage_limitting = false;
  }

  if (pid->output_voltage_limitting) {
    pid->load_limit_cnt++;
  } else if (pid->load_limit_cnt > 0) {
    pid->load_limit_cnt--;
  }
}

void setFinalOutputVoltage(motor_control_cmd_t * cmd, enc_offset_t * enc_offset, float manual_offset)
{
  // +方向にしか増やしてはいけない(最終的には10bitマスクでカバーする)
  const float minus_offset = 2 * M_PI - ROTATION_OFFSET_RADIAN;
  const float plus_offset = ROTATION_OFFSET_RADIAN;
  cmd->out_v_final = cmd->out_v;
  if (cmd->out_v_final < 0) {
    // 2.4
    enc_offset->final = minus_offset + enc_offset->zero_calib + manual_offset;
  } else {
    enc_offset->final = plus_offset + enc_offset->zero_calib + manual_offset;
  }
}

void calcMotorSpeed(motor_real_t * real, as5047p_t * enc, system_t * sys, enc_error_watcher_t * enc_error)
{
  int temp = real->pre_enc_cnt_raw - enc->enc_raw;
  if (temp < -HARF_OF_ENC_CNT_MAX) {
    temp += ENC_CNT_MAX;
  } else if (temp > HARF_OF_ENC_CNT_MAX) {
    temp -= ENC_CNT_MAX;
  }

  if (fabs(real->diff_cnt_max) < fabs(temp)) {
    real->diff_cnt_max = temp;
  }

  // 異常な回転数の場合に無視
  if (fabs((float)temp / ENC_CNT_MAX * 1000) > SPEED_CMD_LIMIT_RPS * 1.5 && sys->free_wheel_cnt == 0) {
    setPwmOutPutFreeWheel();
    sys->free_wheel_cnt += 10;
    enc_error->detect_flag = true;
    enc_error->idx = 0;
    enc_error->cnt = temp;
    return;
  }

  // motor_real[motor].rps = ((float)temp / ENC_CNT_MAX * 1000) * motor_real[motor].k + (1-motor_real[motor].k) * motor_real[motor].pre_rps; // rps
  real->rps = (float)temp / ENC_CNT_MAX * 1000;
  real->rps_ave = real->rps_ave * 0.999 + real->rps * 0.001;
  real->pre_rps = real->rps;
  real->pre_enc_cnt_raw = enc->enc_raw;
}