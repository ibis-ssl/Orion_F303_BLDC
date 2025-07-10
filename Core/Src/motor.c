#include "motor.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "tim.h"

void setFinalOutputVoltage(motor_control_cmd_t * cmd, enc_offset_t * enc_offset, float manual_offset)
{
  // +方向にしか増やしてはいけない(最終的には10bitマスクでカバーする)

  cmd->out_v_final = cmd->out_v;
  enc_offset->final = enc_offset->zero_calib + manual_offset;
  // const floatだとここの分岐で計算時間に差が生まれているらしい
  if (cmd->out_v_final < 0) {
    enc_offset->final += MINUS_OFFSET;
  } else {
    enc_offset->final += PLUS_OFFSET;
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

  if (fabsf(real->diff_cnt_max) < fabsf(temp)) {
    real->diff_cnt_max = temp;
  }

  // 異常な回転数の場合に無視
  if (fabsf((float)temp / ENC_CNT_MAX * 1000) > SPEED_CMD_LIMIT_RPS * 1.5 && sys->free_wheel_cnt == 0) {
    setPwmOutPutFreeWheel();
    sys->free_wheel_cnt += 10;
    enc_error->detect_flag = true;
    enc_error->idx = 0;
    enc_error->cnt = temp;
    return;
  }

  // motor_real[motor].rps = ((float)temp / ENC_CNT_MAX * 1000) * motor_real[motor].k + (1-motor_real[motor].k) * motor_real[motor].pre_rps; // rps
  real->rps = (float)temp / ENC_CNT_MAX * 1000;
  real->rps_ave = real->rps_ave * 0.99 + real->rps * 0.01;
  real->pre_rps = real->rps;
  real->pre_enc_cnt_raw = enc->enc_raw;
}