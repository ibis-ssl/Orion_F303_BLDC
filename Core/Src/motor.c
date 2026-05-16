/*
 * motor.c
 *
 * Builds output voltage and encoder electrical-angle offsets from speed commands,
 * and updates encoder-based speed estimates. PWM generation is handled by tim.c.
 */
#include "motor.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "tim.h"

static inline void updateEffectiveVoltage(motor_pid_control_t * pid, const motor_real_t * real, const motor_param_t * param)
{
  pid->eff_voltage = real->rps * param->voltage_per_rps;
}

static inline void updateCommandVoltageFromSpeed(const motor_param_t * param, motor_control_cmd_t * cmd)
{
  cmd->out_v = cmd->speed * param->voltage_per_rps;
}

static inline void limitOutputVoltageDiff(motor_pid_control_t * pid, motor_control_cmd_t * cmd)
{
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
}

static inline void updateLoadLimitCounter(motor_pid_control_t * pid)
{
  if (pid->output_voltage_limitting) {
    pid->load_limit_cnt++;
  } else if (pid->load_limit_cnt > 0) {
    pid->load_limit_cnt--;
  }
}

void speedToOutputVoltage(motor_pid_control_t * pid, motor_real_t * real, motor_param_t * param, motor_control_cmd_t * cmd)
{
  updateEffectiveVoltage(pid, real, param);

  /*
   * Legacy control does not use the local PID terms here. It limits the voltage
   * command by the difference from the effective voltage estimated from speed.
   */
  updateCommandVoltageFromSpeed(param, cmd);
  limitOutputVoltageDiff(pid, cmd);
  updateLoadLimitCounter(pid);
}

// Legacy control represents voltage sign by selecting an electrical-angle offset.
// This offset was tuned experimentally and compensates more than ideal q-axis phase.
#define LEGACY_MINUS_TORQUE_OFFSET_RADIAN (2 * M_PI - ROTATION_OFFSET_RADIAN)
#define LEGACY_PLUS_TORQUE_OFFSET_RADIAN (ROTATION_OFFSET_RADIAN)

float getLegacyTorqueOffsetRadian(float output_voltage)
{
  if (output_voltage < 0.0f) {
    return LEGACY_MINUS_TORQUE_OFFSET_RADIAN;
  }
  return LEGACY_PLUS_TORQUE_OFFSET_RADIAN;
}

static inline float buildLegacyEncoderOffset(const enc_offset_t * enc_offset, float manual_offset, float torque_offset)
{
  return enc_offset->zero_calib + manual_offset + torque_offset;
}

void setFinalOutputVoltage(motor_control_cmd_t * cmd, enc_offset_t * enc_offset, float manual_offset)
{
  cmd->out_v_final = cmd->out_v;
  enc_offset->final = buildLegacyEncoderOffset(enc_offset, manual_offset, getLegacyTorqueOffsetRadian(cmd->out_v_final));
}

static inline int calcEncoderRawDiffLegacy(int pre_raw, int current_raw)
{
  int diff_cnt = pre_raw - current_raw;
  if (diff_cnt < -HARF_OF_ENC_CNT_MAX) {
    diff_cnt += ENC_CNT_MAX;
  } else if (diff_cnt > HARF_OF_ENC_CNT_MAX) {
    diff_cnt -= ENC_CNT_MAX;
  }
  return diff_cnt;
}

static inline void updatePeakEncoderDiff(motor_real_t * real, int diff_cnt)
{
  if (fabsf(real->diff_cnt_max) < fabsf(diff_cnt)) {
    real->diff_cnt_max = diff_cnt;
  }
}

static inline float encoderDiffToRps(int diff_cnt)
{
  return (float)diff_cnt / ENC_CNT_MAX * 1000;
}

static inline bool isEncoderSpeedJump(float rps, const system_t * sys)
{
  return fabsf(rps) > SPEED_CMD_LIMIT_RPS * 2.0f && sys->free_wheel_cnt == 0;
}

static inline void recordEncoderSpeedError(system_t * sys, enc_error_watcher_t * enc_error, int diff_cnt)
{
  setPwmOutPutFreeWheel();
  sys->free_wheel_cnt += 10;
  enc_error->detect_flag = true;
  enc_error->idx = 0;
  enc_error->cnt = diff_cnt;
}

static inline void updateMotorSpeedValue(motor_real_t * real, const as5047p_t * enc, float rps)
{
  real->rps = rps;
  real->rps_ave = real->rps_ave * 0.99f + real->rps * 0.01f;
  real->pre_rps = real->rps;
  real->pre_enc_cnt_raw = enc->enc_raw;
}

int calcMotorSpeed(motor_real_t * real, as5047p_t * enc, system_t * sys, enc_error_watcher_t * enc_error)
{
  int diff_cnt = calcEncoderRawDiffLegacy(real->pre_enc_cnt_raw, enc->enc_raw);
  float rps = encoderDiffToRps(diff_cnt);

  updatePeakEncoderDiff(real, diff_cnt);

  if (isEncoderSpeedJump(rps, sys)) {
    recordEncoderSpeedError(sys, enc_error, diff_cnt);
    return -1;
  }

  updateMotorSpeedValue(real, enc, rps);
  return 0;
}
