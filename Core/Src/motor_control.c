#include "motor_control.h"

#include "adc.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

void motor_control_init()
{
  p("init motor control\n");
}

typedef struct
{
  float angle;
} control_target_t;

typedef struct
{
  float swerve_angle;
  float motor_angle;
  float pre_motor_angle;
  float motor_speed;
  float motor_diff;
  float motor_diff_integral;
} enc_status_t;

typedef struct
{
  float kp, kd, ki;
  float ki_out_limit;
} pid_t;

control_target_t target = {.angle = 0.5};
pid_t pid = {.ki_out_limit = 1.0, .ki = 0.0, .kd = 40.0, .kp = 20.0}, pid_out;
enc_status_t enc;

void motor_control_cycle(bool enable)
{
  as5047p_update(1);
  as5047p_update(0);

  static float battery_voltage = 24;
  battery_voltage = battery_voltage * 0.99 + adc_get_battery_voltage() * 0.01;

  // ステア
  int swerve_cnt_per_cycle = as5047p[1].pre_enc_raw - as5047p[1].enc_raw;
  if (swerve_cnt_per_cycle > (ENC_CNT_MAX / 2)) {
    swerve_cnt_per_cycle -= ENC_CNT_MAX;
  } else if (swerve_cnt_per_cycle < -(ENC_CNT_MAX / 2)) {
    swerve_cnt_per_cycle += ENC_CNT_MAX;
  }

  // モーター
  int motor_cnt_per_cycle = as5047p[0].pre_enc_raw - as5047p[0].enc_raw;
  if (motor_cnt_per_cycle > (ENC_CNT_MAX / 2)) {
    motor_cnt_per_cycle -= ENC_CNT_MAX;
  } else if (motor_cnt_per_cycle < -(ENC_CNT_MAX / 2)) {
    motor_cnt_per_cycle += ENC_CNT_MAX;
  }

  static int motor_angle_raw_cnt = 0;
  static int swerve_angle_raw_cnt = 0;
  motor_angle_raw_cnt += motor_cnt_per_cycle;
  swerve_angle_raw_cnt += swerve_cnt_per_cycle;

  if (sw_is_pushed_4()) {
    motor_angle_raw_cnt = 0;
    swerve_angle_raw_cnt = 0;
  }

  if (sw_is_pushed_2()) {
    target.angle = 0.125;
  }
  if (sw_is_pushed_3()) {
    target.angle = -0.125;
  }
  float motor_base_swerve_angle = target.angle * 36 / 16;

  enc.motor_angle = ((float)(motor_angle_raw_cnt) / ENC_CNT_MAX) - 0.5;
  enc.swerve_angle = ((float)(swerve_angle_raw_cnt) / ENC_CNT_MAX) - 0.5;

  enc.motor_diff = motor_base_swerve_angle - enc.motor_angle;
  enc.motor_speed = enc.pre_motor_angle - enc.motor_angle;  //微分先行
  enc.pre_motor_angle = enc.motor_angle;

  enc.motor_diff_integral += enc.motor_diff;
  if (enc.motor_diff_integral * pid.ki > pid.ki_out_limit) {
    enc.motor_diff_integral = pid.ki_out_limit;
  } else if (enc.motor_diff_integral * pid.ki < -pid.ki_out_limit) {
    enc.motor_diff_integral = -pid.ki_out_limit;
  }

  pid_out.kd = pid.kd * enc.motor_speed;
  pid_out.ki = pid.ki * enc.motor_diff_integral;
  pid_out.kp = pid.kp * enc.motor_diff;

  float output_voltage = (pid_out.kd + pid_out.ki + pid_out.kp) * 20;

  if (output_voltage > 20) {
    output_voltage = 20;
  } else if (output_voltage < -20) {
    output_voltage = -20;
  }
  if (enable) {
    pwm_set_dc_motor_voltage0(output_voltage, battery_voltage);
    pwm_set_dc_motor_voltage1(output_voltage, battery_voltage);
  } else {
    pwm_set_dc_motor_voltage0(0, battery_voltage);
    pwm_set_dc_motor_voltage1(0, battery_voltage);
  }
  static int cnt = 0;
  cnt++;
  if (cnt > 10) {
    cnt = 0;
    p("M %+6d S %+6d ", motor_angle_raw_cnt, swerve_angle_raw_cnt);
    p("%4.2f ENC0:%6d ENC1:%6d -> %d voltage %+5.3f OUT P %+5.2f  I %+5.2f D %+5.2f\n", battery_voltage, as5047p[0].enc_raw, as5047p[1].enc_raw, enable, output_voltage, pid_out.kp, pid_out.ki,
      pid_out.kd);
  }
}