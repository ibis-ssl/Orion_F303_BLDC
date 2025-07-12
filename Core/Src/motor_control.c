#include "motor_control.h"

#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

void motor_control_init()
{
  p("init motor control\n");
}

void motor_control_cycle()
{
  updateAS5047P(1);
  updateAS5047P(0);

  float motor_duty[2] = {0}, motor_voltage[2] = {0};
  static float battery_voltage = 24;
  battery_voltage = battery_voltage * 0.99 + (adc_raw.batt_v * 3.3 * 11 / 4096) * 0.01;

  for (int i = 0; i < 2; i++) {
    motor_duty[i] = (((float)(as5047p[i].enc_raw) / ENC_CNT_MAX) - 0.5) * 2.0;
    motor_voltage[i] = motor_duty[i] * 22;
  }
  p("%6d %4.2f ENC0:%6d ENC1:%6d -> voltage %+5.3f\n", adc_raw.batt_v, battery_voltage, as5047p[0].enc_raw, as5047p[1].enc_raw, motor_voltage[0]);

  pwm_set_dc_motor_voltage0(motor_voltage[0], battery_voltage);
  pwm_set_dc_motor_voltage1(motor_voltage[0], battery_voltage);
}