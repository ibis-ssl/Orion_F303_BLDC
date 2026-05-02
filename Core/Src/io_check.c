/**
 * @file io_check.c
 * @brief Non-rotating input/output inspection helpers for bench checks before motor drive.
 */

#include "io_check.h"

#include "adc.h"
#include "app_context.h"
#include "can.h"
#include "comms.h"
#include "flash.h"
#include "foc_driver_hal.h"
#include "foc_math.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

static void printSwitchState(void)
{
  p("SW 1:%d 2:%d 3:%d 4:%d\n", isPushedSW1(), isPushedSW2(), isPushedSW3(), isPushedSW4());
}

static void printAdcState(void)
{
  p("ADC raw CS %4d %4d Batt %4d GD %4d FET %4d %4d MotorT %4d %4d Off %4d\n",
    adc_raw.cs_motor[0],
    adc_raw.cs_motor[1],
    adc_raw.batt_v,
    adc_raw.gd_dcdc_v,
    adc_raw.temp_fet[0],
    adc_raw.temp_fet[1],
    adc_raw.temp_motor[0],
    adc_raw.temp_motor[1],
    adc_raw.cs_adc_offset);
  p("ADC val CS %+6.3f %+6.3f Batt %5.2f GD %5.2f FET %3d %3d MotorT %3d %3d\n",
    getCurrentMotor(0),
    getCurrentMotor(1),
    getBatteryVoltage(),
    getGateDriverDCDCVoltage(),
    getTempFET(0),
    getTempFET(1),
    getTempMotor(0),
    getTempMotor(1));
}

static void printEncoderState(void)
{
  for (int i = 0; i < 2; i++) {
    p("ENC M%d raw %5d elec %5d rad %+6.3f diff %+6d min %+6d max %+6d\n",
      i,
      as5047p[i].enc_raw,
      as5047p[i].enc_elec_raw,
      as5047p[i].output_radian,
      as5047p[i].diff_enc,
      as5047p[i].diff_min,
      as5047p[i].diff_max);
    p("ENC M%d reg err 0x%02x prog 0x%02x diag 0x%03x mag 0x%03x enc 0x%03x com 0x%03x\n",
      i,
      as5047p[i].reg.error,
      as5047p[i].reg.prog,
      as5047p[i].reg.diagagc,
      as5047p[i].reg.mag,
      as5047p[i].reg.angleenc,
      as5047p[i].reg.anglecom);
  }
}

void runFocMathCheckOnce(void)
{
  foc_pwm_compare_t sample = {0};
  const bool ok = focRunMathSelfTest(&sample);
  p("FOC math self-test %s sample CCR %4d %4d %4d limited %d\n", ok ? "OK" : "NG", sample.a, sample.b, sample.c, sample.limited);
}

void runIoCheckOnce(void)
{
  p("\n[IO CHECK] non-rotating check start\n");

  setPwmAll(TIM_PWM_CENTER);
  setPwmOutPutFreeWheel();
  cmd[0].speed = 0.0f;
  cmd[1].speed = 0.0f;
  cmd[0].out_v = 0.0f;
  cmd[1].out_v = 0.0f;
  cmd[0].out_v_final = 0.0f;
  cmd[1].out_v_final = 0.0f;
  sys.free_wheel_cnt = 100U;

  updateADC(0);
  updateADC(1);
  updateAS5047P(0);
  updateAS5047P(1);

  printSwitchState();
  printAdcState();
  printEncoderState();
  focDriverPrintState();
  p("CAN rx %lu err 0x%08lx board 0x%03lx flash calib %+6.3f %+6.3f rps/v %+6.3f %+6.3f\n",
    getCanRxCount(),
    getCanError(),
    flash.board_id,
    flash.calib[0],
    flash.calib[1],
    flash.rps_per_v_cw[0],
    flash.rps_per_v_cw[1]);
  runFocMathCheckOnce();

  setLedRed(false);
  setLedGreen(false);
  setLedBlue(false);
  p("[IO CHECK] done, PWM remains freewheel\n\n");
}
