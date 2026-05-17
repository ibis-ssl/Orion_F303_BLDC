/**
 * @file diagnostics.c
 * @brief Runtime telemetry and periodic debug printing.
 */

#include "diagnostics.h"

#include <math.h>

#include "adc.h"
#include "app_context.h"
#include "can.h"
#include "comms.h"
#include "control_mode.h"
#include "flash.h"
#include "foc_driver_hal.h"
#include "foc_math.h"
#include "gpio.h"
#include "motor.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

extern uint32_t ex_can_send_fail_cnt;

#define DIAG_POLE_PAIRS (12)
#define DIAG_SENSOR_DIRECTION (1)
#define DIAG_SENSOR_TORQUE_DIRECTION (-1.0f)
#define DIAG_ENCODER_RAW_TO_MECH_RAD (2.0f * (float)M_PI / (float)ENC_CNT_MAX)
#define DIAG_FOC_SAMPLE_VOLTAGE (2.0f)

typedef struct
{
  int raw;
  float legacy_output_radian;
  float zero_calib;
} foc_diag_snapshot_t;

static void waitPrintDrain(void)
{
  HAL_Delay(3);
}

static bool foc_self_test_ok;
static foc_pwm_compare_t foc_self_test_sample;

void setFocMathSelfTestResult(bool ok, uint16_t a, uint16_t b, uint16_t c, bool limited)
{
  foc_self_test_ok = ok;
  foc_self_test_sample.a = a;
  foc_self_test_sample.b = b;
  foc_self_test_sample.c = c;
  foc_self_test_sample.limited = limited;
}

void runIoCheckOnce(void)
{
  p("\n[IO CHECK] non-rotating check start\n");
  waitPrintDrain();

  cmd[0].speed = 0.0f;
  cmd[1].speed = 0.0f;
  cmd[0].out_v = 0.0f;
  cmd[1].out_v = 0.0f;
  cmd[0].out_v_final = 0.0f;
  cmd[1].out_v_final = 0.0f;
  sys.free_wheel_cnt = 60000U;
  setPwmAll(TIM_PWM_CENTER);
  setPwmOutPutFreeWheel();

  updateADC(0);
  updateADC(1);
  adcUpdateTemperatureFilters();
  updateAS5047P(0);
  updateAS5047P(1);

  p("SW 1:%d 2:%d 3:%d 4:%d\n", isPushedSW1(), isPushedSW2(), isPushedSW3(), isPushedSW4());
  waitPrintDrain();

  p("ADC raw CS %4d %4d Batt %4d GD %4d FET %4d %4d MotorT %4d %4d Off %4d\n", adc_raw.cs_motor[0], adc_raw.cs_motor[1], adc_raw.batt_v, adc_raw.gd_dcdc_v, adc_raw.temp_fet[0], adc_raw.temp_fet[1],
    adc_raw.temp_motor[0], adc_raw.temp_motor[1], adc_raw.cs_adc_offset);
  p("ADC val CS %+6.3f %+6.3f Batt %5.2f GD %5.2f FET %3d %3d MotorT %3d %3d\n", getCurrentMotor(0), getCurrentMotor(1), getBatteryVoltage(), getGateDriverDCDCVoltage(), getTempFET(0), getTempFET(1),
    getTempMotor(0), getTempMotor(1));
  waitPrintDrain();

  for (int i = 0; i < 2; i++) {
    updateAS5047PDiagnostics(i);
    p("ENC M%d raw %5d elec %5d rad %+6.3f diff %+6d min %+6d max %+6d frame 0x%04x spierr %lu\n", i, as5047p[i].enc_raw, as5047p[i].enc_elec_raw, as5047p[i].output_radian, as5047p[i].diff_enc,
      as5047p[i].diff_min, as5047p[i].diff_max, as5047p[i].last_frame, as5047p[i].spi_error_count);
    p("ENC M%d reg err 0x%02x prog 0x%02x diag 0x%03x mag 0x%03x enc 0x%03x com 0x%03x\n", i, as5047p[i].reg.error, as5047p[i].reg.prog, as5047p[i].reg.diagagc, as5047p[i].reg.mag,
      as5047p[i].reg.angleenc, as5047p[i].reg.anglecom);
    waitPrintDrain();
  }

  p("PWM TIM1 CCR %4ld %4ld %4ld CCER 0x%04lx BDTR 0x%04lx\n", htim1.Instance->CCR1, htim1.Instance->CCR2, htim1.Instance->CCR3, htim1.Instance->CCER, htim1.Instance->BDTR);
  p("PWM TIM8 CCR %4ld %4ld %4ld CCER 0x%04lx BDTR 0x%04lx\n", htim8.Instance->CCR1, htim8.Instance->CCR2, htim8.Instance->CCR3, htim8.Instance->CCER, htim8.Instance->BDTR);
  waitPrintDrain();

  p("CAN rx %lu err 0x%08lx board 0x%03lx flash calib %+6.3f %+6.3f rps/v %+6.3f %+6.3f\n", getCanRxCount(), getCanError(), flash.board_id, flash.calib[0], flash.calib[1], flash.rps_per_v_cw[0],
    flash.rps_per_v_cw[1]);
  p("[IO CHECK] done, PWM remains freewheel for 60s or until run command\n\n");
  waitPrintDrain();
}

void runFocMathCheckOnce(void)
{
  foc_diag_snapshot_t snapshot[2];

  __disable_irq();
  for (uint8_t i = 0U; i < 2U; i++) {
    snapshot[i].raw = as5047p[i].enc_raw;
    snapshot[i].legacy_output_radian = as5047p[i].output_radian;
    snapshot[i].zero_calib = enc_offset[i].zero_calib;
  }
  __enable_irq();

  p("\n[FOC CHECK] math %s sample %4u %4u %4u limited %u\n", foc_self_test_ok ? "OK" : "NG", foc_self_test_sample.a, foc_self_test_sample.b, foc_self_test_sample.c,
    foc_self_test_sample.limited ? 1U : 0U);

  for (uint8_t i = 0U; i < 2U; i++) {
    const float mech = (float)snapshot[i].raw * DIAG_ENCODER_RAW_TO_MECH_RAD;
    const float foc_raw_el_pos = focElectricalAngle(mech, DIAG_POLE_PAIRS, 0.0f, +1);
    const float foc_raw_el_neg = focElectricalAngle(mech, DIAG_POLE_PAIRS, 0.0f, -1);
    const float legacy_el = snapshot[i].legacy_output_radian + snapshot[i].zero_calib;
    const float uq_pos = DIAG_SENSOR_TORQUE_DIRECTION * DIAG_FOC_SAMPLE_VOLTAGE;
    const float uq_neg = -uq_pos;
    const foc_pwm_compare_t pos = focDriverBuildSinePwm(uq_pos, 0.0f, legacy_el, getBatteryVoltage());
    const foc_pwm_compare_t neg = focDriverBuildSinePwm(uq_neg, 0.0f, legacy_el, getBatteryVoltage());

    p("[FOC CHECK] M%u raw %5d legacy %+6.3f raw+ %+6.3f raw- %+6.3f uq+ %4u %4u %4u uq- %4u %4u %4u\n", i, snapshot[i].raw, legacy_el, foc_raw_el_pos, foc_raw_el_neg, pos.a, pos.b, pos.c, neg.a,
      neg.b, neg.c);
  }

  p("[FOC CHECK] snapshot only, no SPI update, no PWM output changed\n\n");
}

void printRuntimeDiagnostics(void)
{
  static float main_loop_remain_counter_ave = 0.0f;
  static float task_complete_timer_cnt_ave = 0.0f;
  static float system_exec_time_stamp_ave[10] = {0};

  // Spread debug output over multiple cycles.
  // Full set is printed over several control ticks.
  sys.print_cnt++;

  switch (sys.print_cnt) {
    case 1:
      // p("M0raw %6d M1raw %6d ", as5047p[0].enc_raw, as5047p[1].enc_raw);
      p("\e[0mCS %+5.2f %+5.2f / BV %4.1f ", getCurrentMotor(0), getCurrentMotor(1), getBatteryVoltage());
      // p("P %+3.1f I %+3.1f D %+3.1f ", pid[0].pid_kp, pid[0].pid_ki, pid[0].pid_kd);
      break;
    case 2:
      p("RPS %+6.1f %+6.1f Free %4d ", motor_real[0].rps, motor_real[1].rps, sys.free_wheel_cnt);
      break;
    case 3:
      p("RAW %5d %5d Out_v %+5.1f %+5.1f ", as5047p[0].enc_raw, as5047p[1].enc_raw, cmd[0].out_v, cmd[1].out_v);
      break;
    case 4:
      //p("p%+3.1f i%+3.1f d%+3.1f k%+3.1f ", pid[0].pid_kp, pid[0].pid_ki, pid[0].pid_kd, motor_real[0].k);
      p("Rx %4ld CPU %3ld GD %4.1f Mode %s ", getCanRxCount(), main_loop_remain_counter, getGateDriverDCDCVoltage(), getControlModeName(getCurrentControlMode()));
      clearCanRxCount();
      break;
    case 5:
      if (sys.print_idx == 0) {
        p("Eff %+6.2f %+6.2f %d %d ", pid[0].eff_voltage, pid[1].eff_voltage, pid[0].output_voltage_limitting, pid[1].output_voltage_limitting);
      } else if (sys.print_idx == 1) {
        // FET temperature support depends on board revision.
        p("FET-T %+4d %+4d Motor-T %+4d %+4d", getTempFET(0), getTempFET(1), getTempMotor(0), getTempMotor(1));
      } else {
        p("SPD %+6.1f %+6.1f canErr 0x%04x ", cmd[0].speed, cmd[1].speed, getCanError());
      }
      break;
    case 6:
      task_complete_timer_cnt_ave = task_complete_timer_cnt_ave * 0.99f + (float)sys.task_complete_timer_cnt * 0.01f;
      main_loop_remain_counter_ave = main_loop_remain_counter_ave * 0.99f + (float)main_loop_remain_counter * 0.01f;
      p("CPUA %6.4f CNT %6.2f ", main_loop_remain_counter_ave, task_complete_timer_cnt_ave);
      //p("LoadV %+5.2f %+5.2f CanFail %4d ", cmd[0].out_v - pid[0].eff_voltage, cmd[1].out_v - pid[1].eff_voltage, ex_can_send_fail_cnt);
      ex_can_send_fail_cnt = 0;
      break;
    case 7:
      p("Offset %+4.2f TrqOff %+4.2f %+4.2f RPS %+6.1f %+6.1f ", sys.manual_offset_radian, getLegacyTorqueOffsetRadian(cmd[0].out_v_final), getLegacyTorqueOffsetRadian(cmd[1].out_v_final),
        motor_real[0].rps_ave, motor_real[1].rps_ave);
      //p("LoadCnt %4.3f %4.3f ", (float)pid[0].load_limit_cnt / MOTOR_OVER_LOAD_CNT_LIMIT, (float)pid[1].load_limit_cnt / MOTOR_OVER_LOAD_CNT_LIMIT);
      break;
    case 8:
      for (int i = 0; i < 4; i++) {
        system_exec_time_stamp_ave[i] = system_exec_time_stamp_ave[i] * 0.99f + (float)system_exec_time_stamp[i] * 0.01f;
      }
      p("Ave %6.4f %6.4f %6.4f %6.4f ", system_exec_time_stamp_ave[0], system_exec_time_stamp_ave[1], system_exec_time_stamp_ave[2], system_exec_time_stamp_ave[3]);
      //p("TO %4d %4d diff max M0 %+6d, M1 %+6d %d", cmd[0].timeout_cnt, cmd[1].timeout_cnt, motor_real[0].diff_cnt_max, motor_real[1].diff_cnt_max, enc_error_watcher.detect_flag);
      // p("min %+6d cnt %6d / max %+6d cnt %6d ", as5047p[0].diff_min, as5047p[0].diff_min_cnt, as5047p[0].diff_max, as5047p[0].diff_max_cnt);
      motor_real[0].diff_cnt_max = 0;
      motor_real[1].diff_cnt_max = 0;
      as5047p[0].diff_max = 0;
      as5047p[0].diff_min = 65535;
      as5047p[1].diff_max = 0;
      as5047p[1].diff_min = 65535;
      break;
    case 9:
      p("\n");
      break;
    case 100:
      sys.print_cnt = 0;
      break;
    default:
      break;
  }
}
