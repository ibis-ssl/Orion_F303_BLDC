/**
 * @file diagnostics.c
 * @brief Runtime telemetry and periodic debug printing.
 */

#include "diagnostics.h"

#include "adc.h"
#include "app_context.h"
#include "can.h"
#include "comms.h"
#include "control_mode.h"
#include "motor.h"
#include "tim.h"
#include "usart.h"

extern uint32_t ex_can_send_fail_cnt;

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
      p("Offset %+4.2f RPS %+6.1f %+6.1f ", sys.manual_offset_radian, motor_real[0].rps_ave, motor_real[1].rps_ave);
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
    default:
      p("\n");
      sys.print_cnt = 0;
      break;
  }
}
