/**
 * @file calibration.c
 * @brief Encoder and motor calibration flow management.
 */

#include "calibration.h"

#include <math.h>

#include "adc.h"
#include "app_context.h"
#include "flash.h"
#include "motor.h"
#include "tim.h"
#include "usart.h"

#define MOTOR_CALIB_INIT_CNT (2500)
#define MOTOR_CALIB_READY_CNT (2000)
#define MOTOR_CALIB_START_CNT (1500)
#define MOTOR_CALIB_VOLTAGE_LOW (3.0)
#define MOTOR_CALIB_VOLTAGE_HIGH (5.0)
#define MOTOR_CALIB_CYCLE (12)

#define MOTOR_CALIB_M0_M1_ERROR_TRERANCE (1.0)
#define MOTOR_CALIB_CW_CCW_ERROR_TRERANCE (1.0)
#define MOTOR_CALIB_UNDER_LIMIT (1.0)

static void checkAngleCalibMode(bool motor)
{
  calib[motor].xy_field.radian_ave_x += cos(as5047p[motor].output_radian);
  calib[motor].xy_field.radian_ave_y += sin(as5047p[motor].output_radian);
  calib[motor].ave_cnt++;
  if (calib[motor].pre_raw > HARF_OF_ENC_CNT_MAX && as5047p[motor].enc_raw < HARF_OF_ENC_CNT_MAX && calib_process.force_rotation_speed > 0) {
    // ccw
    calib_process.print_flag = true;
    calib[motor].result_ccw_cnt++;
    calib[motor].xy_field.result_cw_x = calib[motor].xy_field.radian_ave_x / calib[motor].ave_cnt;
    calib[motor].xy_field.result_cw_y = calib[motor].xy_field.radian_ave_y / calib[motor].ave_cnt;
    calib[motor].xy_field.radian_ave_x = 0;
    calib[motor].xy_field.radian_ave_y = 0;
    calib[motor].ave_cnt = 0;
  }
  if (calib[motor].pre_raw < HARF_OF_ENC_CNT_MAX && as5047p[motor].enc_raw > HARF_OF_ENC_CNT_MAX && calib_process.force_rotation_speed < 0) {
    // cw
    calib_process.print_flag = true;
    calib[motor].result_cw_cnt++;
    calib[motor].xy_field.result_ccw_x = calib[motor].xy_field.radian_ave_x / calib[motor].ave_cnt;
    calib[motor].xy_field.result_ccw_y = calib[motor].xy_field.radian_ave_y / calib[motor].ave_cnt;
    calib[motor].xy_field.radian_ave_x = 0;
    calib[motor].xy_field.radian_ave_y = 0;
    calib[motor].ave_cnt = 0;
  }
  calib[motor].pre_raw = as5047p[motor].enc_raw;
}

void calibrationProcess_itr(bool motor)
{
  sys.manual_offset_radian += calib_process.force_rotation_speed;

  // Advance virtual angle and fold it into [0, 2pi).
  if (sys.manual_offset_radian > M_PI * 2) {
    sys.manual_offset_radian -= M_PI * 2;
    checkAngleCalibMode(!motor);
  }
  if (sys.manual_offset_radian < 0) {
    sys.manual_offset_radian += M_PI * 2;
    checkAngleCalibMode(!motor);
  }

  updateADC(motor);
  updateAS5047P(motor);
  setOutputRadianMotor(motor, sys.manual_offset_radian, cmd[motor].out_v_final, getBatteryVoltage(), MOTOR_CALIB_VOLTAGE_HIGH);
}

void encoderCalibrationMode(void)
{
  // Print once per completed electrical rotation sample.
  if (calib_process.print_flag) {
    calib_process.print_flag = false;
    p("enc = %+5.2f %+5.2f  / M0 X %+5.2f Y %+5.2f / M1 X %+5.2f Y %+5.2f / Rad %+5.2f %+5.2f\n", as5047p[0].output_radian, as5047p[1].output_radian, cos(as5047p[0].output_radian),
      sin(as5047p[0].output_radian), cos(as5047p[1].output_radian), sin(as5047p[1].output_radian), atan2(sin(as5047p[0].output_radian), cos(as5047p[0].output_radian)),
      atan2(sin(as5047p[1].output_radian), cos(as5047p[1].output_radian)));
  }

  // Start with CCW rotation, then switch to CW.
  // Capture both directions to average encoder asymmetry.
  // Results are finalized after both directions complete.

  // END of 1st-calibration cycle (CCW)
  if (calib[0].result_ccw_cnt > MOTOR_CALIB_CYCLE && calib[1].result_ccw_cnt > MOTOR_CALIB_CYCLE && calib_process.force_rotation_speed > 0) {
    calib_process.force_rotation_speed = -calib_process.force_rotation_speed;  // Switch from CCW to CW
    p("END of 1st-calibration cycle (CCW)\n");
    HAL_Delay(1);  // write out uart buffer
  }

  // END of 2nd-calibration cycle (CW)
  if (calib[0].result_cw_cnt > MOTOR_CALIB_CYCLE && calib[1].result_cw_cnt > MOTOR_CALIB_CYCLE) {
    // Stop forced rotation before applying calculated offsets.
    cmd[0].out_v_final = 0;
    cmd[1].out_v_final = 0;
    p("END of 2nd-calibration cycle (CW)\n");
    HAL_Delay(1);  // write out uart buffer

    float xy_field_ave_x[2] = {0}, xy_field_ave_y[2] = {0}, xy_field_offset_radian[2] = {0, 0};

    for (int i = 0; i < 2; i++) {
      p("Motor : %d\n", i);
      p("CW X %+5.2f Y %+5.2f\n", calib[i].xy_field.result_cw_x, calib[i].xy_field.result_cw_y);
      p("CCW X %+5.2f Y %+5.2f\n", calib[i].xy_field.result_ccw_x, calib[i].xy_field.result_ccw_y);
      p("CW rad %+5.2f\n", atan2(calib[i].xy_field.result_cw_y, calib[i].xy_field.result_cw_x));
      p("CCW rad %+5.2f\n", atan2(calib[i].xy_field.result_ccw_y, calib[i].xy_field.result_ccw_x));

      xy_field_ave_x[i] = calib[i].xy_field.result_cw_x + calib[i].xy_field.result_ccw_x;
      xy_field_ave_y[i] = calib[i].xy_field.result_cw_y + calib[i].xy_field.result_ccw_y;
      xy_field_offset_radian[i] = (2 * M_PI) - atan2(xy_field_ave_y[i], xy_field_ave_x[i]);
      p("CW+CCW X %+5.2f Y %+5.2f\n", xy_field_ave_x[i], xy_field_ave_y[i]);

      /*       xy_field_offset_radian[i] += ROTATION_OFFSET_RADIAN;
      if (xy_field_offset_radian[i] > M_PI * 2) {
        xy_field_offset_radian[i] -= M_PI * 2;
      }
      if (xy_field_offset_radian[i] < 0) {
        xy_field_offset_radian[i] += M_PI * 2;
      } */

      p("Rad M0 %+5.2f\n\n", xy_field_offset_radian[i]);

      // Store calculated encoder zero offset.
      enc_offset[i].zero_calib = xy_field_offset_radian[i];

      // Keep runtime/flash values synchronized during calibration.
      flash.calib[i] = enc_offset[i].zero_calib;

      calib[i].result_cw_cnt = 0;
      calib[i].ave_cnt = 0;
      cmd[i].out_v_final = 0;
    }

    writeEncCalibrationValue(enc_offset[0].zero_calib, enc_offset[1].zero_calib);

    for (int i = 0; i < 2; i++) {
      // Store calculated encoder zero offset.
      enc_offset[i].zero_calib = xy_field_offset_radian[i];

      // Keep runtime/flash values synchronized during calibration.
      flash.calib[i] = xy_field_offset_radian[i];
    }

    // End encoder calibration mode.
    calib_process.enc_calib_cnt = 0;
    sys.manual_offset_radian = 0;  // Reset manual offset for normal control.

    // Move to motor calibration step.
    calib_process.motor_calib_mode = MOTOR_CALIB_STAGE_INIT;
    calib_process.motor_calib_cnt = 1;

    p("calib %+5.2f %+5.2f", enc_offset[0].zero_calib, enc_offset[1].zero_calib);
    HAL_Delay(900);
  }

  static int print_cnt = 0;
  print_cnt++;
  if (print_cnt > 1000) {
    print_cnt = 0;
    p("%4.1f CNT %4d %4d %4d / %4d %4.1f\n", cmd[0].out_v_final, htim1.Instance->CCR1, htim1.Instance->CCR2, htim1.Instance->CCR3, calib_process.enc_calib_cnt, getBatteryVoltage());
  }
}

static bool checkMotorRpsError(float m0, float m1)
{
  if (fabsf(m0 - m1) > MOTOR_CALIB_M0_M1_ERROR_TRERANCE) {
    p("\n\nCALIBRATION ERROR!!!\n\n");
    calib_process.motor_calib_cnt = 0;
    return true;
  }
  if (m0 < MOTOR_CALIB_UNDER_LIMIT && m1 < MOTOR_CALIB_UNDER_LIMIT) {
    p("\n\nCALIBRATION ERROR!!!\n\n");
    calib_process.motor_calib_cnt = 0;
    return true;
  }
  return false;
}

static bool checkMotorRpsHighLowError(float m0_cw, float m1_cw, float m0_ccw, float m1_ccw)
{
  if (fabsf(m0_cw - m0_ccw) > MOTOR_CALIB_CW_CCW_ERROR_TRERANCE || fabsf(m1_cw - m1_ccw) > MOTOR_CALIB_CW_CCW_ERROR_TRERANCE) {
    p("\n\nCALIBRATION ERROR!!! CW-CCW PARAM UNMATCH\n\n");
    calib_process.motor_calib_cnt = 0;
    return true;
  }
  return false;
}

void motorCalibrationMode(void)
{
  if (calib_process.motor_calib_cnt > 1) {
    calib_process.motor_calib_cnt--;
  }

  // Output Voltage Override
  for (int i = 0; i < 2; i++) {
    if (calib_process.motor_calib_cnt > 1) {
      if (calib_process.motor_calib_cnt < MOTOR_CALIB_START_CNT) {
        calib[i].rps_integral += motor_real[i].rps;
      }

      // Apply configured calibration voltage only after ready period.
      if (calib_process.motor_calib_cnt < MOTOR_CALIB_READY_CNT) {
        cmd[i].out_v = calib_process.motor_calib_voltage;
      } else {
        cmd[i].out_v = 0;
      }
    } else if (calib_process.motor_calib_cnt == 1) {
      cmd[i].out_v = 0;
    }

    setFinalOutputVoltage(&cmd[i], &enc_offset[i], sys.manual_offset_radian);  // select Vq-offset angle
  }

  if (calib_process.motor_calib_cnt == 1) {
    static float rps_per_v_cw_l[2], rps_per_v_ccw_l[2], rps_per_v_cw_h[2], rps_per_v_ccw_h[2];

    switch (calib_process.motor_calib_mode) {
      case MOTOR_CALIB_STAGE_INIT:
        p("\n\nstart motor calib!!\n\n");
        calib_process.motor_calib_cnt = MOTOR_CALIB_INIT_CNT;

        calib_process.motor_calib_mode = MOTOR_CALIB_STAGE_LOW_CW;
        calib_process.motor_calib_voltage = MOTOR_CALIB_VOLTAGE_LOW;
        calib[0].rps_integral = 0;
        calib[1].rps_integral = 0;
        p("set output V = %f\n", calib_process.motor_calib_voltage);
        break;

      case MOTOR_CALIB_STAGE_LOW_CW:
        rps_per_v_cw_l[0] = calib[0].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        rps_per_v_cw_l[1] = calib[1].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        calib[0].rps_integral = 0;
        calib[1].rps_integral = 0;
        p("\n\nMotor Calib rps/v \n M0 %6.2f\n M1 %6.2f\n\n", rps_per_v_cw_l[0], rps_per_v_cw_l[1]);

        if (checkMotorRpsError(rps_per_v_cw_l[0], rps_per_v_cw_l[1])) {
          return;
        }

        calib_process.motor_calib_cnt = MOTOR_CALIB_INIT_CNT;
        calib_process.motor_calib_mode = MOTOR_CALIB_STAGE_LOW_CCW;
        calib_process.motor_calib_voltage = -MOTOR_CALIB_VOLTAGE_LOW;
        p("set output V = %f\n", calib_process.motor_calib_voltage);
        break;

      case MOTOR_CALIB_STAGE_LOW_CCW:
        rps_per_v_ccw_l[0] = calib[0].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        rps_per_v_ccw_l[1] = calib[1].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        calib[0].rps_integral = 0;
        calib[1].rps_integral = 0;
        p("\n\nMotor Calib rps/v \n M0 %6.2f\n M1 %6.2f\n\n", rps_per_v_ccw_l[0], rps_per_v_ccw_l[1]);

        if (checkMotorRpsError(rps_per_v_ccw_l[0], rps_per_v_ccw_l[1])) {
          return;
        } else if (checkMotorRpsHighLowError(rps_per_v_cw_l[0], rps_per_v_cw_l[1], rps_per_v_ccw_l[0], rps_per_v_ccw_l[1])) {
          return;
        }

        calib_process.motor_calib_cnt = MOTOR_CALIB_INIT_CNT;
        calib_process.motor_calib_mode = MOTOR_CALIB_STAGE_HIGH_CW;
        calib_process.motor_calib_voltage = MOTOR_CALIB_VOLTAGE_HIGH;
        p("set output V = %f\n", calib_process.motor_calib_voltage);
        break;

      case MOTOR_CALIB_STAGE_HIGH_CW:
        rps_per_v_cw_h[0] = calib[0].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        rps_per_v_cw_h[1] = calib[1].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        calib[0].rps_integral = 0;
        calib[1].rps_integral = 0;
        p("\n\nMotor Calib rps/v \n M0 %6.2f\n M1 %6.2f\n\n", rps_per_v_cw_h[0], rps_per_v_cw_h[1]);

        if (checkMotorRpsError(rps_per_v_cw_h[0], rps_per_v_cw_h[1])) {
          return;
        }

        calib_process.motor_calib_cnt = MOTOR_CALIB_INIT_CNT;
        calib_process.motor_calib_mode = MOTOR_CALIB_STAGE_HIGH_CCW;
        calib_process.motor_calib_voltage = -MOTOR_CALIB_VOLTAGE_HIGH;
        p("set output V = %f\n", calib_process.motor_calib_voltage);
        break;

      default:

        rps_per_v_ccw_h[0] = calib[0].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        rps_per_v_ccw_h[1] = calib[1].rps_integral / calib_process.motor_calib_voltage / MOTOR_CALIB_START_CNT;
        calib[0].rps_integral = 0;
        calib[1].rps_integral = 0;
        p("\n\nMotor Calib rps/v \n ");
        float spd_diff[2] = {0};
        spd_diff[0] = rps_per_v_ccw_h[0] - rps_per_v_cw_h[0];
        spd_diff[1] = rps_per_v_ccw_h[1] - rps_per_v_cw_h[1];
        p("M0 -%6.2f +%6.2f Diff %+6.2f\n ", rps_per_v_ccw_h[0], rps_per_v_cw_h[0], spd_diff[0]);
        p("M0 -%6.2f +%6.2f Diff %+6.2f \n", rps_per_v_ccw_h[1], rps_per_v_cw_h[1], spd_diff[1]);
        p("\n\n!!!!!!FINISH!!!!!!!!\n\n");

        if (checkMotorRpsError(rps_per_v_ccw_h[0], rps_per_v_ccw_h[1])) {
          return;
        } else if (checkMotorRpsHighLowError(rps_per_v_cw_h[0], rps_per_v_cw_h[1], rps_per_v_ccw_h[0], rps_per_v_ccw_h[1])) {
          return;
        }

        // Adjust encoder offset based on CW/CCW speed delta.
        // Use speed asymmetry to compensate final offset.
        float adj_calib[2] = {0};
        for (int i = 0; i < 2; i++) {
          adj_calib[i] = enc_offset[i].zero_calib + (spd_diff[i] / 10);
          if (adj_calib[i] < 0) {
            adj_calib[i] += 2 * M_PI;
          } else if (adj_calib[i] >= 2 * M_PI) {
            adj_calib[i] -= 2 * M_PI;
          }
        }
        writeEncCalibrationValue(adj_calib[0], adj_calib[1]);
        writeMotorCalibrationValue(rps_per_v_cw_h[0], rps_per_v_cw_h[1]);

        HAL_Delay(10);
        p("enc data : %4.2f %4.2f\n", flash.calib[0], flash.calib[1]);
        p("motor data : %4.2f %4.2f\n", flash.rps_per_v_cw[0], flash.rps_per_v_cw[1]);

        HAL_Delay(1000);

        NVIC_SystemReset();

        break;
    }
  }

  static int print_cnt = 0;
  print_cnt++;
  if (print_cnt > 100) {
    print_cnt = 0;
    p("%4.1f %4.1f\n", cmd[0].out_v, cmd[1].out_v);
  }
}

void startCalibrationMode(void)
{
  p("calibration mode!\n");

  calib_process.enc_calib_cnt = MOTOR_CALIB_INIT_CNT;
  calib_process.motor_calib_cnt = 0;
  sys.manual_offset_radian = 0;

  cmd[0].speed = 0;
  cmd[1].speed = 0;

  cmd[0].out_v_final = 2.0;
  cmd[1].out_v_final = 2.0;
  // Calibration flow overview:
  // 1) Encoder calibration starts here.
  // 2) IRQ rotates motor electrically and collects angle statistics.
  // 3) CAN command processing is ignored during calibration.
  // 4) Final offset is computed in encoderCalibrationMode().

  // Motor calibration flow overview:
  // 1) Starts after encoder calibration is done.
  // 2) Voltage is applied in CW/CCW, low/high steps.
  // 3) Results are validated then saved in flash.
  // 4) Runtime resumes in runMode() after reset.
  // Main loop path after calibration: runMode().
  // (kept intentionally concise)


  // main : runMode()
}
