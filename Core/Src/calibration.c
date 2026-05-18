/**
 * @file calibration.c
 * @brief Encoder and motor calibration flow management.
 */

#include "calibration.h"

#include <math.h>

#include "adc.h"
#include "app_context.h"
#include "flash.h"
#include "foc_control.h"
#include "motor.h"
#include "tim.h"
#include "usart.h"

#define MOTOR_CALIB_INIT_CNT (2500)
#define MOTOR_CALIB_READY_CNT (2000)
#define MOTOR_CALIB_START_CNT (1500)
#define MOTOR_CALIB_VOLTAGE_LOW (3.0)
#define MOTOR_CALIB_VOLTAGE_HIGH (5.0)

#define MOTOR_CALIB_M0_M1_ERROR_TRERANCE (1.0)
#define MOTOR_CALIB_CW_CCW_ERROR_TRERANCE (1.0)
#define MOTOR_CALIB_UNDER_LIMIT (1.0)

#define ENC_CALIB_POLE_PAIRS (12U)
#define ENC_CALIB_POINTS_PER_ELEC (6U)
#define ENC_CALIB_POINTS (ENC_CALIB_POLE_PAIRS * ENC_CALIB_POINTS_PER_ELEC)
#define ENC_CALIB_PASSES (2U)
#define ENC_CALIB_SETTLE_MS (250U)
#define ENC_CALIB_SAMPLE_MS (100U)
#define ENC_CALIB_VOLTAGE (2.0f)
#define ENC_CALIB_HYST_REJECT_RAD (0.13962634016f)
#define ENC_CALIB_MRAD_SCALE (1000.0f)
#define ENC_CALIB_MDEG_SCALE (57295.7795131f)

typedef enum
{
  ENC_CALIB_STAGE_IDLE = 0,
  ENC_CALIB_STAGE_SETTLE,
  ENC_CALIB_STAGE_SAMPLE
} enc_calib_stage_t;

typedef struct
{
  enc_calib_stage_t stage;
  uint8_t pass_index;
  uint8_t point_index;
  uint16_t elapsed_ms;
  float electrical_angle;
  float point_zero_sin_sum[2];
  float point_zero_cos_sum[2];
  uint16_t point_zero_count[2];
  float forward_zero[2][ENC_CALIB_POINTS];
  float zero_sin_sum[2];
  float zero_cos_sum[2];
  float zero_hyst_sum[2];
  float zero_hyst_max[2];
  uint16_t zero_count[2];
  uint16_t zero_used_count[2];
  uint16_t zero_rejected_count[2];
} enc_calib_state_t;

static enc_calib_state_t enc_calib_state;
static volatile float enc_calib_output_radian = 0.0f;

static float normalizeAngle(float angle)
{
  while (angle >= (2.0f * (float)M_PI)) {
    angle -= 2.0f * (float)M_PI;
  }
  while (angle < 0.0f) {
    angle += 2.0f * (float)M_PI;
  }
  return angle;
}

static float angleDiffAbs(float a, float b)
{
  float diff = normalizeAngle(a - b);
  if (diff > (float)M_PI) {
    diff = (2.0f * (float)M_PI) - diff;
  }
  return fabsf(diff);
}

static int floatToMilli(float value)
{
  if (value >= 0.0f) {
    return (int)(value * 1000.0f + 0.5f);
  }
  return (int)(value * 1000.0f - 0.5f);
}

static uint8_t encCalibPointForPass(uint8_t pass_index, uint8_t point_index)
{
  if (pass_index == 0U) {
    return point_index;
  }
  return (uint8_t)((ENC_CALIB_POINTS - 1U) - point_index);
}

static float encCalibElectricalAngle(uint8_t point_index)
{
  const uint8_t electrical_point = (uint8_t)(point_index % ENC_CALIB_POINTS_PER_ELEC);
  return (2.0f * (float)M_PI * (float)electrical_point) / (float)ENC_CALIB_POINTS_PER_ELEC;
}

static void resetEncoderCalibrationState(void)
{
  enc_calib_state.stage = ENC_CALIB_STAGE_IDLE;
  enc_calib_state.pass_index = 0U;
  enc_calib_state.point_index = 0U;
  enc_calib_state.elapsed_ms = 0U;
  enc_calib_state.electrical_angle = 0.0f;
  enc_calib_output_radian = 0.0f;

  for (uint8_t m = 0U; m < 2U; m++) {
    enc_calib_state.point_zero_sin_sum[m] = 0.0f;
    enc_calib_state.point_zero_cos_sum[m] = 0.0f;
    enc_calib_state.point_zero_count[m] = 0U;
    enc_calib_state.zero_sin_sum[m] = 0.0f;
    enc_calib_state.zero_cos_sum[m] = 0.0f;
    enc_calib_state.zero_hyst_sum[m] = 0.0f;
    enc_calib_state.zero_hyst_max[m] = 0.0f;
    enc_calib_state.zero_count[m] = 0U;
    enc_calib_state.zero_used_count[m] = 0U;
    enc_calib_state.zero_rejected_count[m] = 0U;
    for (uint8_t i = 0U; i < ENC_CALIB_POINTS; i++) {
      enc_calib_state.forward_zero[m][i] = 0.0f;
    }
  }
}

static void startEncoderCalibSettle(void)
{
  const uint8_t point = encCalibPointForPass(enc_calib_state.pass_index, enc_calib_state.point_index);
  enc_calib_state.electrical_angle = encCalibElectricalAngle(point);
  enc_calib_output_radian = enc_calib_state.electrical_angle;
  enc_calib_state.stage = ENC_CALIB_STAGE_SETTLE;
  enc_calib_state.elapsed_ms = 0U;

  for (uint8_t m = 0U; m < 2U; m++) {
    enc_calib_state.point_zero_sin_sum[m] = 0.0f;
    enc_calib_state.point_zero_cos_sum[m] = 0.0f;
    enc_calib_state.point_zero_count[m] = 0U;
  }
}

void calibrationProcess_itr(bool motor)
{
  updateADC(motor);
  updateAS5047P(motor);
  focControlApplyFixedAngleVoltage(motor, enc_calib_output_radian, ENC_CALIB_VOLTAGE, MOTOR_CALIB_VOLTAGE_HIGH);
}

void encoderCalibrationMode(void)
{
  if (enc_calib_state.stage == ENC_CALIB_STAGE_IDLE) {
    startEncoderCalibSettle();
    p("enc cal start points %u pass %u settle %u sample %u voltage_mV %+d\n",
      ENC_CALIB_POINTS,
      ENC_CALIB_PASSES,
      ENC_CALIB_SETTLE_MS,
      ENC_CALIB_SAMPLE_MS,
      floatToMilli(ENC_CALIB_VOLTAGE));
    return;
  }

  if (enc_calib_state.stage == ENC_CALIB_STAGE_SETTLE) {
    if (enc_calib_state.elapsed_ms++ < ENC_CALIB_SETTLE_MS) {
      return;
    }
    enc_calib_state.stage = ENC_CALIB_STAGE_SAMPLE;
    enc_calib_state.elapsed_ms = 0U;
    return;
  }

  if (enc_calib_state.stage != ENC_CALIB_STAGE_SAMPLE) {
    return;
  }

  for (uint8_t m = 0U; m < 2U; m++) {
    const float point_zero = normalizeAngle(enc_calib_state.electrical_angle - as5047p[m].output_radian);
    enc_calib_state.point_zero_sin_sum[m] += sinf(point_zero);
    enc_calib_state.point_zero_cos_sum[m] += cosf(point_zero);
    enc_calib_state.point_zero_count[m]++;
  }

  if (++enc_calib_state.elapsed_ms < ENC_CALIB_SAMPLE_MS) {
    return;
  }

  const uint8_t point = encCalibPointForPass(enc_calib_state.pass_index, enc_calib_state.point_index);
  int raw_log[2];
  int zero_mdeg_log[2];
  int hyst_mdeg_log[2];
  uint8_t used_mask = 0U;
  for (uint8_t m = 0U; m < 2U; m++) {
    float point_zero = 0.0f;
    float hysteresis = -1.0f;
    if (enc_calib_state.point_zero_count[m] > 0U &&
        (fabsf(enc_calib_state.point_zero_sin_sum[m]) >= 0.0001f ||
         fabsf(enc_calib_state.point_zero_cos_sum[m]) >= 0.0001f)) {
      point_zero = normalizeAngle(atan2f(enc_calib_state.point_zero_sin_sum[m], enc_calib_state.point_zero_cos_sum[m]));
    }

    if (enc_calib_state.pass_index == 0U) {
      enc_calib_state.forward_zero[m][point] = point_zero;
    } else {
      hysteresis = angleDiffAbs(point_zero, enc_calib_state.forward_zero[m][point]);
      if (hysteresis <= ENC_CALIB_HYST_REJECT_RAD) {
        const float pair_sin = sinf(point_zero) + sinf(enc_calib_state.forward_zero[m][point]);
        const float pair_cos = cosf(point_zero) + cosf(enc_calib_state.forward_zero[m][point]);
        const float pair_zero = normalizeAngle(atan2f(pair_sin, pair_cos));
        enc_calib_state.zero_sin_sum[m] += sinf(pair_zero);
        enc_calib_state.zero_cos_sum[m] += cosf(pair_zero);
        enc_calib_state.zero_used_count[m]++;
        used_mask |= (uint8_t)(1U << m);
      } else {
        enc_calib_state.zero_rejected_count[m]++;
      }
      enc_calib_state.zero_hyst_sum[m] += hysteresis;
      if (hysteresis > enc_calib_state.zero_hyst_max[m]) {
        enc_calib_state.zero_hyst_max[m] = hysteresis;
      }
    }

    raw_log[m] = as5047p[m].enc_raw;
    zero_mdeg_log[m] = (int)(point_zero * ENC_CALIB_MDEG_SCALE);
    hyst_mdeg_log[m] = (hysteresis < 0.0f) ? -1 : (int)(hysteresis * ENC_CALIB_MDEG_SCALE);
    enc_calib_state.zero_count[m]++;
  }

  if (enc_calib_state.pass_index == 0U) {
    p("cal enc p%u i%02u raw %d/%d zero_mdeg %+d/%+d\n",
      enc_calib_state.pass_index,
      point,
      raw_log[0],
      raw_log[1],
      zero_mdeg_log[0],
      zero_mdeg_log[1]);
  } else {
    p("cal enc p%u i%02u raw %d/%d zero_mdeg %+d/%+d hyst %+d/%+d used %02x\n",
      enc_calib_state.pass_index,
      point,
      raw_log[0],
      raw_log[1],
      zero_mdeg_log[0],
      zero_mdeg_log[1],
      hyst_mdeg_log[0],
      hyst_mdeg_log[1],
      used_mask);
  }

  enc_calib_state.point_index++;
  if (enc_calib_state.point_index >= ENC_CALIB_POINTS) {
    enc_calib_state.point_index = 0U;
    enc_calib_state.pass_index++;
  }
  if (enc_calib_state.pass_index < ENC_CALIB_PASSES) {
    startEncoderCalibSettle();
    return;
  }

  for (uint8_t m = 0U; m < 2U; m++) {
    if (enc_calib_state.zero_used_count[m] == 0U ||
        (fabsf(enc_calib_state.zero_sin_sum[m]) < 0.0001f && fabsf(enc_calib_state.zero_cos_sum[m]) < 0.0001f)) {
      p("enc cal M%u zero NG count %u keep %+d mrad\n", m, enc_calib_state.zero_count[m], floatToMilli(enc_offset[m].zero_calib));
    } else {
      enc_offset[m].zero_calib = normalizeAngle(atan2f(enc_calib_state.zero_sin_sum[m], enc_calib_state.zero_cos_sum[m]));
    }
    flash.calib[m] = enc_offset[m].zero_calib;
    const float hyst_avg = enc_calib_state.zero_hyst_sum[m] / (float)ENC_CALIB_POINTS;
    p("enc cal M%u zero_mrad %+d pairs used/rej %u/%u hyst_mdeg avg/max %+d/%+d samples %u\n",
      m,
      floatToMilli(enc_offset[m].zero_calib),
      enc_calib_state.zero_used_count[m],
      enc_calib_state.zero_rejected_count[m],
      (int)(hyst_avg * ENC_CALIB_MDEG_SCALE),
      (int)(enc_calib_state.zero_hyst_max[m] * ENC_CALIB_MDEG_SCALE),
      enc_calib_state.zero_count[m] * ENC_CALIB_SAMPLE_MS);
  }

  cmd[0].out_v_final = 0.0f;
  cmd[1].out_v_final = 0.0f;
  setPwmAll(TIM_PWM_CENTER);
  writeEncCalibrationValue(enc_offset[0].zero_calib, enc_offset[1].zero_calib);

  calib_process.enc_calib_cnt = 0U;
  sys.manual_offset_radian = 0.0f;
  enc_calib_state.stage = ENC_CALIB_STAGE_IDLE;

  calib_process.motor_calib_mode = MOTOR_CALIB_STAGE_INIT;
  calib_process.motor_calib_cnt = 1U;

  p("enc cal done zero %+d/%+d mrad\n", floatToMilli(enc_offset[0].zero_calib), floatToMilli(enc_offset[1].zero_calib));
  HAL_Delay(10);
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

    cmd[i].out_v_final = cmd[i].out_v;
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

        // Keep the multi-point encoder zero calibration unchanged.
        // The motor step only updates the speed-to-voltage coefficient.
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

  resetEncoderCalibrationState();
  calib_process.enc_calib_cnt = MOTOR_CALIB_INIT_CNT;
  calib_process.motor_calib_cnt = 0;
  sys.manual_offset_radian = 0;

  cmd[0].speed = 0;
  cmd[1].speed = 0;

  cmd[0].out_v_final = ENC_CALIB_VOLTAGE;
  cmd[1].out_v_final = ENC_CALIB_VOLTAGE;
  // Calibration flow overview:
  // 1) Encoder calibration starts here.
  // 2) IRQ holds each fixed electrical angle while the 1kHz loop samples encoder zero.
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
