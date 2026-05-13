#pragma once

#include "main.h"

typedef struct
{
    float calib[2];
    uint32_t board_id;
    float rps_per_v_cw[2];
    float voltage_offset[2];
}flash_t;

extern flash_t flash;

void writeCanBoardID(uint32_t board_id);
void writeEncCalibrationValue(float calib_m0, float calib_m1);
void writeMotorCalibrationValue(float calib_m0, float calib_m1);
void writeMotorModelValue(float rps_per_v_m0, float rps_per_v_m1, float offset_m0, float offset_m1);
void writeFullCalibrationValue(float calib_m0, float calib_m1, float rps_per_v_m0, float rps_per_v_m1, float offset_m0, float offset_m1);
void loadFlashData(void);
