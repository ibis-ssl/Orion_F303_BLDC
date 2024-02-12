#pragma once

#include "main.h"

typedef struct
{
    float calib[2];
    uint32_t board_id;
    float rps_per_v_cw[2];
    float rps_per_v_ccw[2]; //unused
}flash_t;

extern flash_t flash;

void writeCanBoardID(uint32_t board_id);
void writeEncCalibrationValue(float calib_m0, float calib_m1);
void writeMotorCalibrationValue(float calib_m0, float calib_m1);
void loadFlashData(void);