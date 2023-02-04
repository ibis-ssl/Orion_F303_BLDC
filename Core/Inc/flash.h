#pragma once

#include "main.h"

typedef struct
{
    float calib[2];
    uint32_t board_id;
}flash_t;

extern flash_t flash;

void writeCanBoardID(uint32_t board_id);
void writeCalibrationValue(float calib_m0, float calib_m1);
void loadFlashData(void);