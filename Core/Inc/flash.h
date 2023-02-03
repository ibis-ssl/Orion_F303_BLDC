#pragma once

#include "main.h"

typedef struct
{
    float calib[2];
    uint32_t can_id;
}flash_t;

extern flash_t flash;

void writeCanID(uint32_t addr);
void writeCalibrationValue(float calib_m0, float calib_m1);
void loadFlashData(void);