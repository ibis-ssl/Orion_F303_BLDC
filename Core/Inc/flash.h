#pragma once

#include "main.h"

typedef struct
{
  float calib[2];
  uint32_t board_id;
  float rps_per_v_cw[2];
  float rps_per_v_ccw[2];  //unused
} flash_t;

extern flash_t flash;

void flash_write_board_id(uint32_t board_id);
void flash_write_enc_calib(float calib_m0, float calib_m1);
void flash_write_motor_calib(float calib_m0, float calib_m1);
void flash_read_saved_data(void);