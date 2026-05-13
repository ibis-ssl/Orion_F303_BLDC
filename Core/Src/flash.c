/**
 * @file flash.c
 * @brief Application flash parameter storage for CAN ID, encoder alignment, and voltage motor model.
 */

#include "flash.h"
#include <math.h>
#include <string.h>

#define FLASH_ADDR_DATA (0x801F000)
#define FLASH_ADDR_CAN_ID (FLASH_ADDR_DATA + 0)
#define FLASH_ADDR_ENC_CALIB_M0 (FLASH_ADDR_CAN_ID + 4)
#define FLASH_ADDR_ENC_CALIB_M1 (FLASH_ADDR_ENC_CALIB_M0 + 4)
#define FLASH_ADDR_MOTOR_CALIB_CW_M0 (FLASH_ADDR_ENC_CALIB_M1 + 4)
#define FLASH_ADDR_MOTOR_CALIB_CW_M1 (FLASH_ADDR_MOTOR_CALIB_CW_M0 + 4)
#define FLASH_ADDR_MOTOR_CALIB_CCW_M0 (FLASH_ADDR_MOTOR_CALIB_CW_M1 + 4)
#define FLASH_ADDR_MOTOR_CALIB_CCW_M1 (FLASH_ADDR_MOTOR_CALIB_CCW_M0 + 4)

flash_t flash;
// 2K / page
// 128Kbyte -> 64page
// page : 0~
static float sanitizeVoltageOffset(float value)
{
    if (isfinite(value) && value >= 0.0f && value <= 3.0f) {
        return value;
    }
    return 0.0f;
}

static void writeFlash(uint32_t board_id,
                       float calib_m0,
                       float calib_m1,
                       float motor_calib_m0,
                       float motor_calib_m1,
                       float motor_offset_m0,
                       float motor_offset_m1)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error = 0;
    erase.TypeErase = TYPEERASE_PAGES;
    erase.PageAddress = FLASH_ADDR_DATA;
    erase.NbPages = 1;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&erase, &page_error);
    HAL_FLASH_Lock();

    HAL_FLASH_Unlock();
    uint32_t flash_raw;
    flash_raw = board_id;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_CAN_ID, flash_raw);
    memcpy(&flash_raw, &calib_m0, 4);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_ENC_CALIB_M0, flash_raw);
    memcpy(&flash_raw, &calib_m1, 4);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_ENC_CALIB_M1, flash_raw);
    memcpy(&flash_raw, &motor_calib_m0, 4);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_MOTOR_CALIB_CW_M0, flash_raw);
    memcpy(&flash_raw, &motor_calib_m1, 4);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_MOTOR_CALIB_CW_M1, flash_raw);
    memcpy(&flash_raw, &motor_offset_m0, 4);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_MOTOR_CALIB_CCW_M0, flash_raw);
    memcpy(&flash_raw, &motor_offset_m1, 4);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_MOTOR_CALIB_CCW_M1, flash_raw);
    HAL_FLASH_Lock();

    loadFlashData();
}

void writeCanBoardID(uint32_t id) { writeFlash(id, flash.calib[0], flash.calib[1], flash.rps_per_v_cw[0], flash.rps_per_v_cw[1], flash.voltage_offset[0], flash.voltage_offset[1]); }
void writeEncCalibrationValue(float calib_m0, float calib_m1) { writeFlash(flash.board_id, calib_m0, calib_m1, flash.rps_per_v_cw[0], flash.rps_per_v_cw[1], flash.voltage_offset[0], flash.voltage_offset[1]); }
void writeMotorCalibrationValue(float calib_m0, float calib_m1) { writeMotorModelValue(calib_m0, calib_m1, flash.voltage_offset[0], flash.voltage_offset[1]); }
void writeMotorModelValue(float rps_per_v_m0, float rps_per_v_m1, float offset_m0, float offset_m1)
{
    writeFlash(flash.board_id, flash.calib[0], flash.calib[1], rps_per_v_m0, rps_per_v_m1, offset_m0, offset_m1);
}
void writeFullCalibrationValue(float calib_m0, float calib_m1, float rps_per_v_m0, float rps_per_v_m1, float offset_m0, float offset_m1)
{
    writeFlash(flash.board_id, calib_m0, calib_m1, rps_per_v_m0, rps_per_v_m1, offset_m0, offset_m1);
}

void loadFlashData(void){
    memcpy(&flash.board_id, (uint32_t *)FLASH_ADDR_CAN_ID, 4);
    memcpy(&flash.calib[0], (uint32_t *)FLASH_ADDR_ENC_CALIB_M0, 4);
    memcpy(&flash.calib[1], (uint32_t *)FLASH_ADDR_ENC_CALIB_M1, 4);
    memcpy(&flash.rps_per_v_cw[0], (uint32_t *)FLASH_ADDR_MOTOR_CALIB_CW_M0, 4);
    memcpy(&flash.rps_per_v_cw[1], (uint32_t *)FLASH_ADDR_MOTOR_CALIB_CW_M1, 4);
    memcpy(&flash.voltage_offset[0], (uint32_t *)FLASH_ADDR_MOTOR_CALIB_CCW_M0, 4);
    memcpy(&flash.voltage_offset[1], (uint32_t *)FLASH_ADDR_MOTOR_CALIB_CCW_M1, 4);
    flash.voltage_offset[0] = sanitizeVoltageOffset(flash.voltage_offset[0]);
    flash.voltage_offset[1] = sanitizeVoltageOffset(flash.voltage_offset[1]);
}
