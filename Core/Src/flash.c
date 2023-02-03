#include "flash.h"
#include <string.h>

#define FLASH_ADDR_DATA (0x801F000)
#define FLASH_ADDR_CAN_ID (FLASH_ADDR_DATA + 0)
#define FLASH_ADDR_CALIB_M0 (FLASH_ADDR_CAN_ID + 4)
#define FLASH_ADDR_CALIB_M1 (FLASH_ADDR_CALIB_M0 + 4)

flash_t flash;
// 2K / page
// 128Kbyte -> 64page
// page : 0~
static void writeFlash(uint32_t can_id, float calib_m0, float calib_m1)
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
    flash_raw = can_id;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_CAN_ID, flash_raw);
    memcpy(&flash_raw, &calib_m0, 4);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_CALIB_M0, flash_raw);
    memcpy(&flash_raw, &calib_m1, 4);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_ADDR_CALIB_M1, flash_raw);
    HAL_FLASH_Lock();
}

void writeCanID(uint32_t id){
    writeFlash(id, flash.calib[0], flash.calib[1]);
}
void writeCalibrationValue(float calib_m0, float calib_m1){
    writeFlash(flash.can_id,calib_m0,calib_m1);
}

void loadFlashData(void){
    memcpy(&flash.can_id, (uint32_t *)FLASH_ADDR_CAN_ID, 4);
    memcpy(&flash.calib[0], (uint32_t *)FLASH_ADDR_CALIB_M0, 4);
    memcpy(&flash.calib[1], (uint32_t *)FLASH_ADDR_CALIB_M1, 4);
}
