/* BLDCブートローダーの最小SystemInitを提供し、C runtime前に出力を停止する。 */
#include "board_io.h"
#include "stm32f303xc.h"
#include <stdint.h>
uint32_t SystemCoreClock = UINT32_C(8000000);
void SystemInit(void) {
#if (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
  SCB->CPACR |= (UINT32_C(3) << 20U) | (UINT32_C(3) << 22U);
#endif
  SCB->VTOR = UINT32_C(0x08000000); board_io_init_safe();
}
void SystemCoreClockUpdate(void) { SystemCoreClock = UINT32_C(8000000); }
void _init(void) {} void _fini(void) {}
