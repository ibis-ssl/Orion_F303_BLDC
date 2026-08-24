/* metadata、vector、CRC32Cを検証し、正常なBLDCアプリへreset相当状態でjumpする。 */
#include "boot_image.h"
#include "boot_crc32c.h"
#include "stm32f303xc.h"
#include <stddef.h>
#include <stdint.h>
static const boot_image_metadata_t *const metadata = (const boot_image_metadata_t *)BOOT_METADATA_BASE;
static bool valid_sp(uint32_t sp) { return (((sp >= BOOT_SRAM_BASE && sp <= BOOT_SRAM_END) || (sp >= BOOT_CCMRAM_BASE && sp <= BOOT_CCMRAM_END)) && (sp & 7U) == 0U); }
bool boot_app_is_valid(void) {
  const boot_image_metadata_t m = *metadata;
  if (m.magic != BOOT_IMAGE_METADATA_MAGIC || m.format_version != BOOT_IMAGE_METADATA_FORMAT || m.record_size != sizeof(m) || m.state != BOOT_IMAGE_STATE_CONFIRMED || m.slot != BOOT_APP_SLOT || m.image_base != BOOT_APP_BASE || m.image_size < 8U || m.image_size > BOOT_APP_SIZE) return false;
  if (boot_crc32c(&m, offsetof(boot_image_metadata_t, record_crc32c)) != m.record_crc32c) return false;
  const uint32_t sp = *(const uint32_t *)BOOT_APP_BASE, rh = *(const uint32_t *)(BOOT_APP_BASE + 4U), ha = rh & ~UINT32_C(1);
  if (!valid_sp(sp) || (rh & 1U) == 0U || ha < BOOT_APP_BASE || ha >= BOOT_APP_BASE + m.image_size) return false;
  return boot_crc32c((const void *)BOOT_APP_BASE, m.image_size) == m.image_crc32c;
}
static void branch_to(uint32_t sp, uint32_t rh) __attribute__((naked, noreturn));
static void branch_to(uint32_t sp __attribute__((unused)), uint32_t rh __attribute__((unused))) { __asm volatile("movs r2,#0\nmsr control,r2\nmsr basepri,r2\nmsr faultmask,r2\nisb\nmsr msp,r0\nmsr primask,r2\nbx r1\n"); }
void boot_jump_to_app(void) {
  const uint32_t sp = *(const uint32_t *)BOOT_APP_BASE, rh = *(const uint32_t *)(BOOT_APP_BASE + 4U);
  __disable_irq(); SysTick->CTRL = 0U; SysTick->LOAD = 0U; SysTick->VAL = 0U;
  for (uint32_t i = 0; i < 8U; i++) { NVIC->ICER[i] = UINT32_MAX; NVIC->ICPR[i] = UINT32_MAX; }
  SCB->VTOR = BOOT_APP_BASE; __DSB(); __ISB(); branch_to(sp, rh);
}
