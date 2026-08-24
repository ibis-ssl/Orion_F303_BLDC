/* 小容量BLDCブートローダー向けにtableを使わないCRC32Cを実装する。 */
#include "boot_crc32c.h"
#include <stdint.h>
uint32_t boot_crc32c(const void *data, size_t size) {
  const uint8_t *bytes = data; uint32_t crc = UINT32_MAX;
  for (size_t i = 0; i < size; i++) { crc ^= bytes[i]; for (unsigned int bit = 0; bit < 8U; bit++) crc = (crc >> 1U) ^ ((crc & 1U) ? UINT32_C(0x82F63B78) : 0U); }
  return ~crc;
}
