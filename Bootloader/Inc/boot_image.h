/* BLDCアプリのmetadata形式と検証・jump APIを定義する。 */
#pragma once
#include "boot_config.h"
#include <stdbool.h>
#include <stdint.h>
typedef struct __attribute__((packed)) {
  uint32_t magic; uint16_t format_version; uint16_t record_size;
  uint32_t generation; uint32_t state; uint32_t slot; uint32_t image_base;
  uint32_t image_size; uint32_t image_crc32c; uint32_t record_crc32c;
} boot_image_metadata_t;
_Static_assert(sizeof(boot_image_metadata_t) == 36U, "metadata size mismatch");
bool boot_app_is_valid(void);
void boot_jump_to_app(void) __attribute__((noreturn));
