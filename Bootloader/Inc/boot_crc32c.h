/* BLDCブートローダーで使用するCRC32C APIを宣言する。 */
#pragma once
#include <stddef.h>
#include <stdint.h>
uint32_t boot_crc32c(const void *data, size_t size);
