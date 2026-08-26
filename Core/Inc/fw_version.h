/* BLDCアプリに埋め込む開発用build IDの最小形式を定義する。 */
/* アプリケーションに埋め込む開発用FW識別子と固定読出し位置を定義する。 */
#ifndef INC_FW_VERSION_H_
#define INC_FW_VERSION_H_
#include <stdint.h>
#define FW_VERSION_MAGIC UINT32_C(0x52565746)
#define FW_VERSION_FLASH_ADDRESS UINT32_C(0x08004400)
typedef struct __attribute__((packed)){uint32_t magic;uint32_t build_id;} fw_version_t;
extern const fw_version_t g_fw_version;
static inline uint32_t fw_version_build_id(void){return ((volatile const fw_version_t *)FW_VERSION_FLASH_ADDRESS)->build_id;}
#endif
