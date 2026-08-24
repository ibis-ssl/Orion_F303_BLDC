/* BLDC基板を更新待機中の安全状態へ移し、OTAノードIDを提供する。 */
#pragma once
#include <stdbool.h>
#include <stdint.h>
void board_io_init_safe(void);
void board_status_set_validating(bool enabled);
void board_status_set_invalid(bool enabled);
uint8_t board_update_node_id(void);
