/* BLDCのPWM/ゲート出力を無効化し、Flash設定からOTAノードIDを取得する。 */
#include "board_io.h"
#include "boot_config.h"
#include "stm32f303xc.h"
#include <stdint.h>
#define PIN(n) (UINT32_C(1) << (n))
void board_io_init_safe(void) {
  RCC->APB2RSTR |= RCC_APB2RSTR_TIM1RST | RCC_APB2RSTR_TIM8RST; RCC->APB2RSTR &= ~(RCC_APB2RSTR_TIM1RST | RCC_APB2RSTR_TIM8RST);
  RCC->APB2ENR &= ~(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN);
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
  /* MX_GPIO_Initと同じくLEDはLow、ゲート制御PB6/PB7はHigh、SWはpull-up inputとする。 */
  GPIOC->BSRR = (PIN(13) | PIN(14) | PIN(15)) << 16U;
  GPIOB->BSRR = PIN(6) | PIN(7);
  GPIOB->MODER = (GPIOB->MODER & ~((UINT32_C(3) << 12U) | (UINT32_C(3) << 14U))) | (UINT32_C(1) << 12U) | (UINT32_C(1) << 14U);
  GPIOC->MODER = (GPIOC->MODER & ~((UINT32_C(3) << 26U) | (UINT32_C(3) << 28U) | (UINT32_C(3) << 30U))) | (UINT32_C(1) << 26U) | (UINT32_C(1) << 28U) | (UINT32_C(1) << 30U);
  GPIOC->MODER &= ~UINT32_C(0xFF);
  GPIOC->PUPDR = (GPIOC->PUPDR & ~UINT32_C(0xFF)) | UINT32_C(0x55);
  GPIOA->MODER = (GPIOA->MODER & ~((UINT32_C(3) << 22U) | (UINT32_C(3) << 24U))) | (UINT32_C(2) << 22U) | (UINT32_C(2) << 24U);
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((UINT32_C(0xF) << 12U) | (UINT32_C(0xF) << 16U))) | (UINT32_C(9) << 12U) | (UINT32_C(9) << 16U);
}
void board_status_set_validating(bool enabled) { (void)enabled; }
void board_status_set_invalid(bool enabled) { (void)enabled; }
uint8_t board_update_node_id(void) {
  const uint32_t board_id = *(const uint32_t *)BOOT_SETTINGS_BASE;
  return (uint8_t)(16U + (board_id <= 1U ? board_id : 0U));
}
