/* BLDCアプリをCAN経由で更新し、順序・欠落・重複・CRC・Flash異常を検出する。 */
#include "boot_can_update.h"
#include "board_io.h"
#include "boot_config.h"
#include "boot_crc32c.h"
#include "boot_image.h"
#include "stm32f303xc.h"
#include <stddef.h>
#include <stdint.h>

#define CAN_COMMAND_ID UINT32_C(0x610)
#define CAN_DATA_ID_BASE UINT32_C(0x480)
#define CAN_DATA_ID_LAST UINT32_C(0x4FF)
#define CAN_RESPONSE_BASE UINT32_C(0x650)
#define BLOCK_CAPACITY 896U
#define RX_FIFO_CAPACITY 32U
enum { CMD_HELLO=1, CMD_BEGIN, CMD_SET_CRC, CMD_BLOCK_BEGIN, CMD_BLOCK_END, CMD_END, CMD_REBOOT };
enum { STATUS_OK=0, STATUS_COMMAND, STATUS_RANGE, STATUS_SEQUENCE, STATUS_CRC, STATUS_FLASH };
typedef struct { uint32_t id; uint8_t data[8]; } can_frame_t;
static uint8_t block[BLOCK_CAPACITY];
static can_frame_t fifo[RX_FIFO_CAPACITY];
static uint32_t image_size, image_crc, received, block_offset, bitmap[4];
static uint16_t block_length;
static uint8_t block_token, session, head, tail, count, node_id;
static bool receiving, begun, overflow;

static void can_init(void) {
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; RCC->APB1ENR |= RCC_APB1ENR_CANEN;
  GPIOA->MODER = (GPIOA->MODER & ~((UINT32_C(3)<<22U)|(UINT32_C(3)<<24U))) | (UINT32_C(2)<<22U) | (UINT32_C(2)<<24U);
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~((UINT32_C(0xF)<<12U)|(UINT32_C(0xF)<<16U))) | (UINT32_C(9)<<12U) | (UINT32_C(9)<<16U);
  CAN->MCR = CAN_MCR_INRQ | CAN_MCR_ABOM; while ((CAN->MSR & CAN_MSR_INAK)==0U) {}
  CAN->BTR = (UINT32_C(4)<<CAN_BTR_TS1_Pos) | (UINT32_C(1)<<CAN_BTR_TS2_Pos);
  CAN->FMR |= CAN_FMR_FINIT; CAN->FA1R=0U; CAN->FS1R=3U; CAN->FM1R=0U; CAN->FFA1R=0U;
  CAN->sFilterRegister[0].FR1=CAN_COMMAND_ID<<21U; CAN->sFilterRegister[0].FR2=UINT32_C(0x7FF)<<21U;
  CAN->sFilterRegister[1].FR1=CAN_DATA_ID_BASE<<21U; CAN->sFilterRegister[1].FR2=UINT32_C(0x780)<<21U;
  CAN->FA1R=3U; CAN->FMR &= ~CAN_FMR_FINIT; CAN->MCR &= ~CAN_MCR_INRQ; while ((CAN->MSR & CAN_MSR_INAK)!=0U) {}
}
static bool hw_read(uint32_t *id, uint8_t data[8]) {
  if ((CAN->RF0R & CAN_RF0R_FMP0_Msk)==0U) return false;
  const CAN_FIFOMailBox_TypeDef *mb=&CAN->sFIFOMailBox[0]; *id=(mb->RIR>>21U)&UINT32_C(0x7FF);
  const uint32_t lo=mb->RDLR, hi=mb->RDHR;
  for(uint32_t i=0;i<4U;i++) data[i]=(uint8_t)(lo>>(i*8U));
  for(uint32_t i=0;i<4U;i++) data[i+4U]=(uint8_t)(hi>>(i*8U));
  CAN->RF0R |= CAN_RF0R_RFOM0; return true;
}
static void drain(void) {
  if ((CAN->RF0R & CAN_RF0R_FOVR0)!=0U) { CAN->RF0R|=CAN_RF0R_FOVR0; overflow=true; }
  while ((CAN->RF0R & CAN_RF0R_FMP0_Msk)!=0U) {
    if (count>=RX_FIFO_CAPACITY) { uint32_t id; uint8_t data[8]; (void)hw_read(&id,data); overflow=true; continue; }
    (void)hw_read(&fifo[tail].id,fifo[tail].data); tail=(uint8_t)((tail+1U)%RX_FIFO_CAPACITY); count++;
  }
}
static bool pop(uint32_t *id,uint8_t data[8]) {
  if(count==0U)return false;
  *id=fifo[head].id; for(uint32_t i=0;i<8U;i++)data[i]=fifo[head].data[i]; head=(uint8_t)((head+1U)%RX_FIFO_CAPACITY); count--; return true;
}
static void send_reply(const uint8_t data[8]) {
  while((CAN->TSR&CAN_TSR_TME0)==0U){} CAN_TxMailBox_TypeDef *mb=&CAN->sTxMailBox[0]; mb->TDTR=8U;
  mb->TDLR=(uint32_t)data[0]|((uint32_t)data[1]<<8U)|((uint32_t)data[2]<<16U)|((uint32_t)data[3]<<24U);
  mb->TDHR=(uint32_t)data[4]|((uint32_t)data[5]<<8U)|((uint32_t)data[6]<<16U)|((uint32_t)data[7]<<24U);
  mb->TIR=((CAN_RESPONSE_BASE+node_id)<<21U)|CAN_TI0R_TXRQ;
}
static void respond(uint8_t command,uint8_t status,uint32_t value) { const uint8_t r[8]={(uint8_t)(command|0x80U),status,node_id,block_token,(uint8_t)value,(uint8_t)(value>>8U),(uint8_t)(value>>16U),(uint8_t)(value>>24U)}; send_reply(r); }
static bool flash_wait(void) { while((FLASH->SR&FLASH_SR_BSY)!=0U){IWDG->KR=UINT32_C(0xAAAA);} const uint32_t e=FLASH->SR&(FLASH_SR_PGERR|FLASH_SR_WRPERR); FLASH->SR=FLASH_SR_EOP|FLASH_SR_PGERR|FLASH_SR_WRPERR; IWDG->KR=UINT32_C(0xAAAA); return e==0U; }
static void flash_unlock(void) { if((FLASH->CR&FLASH_CR_LOCK)!=0U){FLASH->KEYR=UINT32_C(0x45670123);FLASH->KEYR=UINT32_C(0xCDEF89AB);} }
static bool erase_page(uint32_t address) { flash_unlock();if(!flash_wait())return false;FLASH->CR=FLASH_CR_PER;FLASH->AR=address;FLASH->CR|=FLASH_CR_STRT;const bool ok=flash_wait();FLASH->CR=0U;return ok; }
static bool program(uint32_t address,const uint8_t *data,uint32_t length) {
  flash_unlock(); for(uint32_t i=0;i<length;i+=2U){const uint16_t v=(uint16_t)data[i]|((uint16_t)(i+1U<length?data[i+1U]:UINT8_C(0xFF))<<8U);FLASH->CR=FLASH_CR_PG;*(volatile uint16_t *)(address+i)=v;if(!flash_wait()||*(const uint16_t *)(address+i)!=v){FLASH->CR=0U;return false;}} FLASH->CR=0U;return true;
}
static uint32_t u32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8U)|((uint32_t)p[2]<<16U)|((uint32_t)p[3]<<24U);}
static bool complete(void){const uint32_t n=((uint32_t)block_length+6U)/7U;for(uint32_t i=0;i<n;i++)if((bitmap[i/32U]&(UINT32_C(1)<<(i%32U)))==0U)return false;return true;}
static bool write_metadata(void){boot_image_metadata_t m={BOOT_IMAGE_METADATA_MAGIC,BOOT_IMAGE_METADATA_FORMAT,sizeof(m),1U,BOOT_IMAGE_STATE_CONFIRMED,BOOT_APP_SLOT,BOOT_APP_BASE,image_size,image_crc,0U};m.record_crc32c=boot_crc32c(&m,offsetof(boot_image_metadata_t,record_crc32c));return erase_page(BOOT_METADATA_BASE)&&program(BOOT_METADATA_BASE,(const uint8_t *)&m,sizeof(m));}
static void command(const uint8_t data[8]) {
  const uint8_t cmd=data[0]; if(data[1]!=node_id)return;
  if(cmd==CMD_HELLO){block_token=data[2];respond(cmd,STATUS_OK,received);return;}
  if(cmd==CMD_BEGIN){const uint32_t size=u32(&data[4]);block_token=data[2];if(begun&&data[2]==session&&size==image_size){respond(cmd,STATUS_OK,received);return;}session=data[2];image_size=size;received=0U;receiving=false;if(size<8U||size>BOOT_APP_SIZE){respond(cmd,STATUS_RANGE,0U);return;}if(!erase_page(BOOT_METADATA_BASE)){respond(cmd,STATUS_FLASH,0U);return;}for(uint32_t a=BOOT_APP_BASE;a<BOOT_APP_BASE+BOOT_APP_SIZE;a+=UINT32_C(0x800))if(!erase_page(a)){respond(cmd,STATUS_FLASH,a);return;}begun=true;respond(cmd,STATUS_OK,0U);return;}
  if(cmd==CMD_SET_CRC){block_token=data[2];image_crc=u32(&data[4]);respond(cmd,STATUS_OK,image_crc);return;}
  if(cmd==CMD_BLOCK_BEGIN){block_offset=u32(&data[4]);block_token=data[2];const uint32_t rem=block_offset<image_size?image_size-block_offset:0U;block_length=(uint16_t)(rem<BLOCK_CAPACITY?rem:BLOCK_CAPACITY);for(uint32_t i=0;i<4U;i++)bitmap[i]=0U;overflow=false;receiving=begun&&block_offset==received&&block_offset<image_size;respond(cmd,receiving?STATUS_OK:STATUS_RANGE,received);return;}
  if(cmd==CMD_BLOCK_END){block_token=data[2];if(!receiving||overflow||!complete()){receiving=false;respond(cmd,STATUS_SEQUENCE,received);return;}if(boot_crc32c(block,block_length)!=u32(&data[4])){receiving=false;respond(cmd,STATUS_CRC,received);return;}if(!program(BOOT_APP_BASE+block_offset,block,block_length)){respond(cmd,STATUS_FLASH,received);return;}received+=block_length;receiving=false;respond(cmd,STATUS_OK,received);return;}
  if(cmd==CMD_END){block_token=data[2];if(received!=image_size||boot_crc32c((const void *)BOOT_APP_BASE,image_size)!=image_crc){respond(cmd,STATUS_CRC,received);return;}if(!write_metadata()){respond(cmd,STATUS_FLASH,received);return;}respond(cmd,STATUS_OK,received);return;}
  if(cmd==CMD_REBOOT){block_token=data[2];respond(cmd,STATUS_OK,received);for(volatile uint32_t d=0;d<80000U;d++){}NVIC_SystemReset();}
  respond(cmd,STATUS_COMMAND,received);
}
bool boot_can_update_run(unsigned int idle_loops) {
  node_id=board_update_node_id();can_init();
  for(unsigned int idle=0;idle<idle_loops||!boot_app_is_valid();idle++){uint32_t id;uint8_t data[8];IWDG->KR=UINT32_C(0xAAAA);drain();if(pop(&id,data)){idle=0U;if(id==CAN_COMMAND_ID){if(overflow){block_token=data[2];overflow=false;receiving=false;respond(data[0],STATUS_SEQUENCE,received);}else command(data);}else if(id>=CAN_DATA_ID_BASE&&id<=CAN_DATA_ID_LAST&&receiving&&data[0]==block_token){const uint32_t seq=id-CAN_DATA_ID_BASE,pos=seq*7U;if(pos<block_length&&(bitmap[seq/32U]&(UINT32_C(1)<<(seq%32U)))==0U){for(uint32_t i=1U;i<8U&&pos+i-1U<block_length;i++)block[pos+i-1U]=data[i];bitmap[seq/32U]|=UINT32_C(1)<<(seq%32U);}}}}
  return boot_app_is_valid();
}
