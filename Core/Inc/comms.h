#ifndef INC_COMMS_H_
#define INC_COMMS_H_

#include <stdint.h>

void receiveUserSerialCommand(void);
void sendCanData(void);
void initComms(void);

uint32_t getCanRxCount(void);
void clearCanRxCount(void);

#endif /* INC_COMMS_H_ */
