#ifndef __MCUTASK_H
#define __MCUTASK_H
#include "stdint.h"
#define MCU_LEN 7
extern uint8_t uart6_rx_bytes[8];
extern void MCU_Process(void);
extern uint8_t mcu_data[2];
extern float tog_ang;


#endif
