#ifndef __TK1_H
#define __TK1_H

#include "usart.h"
#include "control.h"
#define angx 0
#define angy 1
#define angz 2
void Tk1DataProcess(void);
extern uint8_t uart2_rx_bytes[16];
extern u16 USART_RX_STA;       //接收状态标记	
extern uint8_t autuang_limit;
extern uint8_t autuaim_flag;//自动瞄准用的标志位,见t_remote.c中autuaim()



#endif

