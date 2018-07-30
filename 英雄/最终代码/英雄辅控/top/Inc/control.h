#ifndef __CONTROL_H
#define __CONTROL_H


#include "mytype.h"
#include "stm32f4xx_HAL.h"
#include "usart.h"
#include "m_remote.h"

#define PROTOCOL_REC_LEN 68
#define TK1_LEN          200

extern void TIM3_Init(u16 arr,u16 psc);
extern uint8_t uart1_rx_buff[18];
extern uint8_t uart2_rx_buff[TK1_LEN];
extern uint8_t uart3_rx_buff[PROTOCOL_REC_LEN];
extern void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
extern bool en_flagof2312;
extern short stateof_selfsun;


#endif

