#ifndef  __M_REMOTE_H
#define __M_REMOTE_H

#include "mytype.h"
#include "can.h"
#include "main.h"
#include "stdint.h"
#include "stdio.h"
#include "usart.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"
#include "can.h"
#include "m_moto.h"
#include "pid.h"
#define NOWOF2312  0
#define LASTOF2312 1
#define RC_FRAME_LENGTH    18u

//typedef  __packed struct 
//{
//  __packed struct 
//	{
//	  uint16_t ch0;
//		uint16_t ch1;
//		uint16_t ch2;
//		uint16_t ch3;
//		uint8_t   s1;
//		uint8_t   s2;
//	}rc;

//  __packed struct
//	{
//	  int16_t x;
//		int16_t y;
//		int16_t z;
//		int8_t press_1;
//		int8_t press_2;
//	}mouse;
//	
//	__packed struct 
//	{
//		uint16_t v;
//	}key;
//	
//}RC_Ctl_t;
typedef __packed struct
{
	uint16_t ch0;
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint8_t s1;
	uint8_t s2;
}Remote;
typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	int8_t press_1;
	int8_t press_2;
}Mouse;	
typedef	__packed struct
{
	uint16_t v;
}Key;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;



extern volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH];
extern RC_Ctl_t RC_CtrlData;

extern unsigned short pidswitch_flag[3];

extern void CAN1_TX_IRQHandler(void);
extern void CAN1_RX0_IRQHandler(void);
extern void CAN2_TX_IRQHandler(void);
extern void CAN2_RX0_IRQHandler(void);
extern void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan);
extern void RemoteDataProcess(uint8_t *pData,Remote *rc);
extern void receive_instruction(void);
#endif

