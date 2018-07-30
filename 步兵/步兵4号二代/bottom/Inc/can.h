
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "mytype.h"
	 
#define CAN1_RX0_INT_ENABLE	1		////CAN1接收RX0中断使能.0,不使能;1,使能.

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern void Error_Handler(void);

void MX_CAN1_Init(void);
extern u8 CAN1_Mode_Init(u32 tsjw,u32 tbs2,u32 tbs1,u16 brp,u32 mode);
extern void CAN_Filter_Init_Recv_All(CAN_HandleTypeDef* _hcan);
extern void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan);//can1 和 can2滤波器配置
void MX_CAN2_Init(void);


#ifdef __cplusplus
}
#endif
#endif /*__ can_H */


