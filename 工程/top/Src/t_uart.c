#include "t_uart.h"
#include "mytype.h"
#include "usart.h"
#include "test_app.h"
u8 var_r=0;


//void receive_instruction(void)
//{
//	HAL_UART_Receive(&huart2, &var_r, sizeof(var_r), 1000);
//	switch(var_r)
//	{
//		case 'a':break;
//		case 'b':LED_Red_Toggle();break;
//		case 'c':LED_Green_Toggle();break;
//		default: break;
//	}
//}


