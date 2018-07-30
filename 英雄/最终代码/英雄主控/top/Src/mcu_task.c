#include "mcu_task.h"
#include "usart.h"

uint8_t mcu_data[4];
uint8_t eggflag=0;
//uint8_t tognum=0;

void mcu_task(void)
{
	mcu_data[0]=0xff;
	mcu_data[1]=eggflag;
	mcu_data[3]=0xfe;
	
	HAL_UART_Transmit(&huart6,mcu_data,sizeof(mcu_data),1000);
}

