#include "mcu_task.h"
#include "egg_task.h"
#include "stdint.h"

uint8_t uart6_rx_buff[MCU_LEN];
uint8_t uart6_rx_bytes[8];
uint8_t mcu_data[2]={0,0};
double tog_ang=0;
void MCU_Process(void)
{
	uint8_t i;
	uint8_t start_flag=1;
	static uint8_t shootkey_sta=1;


	for(i=0;i<8;i++)
	{
		if(uart6_rx_bytes[i]==0xff&&uart6_rx_bytes[i+3]==0xfe&&start_flag)
		{ 
			if((uart6_rx_bytes[i+1]==0)|(uart6_rx_bytes[i+1]==1))
			{
				if(uart6_rx_bytes[i+2]<10) 
				{
					mcu_data[0]  = 	uart6_rx_bytes[i+1];	
					mcu_data[1]  =	uart6_rx_bytes[i+2];
					switch(shootkey_sta)
					{
						case 1:
									if( mcu_data[1] )
										shootkey_sta=2;
									break;
						case 2:
									if( mcu_data[1])
										shootkey_sta=3;
									else
										shootkey_sta=1;
									break;
						case 3:
									if( !mcu_data[1])
									{
										shootkey_sta=1;
										tog_ang += 90;
									}
									break;
						default:break;			
					}						
					start_flag	=0;
				}
			}
			
		}
	}
}
