#include "mpu6050.h"
#include "stdint.h"
unsigned char uart3_rx_buff[22];
float mpua[3];
float mpuw[3];
float mpuang[3];
float mpuT;

void MPUDataProcess(void)
{
	char i;
	short start_flag=1;
	for(i=0;i<22;i++)
	{
		if(uart3_rx_buff[i]==0x55&&uart3_rx_buff[i+11]==0x55)
		{
			switch(uart3_rx_buff[i+1])
			{
				case 0x51://加速度包,单位为g/s^2   1对应pitch  2对应yaw
							mpua[0] = ((short)(uart3_rx_buff [i+3]<<8| uart3_rx_buff [i+2]))/32768.0*16;
							mpua[1] = ((short)(uart3_rx_buff [i+5]<<8| uart3_rx_buff [i+4]))/32768.0*16;
							mpua[2] = ((short)(uart3_rx_buff [i+7]<<8| uart3_rx_buff [i+6]))/32768.0*16;
							mpuT = ((short)(uart3_rx_buff [i+9]<<8| uart3_rx_buff [i+8]))/340.0+36.25;
							break;
				case 0x52://角速度包,单位为度/s
							mpuw[0] = ((short)(uart3_rx_buff [i+3]<<8| uart3_rx_buff [i+2]))/32768.0*2000;
							mpuw[1] = ((short)(uart3_rx_buff [i+5]<<8| uart3_rx_buff [i+4]))/32768.0*2000 ;
							mpuw[2] = ((short)(uart3_rx_buff [i+7]<<8| uart3_rx_buff [i+6]))/32768.0*2000;
							mpuT = ((short)(uart3_rx_buff [i+9]<<8| uart3_rx_buff [i+8]))/340.0+36.25;
							break;
				case 0x53://角度包,单位为度
							mpuang[0] = ((short)(uart3_rx_buff [i+3]<<8| uart3_rx_buff [i+2]))/32768.0*180;
							mpuang[1] = ((short)(uart3_rx_buff [i+5]<<8| uart3_rx_buff [i+4]))/32768.0*180;
							mpuang[2] = ((short)(uart3_rx_buff [i+7]<<8| uart3_rx_buff [i+6]))/32768.0*180;
							mpuT = ((short)(uart3_rx_buff [i+9]<<8| uart3_rx_buff [i+8]))/340.0+36.25;
							break;
				default:break;
			}
			start_flag=0;		
		}
	}
}
