#include "t_tk1.h"

extern uint8_t uart2_rx_buff[TK1_LEN];
uint8_t autuaim_flag=0;//自动瞄准用的标志位,见t_remote.c中autuaim()
uint8_t autuang_limit;
uint8_t uart2_rx_bytes[16];
u16 USART_RX_STA=0;       //接收状态标记	
short int Tk1pointang[3]={0,0,0};

void Tk1DataProcess(void)//数据用完了要清零
{  uint8_t i;
	// uint8_t* real_uart2_rx_bytes;
	 uint8_t start_flag=1;
	 short* data_ptr;
  for(i=0;i<16;i++)
	 {
	   if(uart2_rx_bytes[i]==0xff&&uart2_rx_bytes[i+7]==0xfe&&start_flag)
		 { 
		   data_ptr = (short*)(uart2_rx_bytes+i+1);	
       Tk1pointang[angx] =  data_ptr[0];	
			 Tk1pointang[angy] =	data_ptr[1];
			 Tk1pointang[angz] =	data_ptr[2];
       start_flag	=0;	 //16个数中只选一次
		 }			 
		}

}

