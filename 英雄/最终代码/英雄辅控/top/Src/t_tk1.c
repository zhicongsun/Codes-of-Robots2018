#include "t_tk1.h"

extern uint8_t uart2_rx_buff[TK1_LEN];
uint8_t autuaim_flag=0;//自动瞄准用的标志位,见t_remote.c中autuaim()
uint8_t autuang_limit;
uint8_t uart2_rx_bytes;
u16 USART_RX_STA=0;       //接收状态标记	
short int Tk1pointang[3]={0,0,0};

void Tk1DataProcess(void)
{
   if((USART_RX_STA&0x8000)==0)//接收未完成
	{	 
	   if(uart2_rx_bytes==0xfe)
		 {
			 USART_RX_STA|=0x8000;	//接收完成了 
			 short * data_ptr = (short *)(uart2_rx_buff + 1);
			 Tk1pointang[angx] =  data_ptr[0];	
			 Tk1pointang[angy] =	data_ptr[1];
			 Tk1pointang[angz] =	data_ptr[2];		
       //autuaim_flag=1;			 
     }
		 else 
		 {
			 uart2_rx_buff[USART_RX_STA&0X3FFF]=uart2_rx_bytes ;
			 USART_RX_STA++;
			 if(USART_RX_STA > (TK1_LEN-1)||uart2_rx_buff[0]!=0xff)
			 USART_RX_STA=0;//接收数据错误,重新开始接收	  
		 }		 
	 }
}

