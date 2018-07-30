#include "control.h"
#include "gpio.h"
#include "can.h"
#include "main.h"
#include "mytype.h"
#include "m_moto.h"
#include "usart.h"
#include "m_protocol.h"
#include "m_toggle.h"
#include "t_moto.h"
#include "t_tk1.h"
#include "t_remote.h"
uint8_t uart1_rx_buff[18];
uint8_t uart2_rx_buff[TK1_LEN];
uint8_t uart3_rx_buff[PROTOCOL_REC_LEN];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static u8 state_machine=0;
	
	if(htim == &htim3)//3ms一次
	{
		if(state_machine==0)
		{			
				Gm6623_yaw_ang_ctr();
		}
		else if(state_machine==1)
		{
				Gm6623_pitch_ang_ctr();
		}
		else if(state_machine==2)
		{

				Cm3508_spd_ctr();
		}
		else if(state_machine==3)
		{ 
				//Toggler_spd_ctr();
		}
		state_machine++;
    if(state_machine==4)
			state_machine=0;
	}
	else if(htim == &htim5)//1ms一次
	{
		Set_3508_current(&hcan1,Cm3508_spd_pid[0].pos_out,Cm3508_spd_pid[1].pos_out,Cm3508_spd_pid[2].pos_out,Cm3508_spd_pid[3].pos_out);
		Set_6623_current(&hcan1,0,0,Cm3508_spd_pid[4].pos_out,Cm3508_spd_pid[5].pos_out);  		

	}
}

////单独6623尝试成功
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	
//	if(htim == &htim3)//10ms一次
//	{
//			Gm6623_yaw_ang_ctr();
//	}
//}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &huart1)
  {  
    RemoteDataProcess(uart1_rx_buff,&RC_CtrlData.rc);
	//HAL_UART_Receive_IT(&huart1, uart1_rx_buff, 18u);//中断方式
	HAL_UART_Receive_DMA(&huart1,uart1_rx_buff,18u);//DMA方式
  }
  else if(huart == &huart2)
  {  
	  Tk1DataProcess();
	  HAL_UART_Receive_IT(&huart2,&uart2_rx_bytes,1);//串口每次回调后得调用这个函数再次开接收中断
  }
//	else if(huart == &huart3)
//  {  
//		if( DataAnalysis(uart3_rx_buff) )
//		{
//			HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET);//灯亮
//			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET);//灯亮
//		}
//		HAL_UART_Receive_IT(&huart3, uart3_rx_buff, sizeof(uart3_rx_buff));//串口每次回调后得调用这个函数再次开接收中断
  }
//}

