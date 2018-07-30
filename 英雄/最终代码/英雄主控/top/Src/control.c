#include "control.h"
#include "gpio.h"
#include "can.h"
#include "main.h"
#include "mytype.h"
#include "m_moto.h"
#include "usart.h"
#include "DJ_Protocol.h"
#include "m_toggle.h"
#include "t_moto.h"
#include "t_tk1.h"
#include "t_remote.h"
#include "t_protocol.h"
#include "m_imu.h"
#include "t_toggle.h"
#include "mcu_task.h"
#include "mpu6050.h"
uint8_t uart1_rx_buff[18];
uint8_t uart2_rx_buff[TK1_LEN];
uint8_t uart6_rx_buff[PROTOCOL_REC_LEN];
short timeof2312;
int timeof_selfrun=0;
short stateof_selfsun=1;
bool en_flagof2312;
extern float angle_set;
float offset_pitch=1100;

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
					CM_ctr();
			}
			else if(state_machine==3)
			{ 
					Toggler_ctr();
				
			}
			state_machine++;
			if(state_machine==4)
				state_machine=0;

	}
	else if(htim == &htim5)//1ms一次
	{
		if(KeyBoard.R)
		{
				TIM2->CCR1 = 1100;
				TIM2->CCR2 = 1100;
		}
		if(KeyBoard.F)
		{
				TIM2->CCR1 = 1150;
				TIM2->CCR2 = 1150;
		}
		if(KeyBoard.V)
		{
				TIM2->CCR1 = 1200;
				TIM2->CCR2 = 1200;
		}
		
		timeof_selfrun++;
		if(timeof_selfrun==600)
			stateof_selfsun=1;
		else if(timeof_selfrun==1200)
		{
			stateof_selfsun=2;
			timeof_selfrun=0; 
		}	
		
		if(timeof2312<10000)
		{
			timeof2312++;
			if(timeof2312==6000)
			{
				//TIM2->CCR1 = 1400;
				//TIM2->CCR2 = 1400;
				TIM2->CCR1 = 1150;
				TIM2->CCR2 = 1150;
				en_flagof2312=1;//没用上
			}
		}
		Set_3508_current(&hcan1,Cm3508_spd_pid[0].pos_out,Cm3508_spd_pid[1].pos_out,Cm3508_spd_pid[2].pos_out,Cm3508_spd_pid[3].pos_out); 
	//	Set_6623_current(&hcan1,-(Gm6623_yaw_spd_pid.pos_out),-(Gm6623_pit_spd_pid.pos_out),pid_Toggler_spd.pos_out,0);  
		Set_6623_current(&hcan1,-(Gm6623_yaw_spd_pid.pos_out),-(Gm6623_pit_spd_pid.pos_out)+offset_pitch,pid_Toggler_spd.pos_out,0);  

				//Set_3508_current(&hcan1,Cmsetcurt[0],Cmsetcurt[1],Cmsetcurt[2],Cmsetcurt[3]); 


	//	Set_6623_current(&hcan1,0,0,pid_Toggler_spd.pos_out,0);
	
			//	Set_6623_current(&hcan1,-(Gm6623_yaw_spd_pid.pos_out),-(Gm6623_pit_spd_pid.pos_out),0,0);  		
	
		//if(usetogger==1)
//		else
		//Set_6623_current(&hcan1,0,0,pid_Toggler_spd.pos_out,0);  //波弹控制

	
//    if(modeswitch_flag[NOW]==1)//yaw随动模式
//		{  
//			if(usetogger==1)
//				Set_6623_current(&hcan1,-(Gm6623_yaw_spd_pid.pos_out),-(Gm6623_pit_ang_pid.pos_out),pid_Toggler_spd.pos_out,0);    
//			else
//				Set_6623_current(&hcan1,-(Gm6623_yaw_spd_pid.pos_out),-(Gm6623_pit_ang_pid.pos_out),0,0);    
//		}
//		if(modeswitch_flag[NOW]==2)//yaw自动模式
//		{
//			if(usetogger==1)
//				Set_6623_current(&hcan1,-(Gm6623_yaw_spd_pid.pos_out),-(Gm6623_pit_ang_pid.pos_out),pid_Toggler_spd.pos_out,0);    
//			else
//				Set_6623_current(&hcan1,-(Gm6623_yaw_spd_pid.pos_out),-(Gm6623_pit_ang_pid.pos_out),0,0); 
//		}	
//		TIM_SetTIM12Compare(400,0);
	}
}

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
		HAL_UART_Receive_IT(&huart2,uart2_rx_bytes,16);
	}
	else if(huart == &huart3)
	{
		MPUDataProcess();
		HAL_UART_Receive_IT(&huart3, uart3_rx_buff, sizeof(uart3_rx_buff));
	}
	
	//裁判系统的数据分析函数的串口数组类型得改
//	else if(huart == &huart3)
//  {  
//		if( DataAnalysis(uart3_rx_buff) )
//		{
//			HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET);//灯亮
//			HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET);//灯亮
//		}
//		HAL_UART_Receive_IT(&huart3, uart3_rx_buff, sizeof(uart3_rx_buff));//串口每次回调后得调用这个函数再次开接收中断
//}
}

//		DataAnalysis();
//		HAL_UART_Receive_IT(&huart6, uart6_rx_buff, sizeof(uart6_rx_buff));//串口每次回调后得调用这个函数再次开接收中断

