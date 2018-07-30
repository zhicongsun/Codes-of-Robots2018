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
#include "egg_task.h"
#include "mcu_task.h"
extern void  Cm3508_ang_ctr(pid_t *Cm3508_ang_pid, pid_t *Cm3508_spd_pid,moto_measure_t *moto3508_elevator,float* g3508_angle_set);
uint8_t uart1_rx_buff[18];
uint8_t uart2_rx_buff[TK1_LEN];
uint8_t uart3_rx_buff[PROTOCOL_REC_LEN];
short timeof2312;
int timeof_selfrun=0;
short stateof_selfsun=1;
bool en_flagof2312;
extern float angle_set;
int timeclk=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//static u8 state_machine=0;
	if(htim == &htim3)//12ms一次
	{	
		pid_calc(&Cm3508_spd_pid[0], moto_chassis[0].speed_rpm,cyl_spd);
		if(egg_task_flag)
			pid_calc(&Cm3508_spd_pid[2], moto_chassis[2].speed_rpm,tog_spd);
		else
			Cm3508_ang_ctr(&Cm3508_ang_pid[2], &Cm3508_spd_pid[2], &moto_chassis[2],&tog_ang);
//		Cm3508_ang_ctr(&Cm3508_ang_pid[1], &Cm3508_spd_pid[1], &moto_chassis[1], cyl_ang);
//		if(state_machine==0)
//		{			
//				Gm6623_yaw_ang_ctr();
//		}
//		else if(state_machine==1)
//		{
//				Gm6623_pitch_ang_ctr();
//		}
//		else if(state_machine==2)
//		{
//				if(pidswitch_flag[NOW]==1)
//				{ 
//					CM_follow_gy_dataprocess();//随动模式底盘分速度
//				}
//				if(pidswitch_flag[NOW]==2)
//				{ 	   
//					autu_CM3508_dataprocess();//自动瞄准模式底盘分速度
//				}
//				//Cm3508_spd_ctr();
//		}
//		else if(state_machine==3)
//		{ 
//			//Toggler_control();
////				pid_calc(&pid_Toggler_ang, moto_toggle.total_angle, angle_set);	
////				pid_calc(&pid_Toggler_spd, moto_toggle.speed_rpm, pid_Toggler_ang.pos_out);
//			//pid_calc(&pid_Toggler_spd, moto_toggle.speed_rpm, 2000);//先调速度环
//				//Toggler_ctr(Toggler_ang_set);
//		}
//		state_machine++;
//    if(state_machine==4)
//			state_machine=0;
	}
	else if(htim == &htim5)//1ms一次
	{
		
//		timeof_selfrun++;
//		if(timeof_selfrun==600)
//			stateof_selfsun=1;
//		else if(timeof_selfrun==1200)
//		{
//			stateof_selfsun=2;
//			timeof_selfrun=0;
//		}	
//		
//		if(timeof2312<15000)
//		{
//			timeof2312++;
//			if(timeof2312==10000)
//			{
//				TIM2->CCR1 = 1300;
//				TIM2->CCR2 = 1300;
//				en_flagof2312=1;
//			}
		//
//		Set_6623_current(&hcan1,-(Gm6623_yaw_spd_pid.pos_out),-(Gm6623_pit_spd_pid.pos_out),0,0);
		Set_3508_current(&hcan1,Cm3508_spd_pid[0].pos_out,0,Cm3508_spd_pid[2].pos_out,0);
		
		//Set_6623_current(&hcan1, 0, 0, pid_Toggler_spd.pos_out, 0);//拨弹2006
		//Set_6623_current(&hcan1, 0, 0, 0, Cm3508_spd_pid[4].pos_out);//1丝杠3508
		//Set_6623_current(&hcan1, 0, 0, Cm3508_spd_pid[5].pos_out, Cm3508_spd_pid[4].pos_out);//2丝杠3508
		
//		if(usetogger==1)
//				Set_6623_current(&hcan1,-(Gm6623_yaw_spd_pid.pos_out),-(Gm6623_pit_spd_pid.pos_out),pid_Toggler_spd.pos_out,0);  
//         		
//		else
//				Set_6623_current(&hcan1,-(Gm6623_yaw_spd_pid.pos_out),-(Gm6623_pit_spd_pid.pos_out),0,0);  		

		
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
//  if(huart == &huart6)
//  {
		MCU_Process();
		HAL_UART_Receive_IT(&huart6, uart6_rx_bytes, sizeof(uart6_rx_bytes) );
  //}
//  else if(huart == &huart2)
//  {  
//		Tk1DataProcess();
//		HAL_UART_Receive_IT(&huart2,&uart2_rx_bytes,1);//串口每次回调后得调用这个函数再次开接收中断
//  }
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

