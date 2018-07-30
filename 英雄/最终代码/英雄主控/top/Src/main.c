#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "stdio.h"

#include "m_imu.h"
#include "test_app.h"
#include "test_can.h"
#include "test_uart.h"

#include "mytype.h"
#include "pid.h"
#include "m_moto.h"
#include "control.h"
#include "m_remote.h"
#include "t_remote.h"
#include "DJ_Protocol.h"
#include "t_tk1.h"
#include "t_moto.h"
#include "t_toggle.h"
#include "t_uart.h"
#include "mcu_task.h"
#include "mpu6050.h"

void SystemClock_Config(void);
void Error_Handler(void);
float angle_set=0;
float var[7]={100,100,100,100,100,100,100};
void vcan_sendware(u8 *wareaddr, u32 waresize);
void all_bsp_init(void);

void all_pid_init(void);

int main(void)
{
	//short i;
	all_bsp_init();
	all_pid_init();
//	int  time=0;
//	int flag=0;
//	int time_threshhold=1200;
	
  while (1) 
  {	
		shootnum=toggle_heat_ctr( ReadData.robot_level, ReadData.bullet_spd, ReadData.bullet_freq, ReadData.bullet_type,ReadData.shoot_heat42);

		eggflag=KeyBoard.G;

		mcu_task();
	  
//		if((	modeswitch_flag[NOW]!=2)|(angorspd==1)) 
//		{
//			for(i=0;i<4;i++)
//				Cm3508_ang_set[i] = (moto_chassis[i].total_angle-moto_chassis[i].offset_angle)*360/8192/19;//更新实时角度
//		}
//	  		var[0]=NowAngle;
//				var[1]=moto_gym[1].angle;
//				var[2]=Gm6623_pit_ang_pid.pos_out;
//				var[3]=km_nor_pit_spd_fb;
		//var[4]=Cm3508_spd_pid[2].pos_out;
		//var[5]=Cm3508_spd_pid[3].pos_out;	
    //var[6]=set_totalcurt;
//				vcan_sendware((u8 *)var, sizeof(var));
//				HAL_Delay(10);

  }
}	

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

void vcan_sendware(u8 *wareaddr, u32 waresize)
{
#define CMD_WARE     3
    u8 cmdf[2] = {CMD_WARE, ~CMD_WARE};    //串口调试 使用的前命令
    u8 cmdr[2] = {~CMD_WARE, CMD_WARE};    //串口调试 使用的后命令

		HAL_UART_Transmit(&huart6,cmdf,sizeof(cmdf),1000);
		HAL_UART_Transmit(&huart6,wareaddr,waresize,1000);
		HAL_UART_Transmit(&huart6,cmdr,sizeof(cmdr),1000);
}


void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

void all_bsp_init(void)
{
	HAL_Init();//4位抢占优先级、0位响应优先级
	SystemClock_Config();
	
	/***************************************************************/
	MX_GPIO_Init();
	HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET);//亮
	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET);//灯亮
	HAL_GPIO_WritePin(GPIOG,LASER_Pin,GPIO_PIN_SET);//灯亮

	/***************************************************************/
	
	/***************************************************************/
	MX_SPI5_Init();//板上陀螺仪
	MPU6500_Init();//陀螺仪一定要在定时器前面开启，否则初始化时会疯一下
	HAL_Delay(100);//陀螺仪初始化后的必要的延时
	/***************************************************************/
	
	/***************************************************************/
	MX_CAN1_Init();//3508、6623，中断抢占优先级为2
	my_can_filter_init_recv_all(&hcan1);//3508，6623
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);//3508，6623
	/***************************************************************/
	
	TIM12_PWM_Init(500-1,90-1);//拨弹电机//90M/90=1M的计数频率，自动重装载为500，那么PWM频率为1M/500=2kHZ   
	/***************************************************************/
	//摩擦轮初始化
	MX_TIM2_Init();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	HAL_Delay(400);//必要的延时否则启动失败	
	TIM2->CCR1 = 1000;
	TIM2->CCR2 = 1000;
	TIM2->CCR3 = 1000;
	TIM2->CCR4 = 1000;
	/***************************************************************/
	/***************************************************************/
	MX_USART1_UART_Init();//遥控器，中断抢占优先级为0
	MYDMA_Config(DMA2_Stream2,DMA_CHANNEL_4);//初始化DMA
	//HAL_UART_Receive_IT(&huart1, uart1_rx_buff, 18u);//中断方式
	while(HAL_UART_Receive_DMA(&huart1,uart1_rx_buff,18u)!=HAL_OK);//DMA方式
	/***************************************************************/

	MX_USART2_UART_Init();//TK1通信
	HAL_UART_Receive_IT(&huart2,uart2_rx_bytes,sizeof(uart2_rx_bytes));

	/***************************************************************/
	MX_USART6_UART_Init();//双机通信
	//HAL_UART_Receive_IT(&huart6, uart6_rx_buff, sizeof(uart6_rx_buff));

	MX_USART3_UART_Init();//MPU6050
	HAL_UART_Receive_IT(&huart3, uart3_rx_buff,sizeof(uart3_rx_buff));

	/***************************************************************/
	TIM3_Init(300-1,900-1);//PID控制       //定时器3初始化，定时器时钟约为90M，分频系数为900-1，                                    
	                                         //所以定时器3的频率为90M/900=100K，自动重装载为300-1，那么定时器周期就是3ms
	TIM5_Init(100-1,900-1);//电机定时输出   //定时器3初始化，定时器时钟约为90M，分频系数为900-1，                                    
	                                         //所以定时器3的频率为90M/900=100K，自动重装载为100-1，那么定时器周期就是1ms
	/***************************************************************/
	/**************************暂时没用到的***********************************/
  //MX_USB_DEVICE_Init();//暂时没用到
	  //MX_CAN2_Init();//暂时没用到
  //HAL_TIM_Base_Start_IT(&htim6);//暂时没用到
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);//暂时没用到
  //if(MPU_id != 0)sTestResult.imuTest = 0x01;//暂时没用到，好像与陀螺仪有关
  //HAL_Delay(100);
	/**************************暂时没用到的***********************************/
	HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_SET);
}

void all_pid_init(void)//PID参数初始化
{
	for(int i=0; i<4; i++)
	{
		PID_struct_init(&Cm3508_ang_pid[i], POSITION_PID, 4300,1000,P_3508_ang,I_3508_ang,D_3508_ang);  //4 motos angular rate closeloop.
		PID_struct_init(&Cm3508_spd_pid[i], POSITION_PID, 6000,3000,P_3508_spd,I_3508_spd,D_3508_spd);  //4 motos angular rate closeloop.
	}
	PID_struct_init(&Follw_3508_pid, POSITION_PID, 3000,3000,P_Follow,I_Follow,D_Follow);
	//pitch
	PID_struct_init(&Gm6623_pit_ang_pid, POSITION_PID, 5000,5000,P_6623_pit_ang,I_6623_pit_ang,D_6623_pit_ang);
	PID_struct_init(&Gm6623_pit_spd_pid, POSITION_PID, 5000,5000,P_6623_pit_spd,I_6623_pit_spd,D_6623_pit_spd);
	//yaw
	PID_struct_init(&Gm6623_yaw_ang_pid, POSITION_PID, 5000,5000,P_6623_yaw_ang,I_6623_yaw_ang,D_6623_yaw_ang);
	PID_struct_init(&Gm6623_yaw_spd_pid, POSITION_PID, 4000,3000,P_6623_yaw_spd,I_6623_yaw_spd,D_6623_yaw_spd);//总限制幅度  积分限制幅度
	//tongge
	PID_struct_init(&Gm6623_yaw_ang_pid_slow, POSITION_PID, 300,300,0.25, 0.01,0.02);
	PID_struct_init(&Gm6623_yaw_ang_pid_quick, POSITION_PID,300,300, 1.5, 0.01, 0.1);
	PID_struct_init(&Gm6623_pit_ang_pid_slow, POSITION_PID, 500, 25,0.17,  0  ,0.1);
	PID_struct_init(&Gm6623_pit_ang_pid_quick, POSITION_PID, 500, 25,0.17,  0  ,0.1);
	
	PID_struct_init(&autu_Gm6623_yaw_ang_pid, POSITION_PID, 7000,7000,P_autu_6623_yaw_ang,I_autu_6623_yaw_ang,D_autu_6623_yaw_ang);
	PID_struct_init(&pid_Toggler_ang, POSITION_PID, 1800,1800,P_tog_ang,I_tog_ang,D_tog_ang);
	PID_struct_init(&pid_Toggler_spd, POSITION_PID, 10000,10000,P_tog_spd,I_tog_spd,D_tog_spd);
}
