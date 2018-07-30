#include "egg_task.h"
#include "t_cylinder.h"
#include "pid.h"
#include "m_moto.h"
#include "mcu_task.h"
float cyl_ang=0;
float cyl_spd=0;
float cyl_err=5000;
float tog_spd=0;
short egg_task_flag=0;
short egg_task_temp=0;
int timeofegg=300;
bool limitswt[3]={1,1,1};
int testspd=10000;
 void egg_task(void)
{	
//	egg_task_temp=mcu_data[0];
//	egg_task_flag=egg_task_temp;
	
	if(egg_task_flag)  
	{
		tog_spd=-1000;
		//上升段一
		cyl_spd= -testspd;
		//cyl_ang=-2300;
		//cyl_err=cyl_ang*19*8192/360-(moto_chassis[0].total_angle-moto_chassis[0].offset_angle);
	    limitswt[0]=HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0);
		while(limitswt[0])
		{
			limitswt[0]=HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0);
		}
		cyl_spd=0;
		//夹取动作
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);	//气缸1伸出	
		HAL_Delay(CYTIME);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_SET);	//气缸2伸出
		HAL_Delay(CYTIME);
		 
		//上升段二
		cyl_spd=-testspd;
		//cyl_ang=-5200;
		//cyl_err=cyl_ang*19*8192/360-(moto_chassis[0].total_angle-moto_chassis[0].offset_angle);	
		limitswt[1]=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4);
		while(limitswt[1])
		{
			limitswt[1]=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_4);
		}
		cyl_spd=0;
		
		//收回动作
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);	//气缸1回缩
		HAL_Delay(400);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_SET);	//气缸3伸出
		HAL_Delay(CYTIME);

		//伸出扔弹药箱动作
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_6,GPIO_PIN_RESET);	//气缸3回缩
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_SET);	//气缸1伸出	
		HAL_Delay(1000);//延时确保弹药箱复位
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);	//气缸2回缩	
		HAL_Delay(400);
		HAL_GPIO_WritePin(GPIOF,GPIO_PIN_1,GPIO_PIN_RESET);	//气缸1回缩
		
		cyl_spd=testspd;
		limitswt[2]=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12);
		while(limitswt[2])
		{
			limitswt[2]=HAL_GPIO_ReadPin(GPIOE,GPIO_PIN_12);
		}
		cyl_spd=0;
		tog_spd=0;
		egg_task_flag=0;
	}

}

