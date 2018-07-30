#include "t_protocol.h"
#include "m_protocol.h"
#include "m_toggle.h"
#include "t_moto.h"
#include "pid.h"

u8 level=1;

float HEAT_MAX[3]={1600,3000,6000};
float HEAT_SET[3]={1400,2800,5800};

void PowerHeat_ctr(void)
{
	float Power_limit_ratio;//这个系数使得3508速度设定值不仅仅取决于遥控器，还取决于功率环。
	Power_limit_ratio = pid_calc(&Power_pid,FrameData->Data.extPowerHeatData.chassisPower,POWER_SET);
	HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET);//灯liang
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET);//灯liang
	for(int i=0;i<4;i++)
	{
		Cm3508_spd_set[i] = Cm3508_spd_set[i] * Power_limit_ratio;
	}
	
	//步兵每发射一个速度为v的17mm弹丸，热量增加v^2，我们通过降低射频来控制热量
	Toggler_spd_set = pid_calc(&Toggler_spd_pid,FrameData->Data.extPowerHeatData.shooterHeat0,HEAT_SET[level]);
	 	
}

