#include  "m_toggle.h"
#include "t_remote.h"
#include "pid.h"
#include "mcu_task.h"
float Toggler_ang_set = 0;
float speedtest=0;

int Toggler_working_time;
float Backangle=0;
float error1=0;
float error2=0;
short TimeCount_shoot=0; // 拨弹转到位计数，

void Toggler_ctr(void)
{
    //static int Toggler_working_time;
	
		error2 = error2*0.8 +  (Toggler_ang_set - moto_toggle.total_angle)*0.2;
    if((error2 < Toggler_angle_zero)&&(error2 > -Toggler_angle_zero))
        Toggler_working_time = 0;
    else
        Toggler_working_time++;

		
		if(shootflag==1)
		{
			if((error2 < Toggler_angle_zero)&&(error2 > -Toggler_angle_zero))
			{
				TimeCount_shoot++;
				if(TimeCount_shoot>7)
				{
					shootflag=0;
					mcu_data[2]=0;//RC_CtrlData.mouse.press_1;
					
				}
				
			}
		}
		
		pid_calc(&pid_Toggler_ang, moto_toggle.total_angle-moto_toggle.offset_angle, Toggler_ang_set);
    pid_calc(&pid_Toggler_spd, moto_toggle.speed_rpm, pid_Toggler_ang.pos_out);
		 if(Toggler_working_time > 30)
    {
      if(Toggler_ang_set - moto_toggle.total_angle > Toggler_angle_one)
			{
				Toggler_ang_set -= Toggler_angle_one;
			}		
			pid_Toggler_spd.pos_out=-5000;
			Toggler_working_time = 0;
		}
    //pid_calc(&pid_Toggler_spd, moto_toggle.speed_rpm, speedtest);
}








/*
float Toggler_ang_set = 0;

void Toggler_ctr(void)
{
	static int Toggler_working_time;	
	
	if((Toggler_ang_set - moto_toggle.total_angle < Toggler_angle_zero)&&(Toggler_ang_set - moto_toggle.total_angle > -Toggler_angle_zero))
		Toggler_working_time = 0;
	else
		Toggler_working_time++;
		
	if(Toggler_working_time > 30) 
	{
		Toggler_ang_set =moto_toggle.total_angle - Toggler_angle_one;
		Toggler_working_time = 0;
	}
	pid_calc(&pid_Toggler_ang, moto_toggle.total_angle, Toggler_ang_set);	
	pid_calc(&pid_Toggler_spd, moto_toggle.speed_rpm, pid_Toggler_ang.pos_out);
}
*/
