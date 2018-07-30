#include  "m_toggle.h"
#include "t_remote.h"
#include "pid.h"


int Toggler_ang_set = 0;
float Toggler_spd_set=0;

void Toggler_control(void)
{
	static int Toggler_working_time;	
	
	if((Toggler_ang_set - moto_toggle.total_angle < Toggler_angle_zero)&&(Toggler_ang_set - moto_toggle.total_angle > -Toggler_angle_zero))
		Toggler_working_time = 0;
	else
		Toggler_working_time++;
		
	if(Toggler_working_time > 30) 
	{
		Toggler_ang_set -= Toggler_angle_one;
		Toggler_working_time = 0;
	}

	
	pid_calc(&pid_Toggler_ang, moto_toggle.total_angle, Toggler_ang_set);	
	pid_calc(&pid_Toggler_spd, moto_toggle.speed_rpm, pid_Toggler_ang.pos_out);

}


