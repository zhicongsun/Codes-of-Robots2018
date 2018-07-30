#include  "m_toggle.h"
#include "t_remote.h"
#include "pid.h"



float Toggler_ang_set = 0;

float Toggler_spd_set=0;
void Toggler_spd_ctr(void)
{
//	if( shootflag == 1)
//	{
//			Toggler_ang_set-=2048;
////			if(Toggler_ang_set < 0)
////				Toggler_ang_set += 8192;	
//			shootflag=0;//点击一次旋转一次
//	}
//	if( shootflag == 2)
//	{
//			Toggler_ang_set+=2048;
////			if(Toggler_ang_set>8191)
////				Toggler_ang_set-=8191;
//			shootflag=0;
//	}
	pid_calc_toggle_ang(&moto_toggle, &pid_Toggler_ang, &pid_Toggler_spd, Toggler_ang_set*36);
}
