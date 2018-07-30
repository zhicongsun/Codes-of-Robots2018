#include "t_protocol.h"
#include "DJ_Protocol.h"
#include "m_toggle.h"
#include "t_moto.h"
#include "pid.h"
#include "control.h"

float Cmsetcurt[4];

float imgpower;
float set_totalcurt;
#define FABS(x) (x>0?x:-x)  
signed char syb(float numb)
{
	if(numb>0)
		return	1;
	else
		return	-1;
}
extern float var[];
void Power_ctr(float setcurt1,float setcurt2,float setcurt3,float setcurt4,float powerfb,float curtfb )
{
	set_totalcurt=(FABS(setcurt1)+FABS(setcurt2)+FABS(setcurt3)+FABS(setcurt4))*Current_Unit;
	imgpower=Max_Volt*set_totalcurt;
	//var[6]= set_totalcurt;
	if((imgpower<Max_Power)&&(powerfb<Max_Power))
	{
			Cmsetcurt[0] = setcurt1;
			Cmsetcurt[1] = setcurt2;
			Cmsetcurt[2] = setcurt3;
			Cmsetcurt[3] = setcurt4;				
	}
	else
	{
			Cmsetcurt[0] = Max_Power/Max_Volt*setcurt1/set_totalcurt;
			Cmsetcurt[1] = Max_Power/Max_Volt*setcurt2/set_totalcurt;
			Cmsetcurt[2] = Max_Power/Max_Volt*setcurt3/set_totalcurt;
			Cmsetcurt[3] = Max_Power/Max_Volt*setcurt4/set_totalcurt;
	}
}

