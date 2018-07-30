#ifndef __T_PROTOCOL_H
#define __T_PROTOCOL_H

#include "mytype.h"
#include "control.h"

#define Max_Power		80
#define Max_Volt		26
#define Current_Unit	20/16384

extern float Cmsetcurt[4];
extern signed char syb(float numb);
extern void Power_ctr(float setcurt1,float setcurt2,float setcurt3,float setcurt4,float powerfb,float curtfb );

#endif


