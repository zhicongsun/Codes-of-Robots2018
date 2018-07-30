#ifndef __T_PROTOCOL_H
#define __T_PROTOCOL_H

#include "mytype.h"
#include "control.h"

#define Max_Power		80		//80
#define Limit_Power 75
#define Max_Volt		26
#define Current_Unit	20/16384
extern double k_chassis3508_power;
extern void Power_ctr(void);

#endif


