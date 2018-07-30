#ifndef __T_PROTOCOL_H
#define __T_PROTOCOL_H

#include "mytype.h"


#define POWER_MAX 80//英雄车是120，其他车不限制底盘功率
#define POWER_SET 75//

//float HEAT_MAX[3]={1600,3000,6000};//百思不得其解：为什么放这就会报错？？？？？？？？？？？？？？？？？？？
//float HEAT_SET[3]={1400,2800,5800};

#define SHOOT17_SPD_MAX 30//  30m/s  //英雄车有：#define SHOOT42_SPD_MAX 16.5//  16.5m/s

extern u8 level;
extern void PowerHeat_ctr(void);


#endif


