#ifndef __T_TOGGLE_H
#define __T_TOGGLE_H
#include "stdint.h"

#define Infantry 0
#define Hero     1

#define Bullet17 0
#define Bullet42 1
extern short shootnum;
extern unsigned short toggle_heat_ctr(uint8_t rlgrade,float rlspd,uint8_t bulletfreq,uint8_t bullettype,uint16_t heat1);

#endif
