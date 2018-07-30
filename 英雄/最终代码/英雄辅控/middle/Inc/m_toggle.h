#ifndef __M_TOGGLE_H
#define __M_TOGGLE_H

extern float Toggler_spd_set;
extern int Toggler_ang_set;
extern void Toggler_control(void);

#define Toggler_angle_zero 	5*8192/360*36
#define Toggler_angle_half 	36*8192/360*36
#define Toggler_angle_one 	72*8192/360*36
#define Toggler_angle_two 	144*8192/360*36

#endif

