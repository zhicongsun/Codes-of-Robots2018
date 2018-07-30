#ifndef __EGGTASK_H
#define __EGGTASK_H

extern float cyl_ang;
extern float cyl_spd;
extern short egg_task_flag;
extern short egg_task_temp;
extern float tog_spd;

float qiut_sig(float num);
extern void egg_task(void);
extern short limitswt[3];
extern short limitflag[3];

#endif
