#ifndef __T_REMOTE_H
#define __T_REMOTE_H
#include "usart.h"
#include "control.h"
#include "m_remote.h"
#include "t_tk1.h"

#define GO        0
#define BACK      1
#define LEFT      2
#define RIGHT     3
#define TURNLEFT  4
#define TURNRIGHT 5

#define PITMAX   30.0f
#define PITMIN  -20.0f
#define YAWMAX   90.0f
#define YAWMIN  -90.0f

#define MKMAXSPD 6000
#define MKMINSPD 3000
typedef enum
{
	REMOTE_INPUT = 1,
	KEY_MOUSE_INPUT = 2,
	AUTUAIM = 3,
}InputMode_e;
typedef enum
{
	SHOOT = 1,
	STOPSHOOT = 2,
	UNSHOOT=3
}LeftsideMode_e;
typedef enum
{
	MKNORMAL=1,
	MKUPDOWN=2,
	MKEGG=3
}MouseKeyMode_e;
typedef enum
{
	MKFOLLOW=1,
	MKSEPARATE=2,
	EGG=3
}MouseKeyNormode_e;
typedef  __packed struct{
        u8 W;
        u8 S;
				u8 A;
				u8 D;
				u8 Q;
				u8 E;
				u8 shift;
				u8 ctrl;
	      u8 R;
        u8 F;
				u8 G;
				u8 Z;
				u8 X;
				u8 C;
				u8 V;
				u8 B;
}KEY;

extern uint8_t shootflag;
extern bool usetogger;
extern int Rub2323_spd;
extern unsigned short follow_flag;
extern short angorspd;
extern KEY KeyBoard;

InputMode_e getinputmode(void);
void setinputmode(Remote *rc);
void setleftsidemode(Remote *rc);
void remotectlprocess(Remote *rc);
void readkey(Key *key);
void ctrcm(Remote *rc);      //底盘运动合成
void ctrcmsqin(void);
void ctrcmallrun(Remote *rc);//底盘全向移动
void ctrgy(Remote *rc);      //云台运动
void autuaim(Remote *rc);

void mknormal(Mouse *mouse, Key *key);
void mkallrun(KEY* keyboard);//键盘控制全方位移动

void mouse_ctrl(Mouse*mouse);

void mkfollow(KEY* keyboard,Mouse*mouse);
void mkseparate(KEY* keyboard,Mouse*mouse);
void mkegg(void);

#endif
