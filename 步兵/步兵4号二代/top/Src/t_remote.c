#include "t_remote.h"
#include "t_moto.h"
#include "m_toggle.h"
#include "t_toggle.h"
#include "control.h"
#include "DJ_Protocol.h"
#include "main.h"

extern float Cm3508_spd_xset[4];//x方向分量速度
extern float Cm3508_spd_yset[4];//y方向分量速度
extern float Cm3508_spd_set[4];//合成速度
extern float Cm3505_spd_sqin[4];//自旋运动
extern float Cm3505_spd_sqin[4];//自旋运动

extern float Gm6623_pit_ang_set;
extern float Gm6623_pit_ang_offset;
extern float Gm6623_pit_spd_set;
extern float Gm6623_yaw_ang_set;
extern float Gm6623_yaw_spd_set;

extern short int Tk1pointang[3];
extern u16 USART_RX_STA;       //接收状态标记

	
InputMode_e inputmode = REMOTE_INPUT;   //输入模式设定,默认为遥控器模式
LeftsideMode_e leftsidemode=STOPSHOOT;    //左摇杆模式设定，默认为自旋运动
MouseKeyMode_e mousekeymode=MKNORMAL;     //键鼠模式选择，默认为控制地盘全方位移动
MouseKeyNormode_e mousekeynormode=MKFOLLOW;

unsigned short follow_flag;
int Rub2323_spd=1500;
float yaw_tk1_angle=4650;
float mousekey_spd[6]={3500,3500,3500,3500,3500,3500};
KEY KeyBoard;
unsigned short shiftcount=0;
unsigned short press2count=0;
uint8_t shootflag=0;
bool usetogger=0;
bool usedtogger=0;
//输入模式设置 
void setinputmode(Remote *rc)
{
	if(rc->s2 == 1)
	{
		inputmode = REMOTE_INPUT;
	}
	else if(rc->s2 == 3)
	{
		inputmode = KEY_MOUSE_INPUT;
	}
	else if(rc->s2 == 2)
	{
		inputmode = KEY_MOUSE_INPUT;
	}	
}
InputMode_e getinputmode(void)
{
	return inputmode;
}
//左边摇杆模式设置 
void setleftsidemode(Remote *rc)
{
	if(rc->s1 == 1)
	{
		leftsidemode = SHOOT;
	}
	else if(rc->s1 == 3)
	{
		leftsidemode = STOPSHOOT;
	}
	else if(rc->s1 == 2)
	{
		leftsidemode=UNSHOOT;
	}
}
LeftsideMode_e getlsmode(void)
{
   return leftsidemode;
}

void ctrcm(Remote *rc)
{ 
	 //底盘合成运动
	 Cm3508_spd_set[0]=Cm3508_spd_xset[0]+Cm3508_spd_yset[0]+Cm3505_spd_sqin[0];
	 Cm3508_spd_set[1]=Cm3508_spd_xset[1]+Cm3508_spd_yset[1]+Cm3505_spd_sqin[1];
	 Cm3508_spd_set[2]=Cm3508_spd_xset[2]+Cm3508_spd_yset[2]+Cm3505_spd_sqin[2];
	 Cm3508_spd_set[3]=Cm3508_spd_xset[3]+Cm3508_spd_yset[3]+Cm3505_spd_sqin[3];
}
void ctrcmsqin(void)
{

	 if(stateof_selfsun==1) 
	 {
		 Cm3505_spd_sqin[0]=1800;
		 Cm3505_spd_sqin[1]=1800;
		 Cm3505_spd_sqin[2]=1800;
		 Cm3505_spd_sqin[3]=1800;
	 }
	 else if(stateof_selfsun==2)
	 {
		 Cm3505_spd_sqin[0]=-1800;
		 Cm3505_spd_sqin[1]=-1800;
		 Cm3505_spd_sqin[2]=-1800;
		 Cm3505_spd_sqin[3]=-1800;
	 }
		
	 
	 if(Cm3505_spd_sqin[0]>=-3000)
	 {
		 Cm3505_spd_sqin[0]+=100;
		 Cm3505_spd_sqin[1]+=100;
		 Cm3505_spd_sqin[2]+=100;
		 Cm3505_spd_sqin[3]+=100;
	 }
	 if(Cm3505_spd_sqin[0]<=3000)
	 {
		 Cm3505_spd_sqin[0]-=100;
		 Cm3505_spd_sqin[1]-=100;
		 Cm3505_spd_sqin[2]-=100;
		 Cm3505_spd_sqin[3]-=100;
	 }
	 
	 //角度限制
	 err = moto_gym[0].angle - yaw6623_MiddleAngle;
	 err = err>0 ? err : -err;
	
	 if(err > yaw6623_AngleAdjustLimit)
	 {
			Cm3505_spd_sqin[0]=0;
			Cm3505_spd_sqin[1]=0;
			Cm3505_spd_sqin[2]=0;
			Cm3505_spd_sqin[3]=0;
	 }
	
}

void ctrcmallrun(Remote *rc)
{  //全方位运动
   Cm3508_spd_yset[0]=(rc->ch3-1024.0)/660.0*3000.0;   //这里的3000为可变参数，用于设定遥控器满油门电流值，幅值为16384
	 Cm3508_spd_yset[1]=-((rc->ch3-1024.0)/660.0*3000.0);//注意这里的数为浮点数，否则遥控出错
	 Cm3508_spd_yset[2]=(rc->ch3-1024.0)/660.0*3000.0;
	 Cm3508_spd_yset[3]=-((rc->ch3-1024.0)/660.0*3000.0);

	 Cm3508_spd_xset[0]=(rc->ch2-1024.0)/660.0*3000.0;
	 Cm3508_spd_xset[1]=((rc->ch2-1024.0)/660.0*3000.0);
	 Cm3508_spd_xset[2]=-(rc->ch2-1024.0)/660.0*3000.0;
	 Cm3508_spd_xset[3]=-((rc->ch2-1024.0)/660.0*3000.0);
}


//控制yaw速度环
void ctrgy(Remote *rc)
{
  // Gm6623_pit_spd_set=-(rc->ch1-1024.0)/660.0*350.0;  
	Gm6623_pit_spd_set=-(rc->ch1-1024.0)/660.0*350.0/2;//为了让pitch轴灵敏度降低
	 Gm6623_yaw_spd_set=-(rc->ch0-1024.0)/660.0*350.0;//乘500时候满油门为360度/秒
	
	 Cm3505_spd_sqin[0]=0;
	 Cm3505_spd_sqin[1]=0;
	 Cm3505_spd_sqin[2]=0;
	 Cm3505_spd_sqin[3]=0;
	
	//角度限制
	 err = moto_gym[0].angle - yaw6623_MiddleAngle;
	 err = err>0 ? err : -err;
	
	 if(err > yaw6623_AngleAdjustLimit)
	 {
			Gm6623_yaw_spd_set=0;
	 }
}

//遥控模式
void remotectlprocess(Remote *rc)
{
	// modeswitch_flag[NOW]=1;
	 //modeswitch_flag[LAST]=1;
	 setleftsidemode(rc);
	 switch(getlsmode())
	 {
		 case SHOOT:
										usetogger=1;
										if(usedtogger==1)
										{
											Toggler_ang_set+=4*1600;
											usedtogger=0;
										}
		 
										break;
		 case STOPSHOOT:  
										usetogger=0;
										usedtogger=1;
										Toggler_ang_set=Toggler_ang_set;
										break;
		 
		 case UNSHOOT:      
										usetogger=1;
										if(usedtogger==1)
										{
											Toggler_ang_set-=1600;
											usedtogger=0;
										}
										break;

		 default:
										Toggler_ang_set=Toggler_ang_set;
										break;
	 }
}

MouseKeyMode_e getmousekeymode(void)
{
   return mousekeymode;
}

MouseKeyNormode_e getmknormode(void)
{
   return mousekeynormode;
}

//自动瞄准模式一（有标志位autuaim_flag,正式用）
void autuaim(Remote *rc)
{ 	
	modeswitch_flag[NOW]=2;
	modeswitch_flag[LAST]=2;
	//若串口接收到tk1更新的数据，在Tk1dataprocess()中将autuaim_flag置1，Tk1pointang起作用,否则清零
	autu_Gm6623_yaw_ang_set= Tk1pointang[angx]/100*8191/360;	
	Gm6623_pit_ang_set= 2700;	
	//Gm6623_pit_ang_set= moto_gym[1].angle-Tk1pointang[angy]/100*8191/360*autuaim_flag-Gm6623_pit_ang_offset;	

	if(USART_RX_STA&0x8000)//这里和tk1配合
		USART_RX_STA=0; 
	ctrcmallrun(rc);
	//ctrcmsqin(rc);
	ctrcm(rc);
	//	//摩擦轮速度设置
	
}

void readkey(Key *key)
{  
   if((key->v&0x0001)!=0) KeyBoard.W=1; else KeyBoard.W=0;
   if((key->v&0x0002)!=0) KeyBoard.S=1; else KeyBoard.S=0;
   if((key->v&0x0004)!=0) KeyBoard.A=1; else KeyBoard.A=0;
   if((key->v&0x0008)!=0) KeyBoard.D=1; else KeyBoard.D=0;
   if((key->v&0x0010)!=0) KeyBoard.shift=1; else KeyBoard.shift=0;
   if((key->v&0x0020)!=0) KeyBoard.ctrl=1;  else KeyBoard.ctrl=0;
	 if((key->v&0x0040)!=0) KeyBoard.Q=1; else KeyBoard.Q=0;
   if((key->v&0x0080)!=0) KeyBoard.E=1; else KeyBoard.E=0;
	 if((key->v&0x0100)!=0) KeyBoard.R=1; else KeyBoard.R=0;
   if((key->v&0x0200)!=0) KeyBoard.F=1; else KeyBoard.F=0;
   if((key->v&0x0400)!=0) KeyBoard.G=1; else KeyBoard.G=0;
   if((key->v&0x0800)!=0) KeyBoard.Z=1; else KeyBoard.Z=0;
   if((key->v&0x1000)!=0) KeyBoard.X=1; else KeyBoard.X=0;
   if((key->v&0x2000)!=0) KeyBoard.C=1; else KeyBoard.C=0;
   if((key->v&0x4000)!=0) KeyBoard.V=1; else KeyBoard.V=0;
   if((key->v&0x8000)!=0) KeyBoard.B=1; else KeyBoard.B=0;
};
void readmouse(Mouse*mouse)
{
}

void spd_updown(void)
{
	static int mkkey_staup=0,mkmode_staup=1;
	int i;
	switch(mkkey_staup)
	{
		case 0:	//no key
					if( 1 == KeyBoard.Z )
						mkkey_staup = 1;
					break;
		case 1: //key down wait release.
					if( 1 == KeyBoard.Z )
					{
						mkkey_staup = 2;
					}
					else
					{
						mkkey_staup = 0;
					}
					break;
		case 2: 
					if( 0 == KeyBoard.Z )
					{
						mkkey_staup = 0;
						if(mousekey_spd[GO]<5500)
						{
							for(i=0;i<5;i++)
								mousekey_spd[i] +=200;
						}
					}
					break;
					
		default:									
					if( 1 == KeyBoard.Z )
						mkkey_staup = 1;
					break;
	}
	
	static int mkkey_stadown=0,mkmode_stadown=1;
	switch(mkkey_stadown)
	{
		case 0:	//no key
					if( 1 == KeyBoard.X )
						mkkey_stadown = 1;
					break;
		case 1: //key down wait release.
					if( 1 == KeyBoard.X )
					{
						mkkey_stadown = 2;
					}
					else
					{
						mkkey_stadown = 0;
					}
					break;
		case 2: 
					if( 0 == KeyBoard.X )
					{
						mkkey_stadown = 0;
						if(mousekey_spd[GO]>3000)
						{
							for(i=0;i<5;i++)
								mousekey_spd[i] -=200;
						}
					}
					break;
					
		default:									
					if( 1 == KeyBoard.X )
						mkkey_stadown = 1;
					break;
	}


}
/****************************************
功能：键盘控制底盘全方位运动
日期：2018.4.1
注意：数据类型非常重要
****************************************/
void mkallrun(KEY* keyboard)
{
	float mk_rl_spd;
	short rl_flag[2];
	
	spd_updown();
	if(( keyboard->A )&&( !keyboard->D ))
	{
		rl_flag[0]=-1;
		rl_flag[1]= 1;
		mk_rl_spd=mousekey_spd[TURNLEFT];
	}
	else if(( keyboard->D )&&( !keyboard->A ))
	{
		rl_flag[0]= 1;
		rl_flag[1]=-1;
		mk_rl_spd=mousekey_spd[TURNRIGHT];
	}
	else 
	{
		rl_flag[0]=rl_flag[1]=0;
		mk_rl_spd=0;
	}
	Cm3508_spd_xset[0] = mk_rl_spd * rl_flag[0];
	Cm3508_spd_xset[1] = mk_rl_spd * rl_flag[0];
	Cm3508_spd_xset[2] = mk_rl_spd * rl_flag[1];
	Cm3508_spd_xset[3] = mk_rl_spd * rl_flag[1];

	float mk_ud_spd;
	short ud_flag[2];
	if(( keyboard->W )&&( !keyboard->S ))
	{
		ud_flag[0]= 1;
		ud_flag[1]=-1;
		mk_ud_spd=mousekey_spd[GO];
	}
	else if(( keyboard->S )&&( !keyboard->W ))
	{
		ud_flag[0]=-1;
		ud_flag[1]= 1;
		mk_ud_spd=mousekey_spd[BACK];
	}
	else 
	{
		ud_flag[0]=ud_flag[1]=0;
		mk_ud_spd=0;
	}
	Cm3508_spd_yset[0] = mk_ud_spd * ud_flag[0];
	Cm3508_spd_yset[1] = mk_ud_spd * ud_flag[1];
	Cm3508_spd_yset[2] = mk_ud_spd * ud_flag[0];
	Cm3508_spd_yset[3] = mk_ud_spd * ud_flag[1];
}

void mktotalrun(void)
{
	//合成运动
	Cm3508_spd_set[0]=Cm3508_spd_xset[0]+Cm3508_spd_yset[0]+Cm3505_spd_sqin[0];
	Cm3508_spd_set[1]=Cm3508_spd_xset[1]+Cm3508_spd_yset[1]+Cm3505_spd_sqin[1];
	Cm3508_spd_set[2]=Cm3508_spd_xset[2]+Cm3508_spd_yset[2]+Cm3505_spd_sqin[2];
	Cm3508_spd_set[3]=Cm3508_spd_xset[3]+Cm3508_spd_yset[3]+Cm3505_spd_sqin[3];
}
//射击函数
void shoot(Mouse*mouse)
{
	static uint8_t shootkey_sta=1;
	switch(shootkey_sta)
	{
		case 1:
					if( mouse->press_1 == 1 )
						shootkey_sta=2;
					break;
		case 2:
					if( mouse->press_1 == 1)
						shootkey_sta=3;
					else
						shootkey_sta=1;
					break;
		case 3:
					if( mouse->press_1 == 0)
					{
						shootkey_sta=1;
						Toggler_ang_set += shootnum*Toggler_angle_one;
						shootflag=1;//发射标志位
					}
					break;
		default:break;			
	}
	
	static uint8_t shootback_sta=1;
	switch(shootback_sta)
	{
		case 1:
					if( KeyBoard.C )
						shootback_sta=2;
					break;
		case 2:
					if( KeyBoard.C )
						shootback_sta=3;
					else
						shootback_sta=1;
					break;
		case 3:
					if( !KeyBoard.C )
					{
						shootback_sta=1;
						Toggler_ang_set -= 2*Toggler_angle_zero;
					}
					break;
		default:break;			
	}	
} 


//键鼠普通模式一：云台底盘跟随模式
void mkfollow(KEY* keyboard,Mouse*mouse)
{  	
	
	if(mouse->press_2)
	{
		modeswitch_flag[NOW]=4;
		follow_flag=1;	
		Cm3505_spd_sqin[0]=0;
		Cm3505_spd_sqin[1]=0;
		Cm3505_spd_sqin[2]=0;
		Cm3505_spd_sqin[3]=0;
		//follow_flag=0;	
		//ctrcmsqin();
	}
	else
	{
		modeswitch_flag[NOW]=3;
		if(KeyBoard.G)
		{
			modeswitch_flag[NOW]=3;
			follow_flag=0;
			ctrcmsqin();
		}
		else
		{
			follow_flag=1;	
			Cm3505_spd_sqin[0]=0;
			Cm3505_spd_sqin[1]=0;
			Cm3505_spd_sqin[2]=0;
			Cm3505_spd_sqin[3]=0;
		}

	}
   
	 mkallrun(&KeyBoard);
	 mktotalrun();
	 
}

//键鼠普通模式二：云台底盘分离
void mkseparate(KEY* keyboard,Mouse*mouse)
{
		
	 modeswitch_flag[NOW]=3;
	 follow_flag=0;
	
	 float mk_spin_spd;
	 float spin_flag;
 
	// mouse_ctrl(mouse);//鼠标控制云台的函数
			
	if(( keyboard->Q )&&( !keyboard->E ))
	{
		spin_flag= 1.0;
		mk_spin_spd=mousekey_spd[LEFT];
	}
	else if(( keyboard->E )&&( !keyboard->Q ))
	{
		spin_flag=-1.0;
		mk_spin_spd=mousekey_spd[RIGHT];
	}
	else 
	{
		spin_flag=0;
		mk_spin_spd=0;
	}
	Cm3505_spd_sqin[0] = mk_spin_spd * spin_flag;
	Cm3505_spd_sqin[1] = mk_spin_spd * spin_flag;
	Cm3505_spd_sqin[2] = mk_spin_spd * spin_flag;
	Cm3505_spd_sqin[3] = mk_spin_spd * spin_flag;
	
	mkallrun(&KeyBoard);
	mktotalrun();
}

//普通模式三：自动瞄准模式
void mkautu()
{
}

//键鼠模式一：普通模式（默认模式）
void mknormal(Mouse *mouse, Key *key)
{
	mkfollow(&KeyBoard,&RC_CtrlData.mouse);
//		static int normode_sta;

//		if     ( 1 == KeyBoard.Z )
//						normode_sta = 1;
//		else if( 1 == KeyBoard.X )
//						normode_sta = 2;
//		else if( 1 == KeyBoard.C )
//						normode_sta = 3;
//	
//	switch(normode_sta)
//	{
//		//跟随模式
//		case MKFOLLOW:	 mkfollow(&KeyBoard,&RC_CtrlData.mouse);
//										 break;
//		//分离模式
//		case MKSEPARATE: //mkseparate(&KeyBoard,&RC_CtrlData.mouse);
//										 break;
//		//自动瞄准模式
//		case MKAUTU:	   //mkautu();
//										 break;
//		
//		default:				 mkfollow(&KeyBoard,&RC_CtrlData.mouse);
//										 break;
//	}
}

//键鼠模式二：上下岛模式
void mkupdown(Key *key)
{
}
void mkegg(void)
{
}

//键鼠模式
void mousekeycontrolprocess(Mouse *mouse, Key *key)
{
	readkey(key);//读键盘值
	shoot(&RC_CtrlData.mouse);//射击动作选择
	mknormal(&RC_CtrlData.mouse,&RC_CtrlData.key);
//	static int mkkey_sta,mkmode_sta=1;
//	switch(mkkey_sta)
//	{
//		case 0:	//no key
//					if( 1 == KeyBoard.ctrl )
//						mkkey_sta = 1;
//					break;
//		case 1: //key down wait release.
//					if( 1 == KeyBoard.ctrl )
//					{
//						mkkey_sta = 2;
//					}
//					else
//					{
//						mkkey_sta = 0;
//					}
//					break;
//		case 2: 
//					if( 0 == KeyBoard.ctrl )
//					{
//						mkkey_sta = 0;
//						mkmode_sta++;
//						if( mkmode_sta == 4 )
//							mkmode_sta=1;	
//					}
//					break;
//					
//		default:									
//					if( 1 == KeyBoard.ctrl )
//						mkkey_sta = 1;
//					break;
//	}
//	switch(mkmode_sta)
//	{
//		//普通模式
//		case MKNORMAL:	mknormal(&RC_CtrlData.mouse,&RC_CtrlData.key);
//										break;
//		//上下岛模式
//		case MKUPDOWN:	mkupdown(&RC_CtrlData.key);
//										break;
//		//补弹模式
//		case MKEGG:     mkegg();
//										break;
//		
//		default:				mknormal(&RC_CtrlData.mouse,&RC_CtrlData.key);
//										break;
//	}
}

