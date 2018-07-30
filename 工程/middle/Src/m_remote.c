#include "m_remote.h"
#include "t_moto.h"
#include "t_remote.h"
#include "control.h"
#include "egg_task.h"

int limitswtch=0;

#define RC_CH_VALUE_MIN      ((uint16_t)364)
#define RC_CH_VALUE_OFFSET   ((uint16_t)1024)
#define RC_CH_VALUE_         ((uint16_t)1684)


/***********************************RC Switch Definition***************/
#define RC_SW_UP               ((uint16_t)1)
#define RC_SW_MID              ((uint16_t)3)
#define RC_SW_DOWN             ((uint16_t)2)

/***********************************PC Key Definiton*******************/
#define KEY_PRESSED_OFFSET_W       ((uint16_t)0x01<<0)
#define KEY_PRESSED_OFFSET_S       ((uint16_t)0x01<<1)
#define KEY_PRESSED_OFFSET_A       ((uint16_t)0x01<<2)
#define KEY_PRESSED_OFFSET_D       ((uint16_t)0x01<<3)
#define KEY_PRESSED_OFFSET_Q       ((uint16_t)0x01<<4)
#define KEY_PRESSED_OFFSET_E       ((uint16_t)0x01<<5)
#define KEY_PRESSED_OFFSET_SHIFT   ((uint16_t)0x01<<6)
#define KEY_PRESSED_OFFSET_CTRL    ((uint16_t)0x01<<7)

volatile unsigned char sbus_rx_buffer[2][RC_FRAME_LENGTH];
unsigned short pidswitch_flag[3]={1,1,1};
//static RC_Ctl_t RC_CtrlData;
RC_Ctl_t RC_CtrlData;

extern void Set_CM_ySpeed(CAN_HandleTypeDef* hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4);
float PID_Calc(pid_t* pid, float get, float set);
bool setrunof2312[2];
extern void mousekeycontrolprocess(Mouse *mouse, Key *key);
extern void remotectlprocess(Remote *rc);
extern void stopprocess(void);

/*--------------------------------------------------------Ò£¿ØÆ÷Êý¾Ý½âÂë--------------------------------------------------------------------------------*/

void RemoteDataProcess(uint8_t *pData,Remote *rc)
{
	
//	static uint8_t runof2312_s2;
		//static uint8_t runof2312_mode;

    if(pData==NULL)
	{
	   return;
	}
    RC_CtrlData.rc.ch0=((int16_t)pData[0]|((int16_t)pData[1]<<8))&0x07FF;
	RC_CtrlData.rc.ch1=(((int16_t)pData[1]>>3)|((int16_t)pData[2]<<5))&0x07FF;
	RC_CtrlData.rc.ch2=(((int16_t)pData[2]>>6)|((int16_t)pData[3]<<2)|((int16_t)pData[4]<<10))&0x07FF;
	RC_CtrlData.rc.ch3=(((int16_t)pData[4]>>1)|((int16_t)pData[5]<<7))&0x07FF;
  
	RC_CtrlData.rc.s1=((pData[5]>>4)&0x000c)>>2;
    RC_CtrlData.rc.s2=((pData[5]>>4)&0x0003);
	
	RC_CtrlData.mouse.x=((int16_t)pData[6])|((int16_t)pData[7]<<8);
	RC_CtrlData.mouse.y=((int16_t)pData[8])|((int16_t)pData[9]<<8);
    RC_CtrlData.mouse.z=((int16_t)pData[10])|((int16_t)pData[11]<<8);

	RC_CtrlData.mouse.press_1=pData[12];
	RC_CtrlData.mouse.press_2=pData[13];
	
	RC_CtrlData.key.v=((int16_t)pData[14])|((int16_t)pData[15]<<8);
	
	setinputmode(&RC_CtrlData.rc);
	
	cyl_spd=-(rc->ch1-1024.0)/660.0*3000.0;  
	if(limitswtch)
	{
		if(cyl_spd<0)
			cyl_spd=0;
	}
	ctrcmsqin(&RC_CtrlData.rc); 
	ctrcmallrun(rc);
	ctrcm(rc);
}


