#include "m_remote.h"
#include "t_moto.h"
#include "t_remote.h"
#include "control.h"
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

/*--------------------------------------------------------遥控器数据解码--------------------------------------------------------------------------------*/

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
	
	RC_CtrlData.key.v=((int16_t)pData[14]);//|((int16_t)pData[]15<<8;
	
	setinputmode(&RC_CtrlData.rc);
	
	switch(getinputmode())
		
	{
		case REMOTE_INPUT:	  //随动模式	
													pidswitch_flag[NOW]=1;
													remotectlprocess(&RC_CtrlData.rc);
													ctrcmallrun(rc);
													ctrgy(rc);
													ctrcm(rc);
													break;//控制云台模式
		case KEY_MOUSE_INPUT:	
													//分离模式
													pidswitch_flag[NOW]=2;
													remotectlprocess(&RC_CtrlData.rc);
													//ctrcmallrun(rc);
													ctrgy(rc);
													ctrcmsqin(rc);
													ctrcm(rc);
													break;//控制底盘盘模式
													
		
		case AUTUAIM:					
//												  remotectlprocess(&RC_CtrlData.rc);
//													if(en_flagof2312 == 1)
//													{
//															
//															switch(runof2312_s2)
//															{
//																	case 0:
//																				if( RC_CtrlData.rc.s1 == 2 )
//																					runof2312_s2=1;
//																				break;
//																	case 1:
//																				if( RC_CtrlData.rc.s1 == 2)
//																					runof2312_s2=2;
//																				else
//																					runof2312_s2=0;
//																				break;
//																	case 2:
//																				if(RC_CtrlData.rc.s1 != 2)
//																				{
//																					runof2312_s2=0;
//																					runof2312_mode++;
//																					if(runof2312_mode==1)
//																						runof2312_mode=0;
//																					switch(runof2312_mode)
//																					{
//																						case 0:	setrunof2312[NOWOF2312]=1;break;

//																						case 1:setrunof2312[NOWOF2312]=0;break;
//																						default:break;
//																					}
//																				}
//																				break;
//																	default:break;			
//															}
//															
//															
//													}
//													if(setrunof2312[NOWOF2312]==1)
//													{
//														TIM2->CCR1 = 1600;
//														TIM2->CCR2 = 1600;
//													}	
//													else if(setrunof2312[NOWOF2312]==0)
//													{
//														TIM2->CCR1 = 1000;
//														TIM2->CCR2 = 1000;
//													}
													break;
	}			
}


