#ifndef __M_PROTOCOL_H
#define __M_PROTOCOL_H

/**裁判系统通信协议**/
#include "usart.h"
#include "mytype.h"

/********************  结束更改  ***************************/
//0-3 bits
#define Strike_0   0x00 //0号装甲面（前）
#define Strike_1   0x01 //1号装甲面（左）
#define Strike_2   0x02 //2号装甲面（后）
#define Strike_3   0x03 //3号装甲面（右）
#define Strike_4   0x04 //4号装甲面（上1）
#define Strike_5   0x05 //5号装甲面（上2）
//4-7 bits
#define BC_Strike          0x00 //装甲伤害（受到攻击） BloodChanged    BC
#define BC_ModuleOffline   0x01 //模块离线扣血
#define BC_ShootSpeed      0x02 //子弹超速扣血
#define BC_ShootFreq       0x03 //子弹超频扣血
#define BC_ShootQLimit     0x04 //枪口热量超限
#define BC_CmPLimit        0x05 //地盘功率超限

#define BC_CommonFoul      0x06 //普通犯规扣血
#define BC_Tarmac          0x0a //停机坪加血
#define BC_EngineerOuto    0x0b //工程机器人自动回血


//比赛机器人状态（0x0001）
typedef __packed struct
{
	uint8_t validFlag;//位置角度信息有效标志位
	float x;
	float y;
	float z;
	float yaw;//枪口朝向角度值
}position_t;

typedef __packed struct
{
	uint16_t stageRemianTime;//当前剩余的时间
	uint8_t  gameProgress; //当前比赛阶段
	uint8_t  robotLevel;//机器人等级
	uint16_t remainHP;//机器人当前血量
	uint16_t maxHP;//机器人满血量
  position_t position;
}extGameRobotState_t;

typedef union
{
  extGameRobotState_t extGameRobotState;
  uint8_t extGameRobotState_recvbuff[34];
}extGameRobotState_recvtype;

//伤害数据（0x0002）
typedef __packed struct
{
	uint8_t armorType:4;//该种形式出现于结构体或共用体的定义中，是位域定义的标准形式。当armorType的位数超过4位是会被截断。
	uint8_t hurtType:4;
}exRobotHurt_t;

//实时射击信息（0x0003）

typedef __packed struct
{
	uint8_t bulletType;//弹丸类型
	uint8_t bulletFreq;//弹丸射频
	float bulletSpeed;//弹丸射速
	float reseved;    //保留
}extShootData_t;

//实时功率热量数据（0x004）
typedef __packed struct
{
	  float chassisVolt;//底盘输出电压
	  float chassisCurrent;//底盘输出电流
	  float chassisPower;//底盘输出功率
	  uint16_t shooterHeat0;//17mm枪口热量
	  uint16_t shooterHeat1;//42mm枪口热量
}extPowerHeatData_t;

typedef union
{
    extPowerHeatData_t extPowerHeatData;//弹丸射速
	  uint8_t extPowerHeatData_recvbuff[20];
}extPowerHeatData_recvtype;

//场地交互数据（0x0005）
typedef __packed struct
{
	uint8_t cardType;//卡类型
	uint8_t cardldx;//卡索引号，可以区分不同区域
}extRifddDetect_t;

//场地胜负数据（0x006）
typedef __packed struct
{
	uint8_t winner;//胜者
}extGameResult_t;

//Buff获取数据（0x0007）
typedef __packed struct
{
	uint8_t buffType;//Buff类型：攻击加成、防御加成、获得大能量机关
	uint8_t buffAddition;//加成百分比，以%为单位
}extGetBuff_t;

//参赛队自定义数据
typedef __packed struct
{
	float data1;
	float data2;
	float data3;
	uint8_t mask;
}extShowData_t;



typedef __packed struct
{
	uint8_t SOF;          //数据起始字节，固定为0xA5          
	uint16_t DataLength;  //数据长度
	uint8_t Seq;          //包序号
	uint8_t CRC8;         //帧头CRC校验
}FrameHeader_t;//帧头

typedef enum //这个是按顺序来的
{
	extGameRobotState = 0x0001,   
	exRobotHurt, 
	extShootData,            
	extPowerHeatData, 
	extRifddDetect,
	extGameResult,
	extGetBuff,
	extShowData = 0x0100
}CmdID_t;

typedef __packed struct
{
	FrameHeader_t    FrameHeader;
	CmdID_t          CmdID;
	__packed union 
	{
		extGameRobotState_t       extGameRobotState;        //比赛进程信息
		extPowerHeatData_t        extPowerHeatData;       //实时电流电压等
		extShootData_t            extShootData;       //实时射击信息 (0×0003)
		exRobotHurt_t             exRobotHurt;//实时扣血信息
		extRifddDetect_t          extRifddDetect;
		extGameResult_t           extGameResult;
		extGetBuff_t              extGetBuff;
		extShowData_t             extShowData; //自定义数据 
	}Data;
	uint16_t        CRC16;         //之前所有数据CRC校验   注意此数据和之前的数据可能不连续，所以不要直接使用，若需要直接使用，必须在此赋值
}Frame_t;  //数据帧


extern Frame_t* FrameData;//定义帧数据指针用来保存数据信息

extern int DataAnalysis(uint8_t *pData);

#endif




