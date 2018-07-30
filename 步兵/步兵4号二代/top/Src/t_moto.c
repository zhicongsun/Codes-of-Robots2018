#include "t_moto.h"
#include "m_moto.h"
#include "m_imu.h"
#include "pid.h"
#include "m_remote.h"
#include "t_remote.h"
#include "t_tk1.h"
#include "t_protocol.h"
#include "DJ_Protocol.h"
extern short int Tk1pointang[3];
extern float angle_set;//调试用
extern float var[7];//调试用
float Cm3508_spd_get[4]= {0,0,0,0};
float Cm3508_spd_xset[4];//x方向分量速度
float Cm3508_spd_yset[4];//y方向分量速度
float Cm3505_spd_sqin[4];//自旋运动
float Cm3508_spd_set[4];//合成速度
float Cm3508_total_set[4];//总pid输出=底盘运动的pid输出+随动pid输出

float Gm6623_pit_ang_set=pit6623_MiddleAngle;
float Gm6623_pit_spd_set=0;
float Gm6623_pit_ang_offset=25;//克服重力影响的偏差角，机械一调整需要重新矫正

float Gm6623_yaw_ang_set=yaw6623_MiddleAngle;
float Gm6623_yaw_spd_set=0;
float Gm6623_yaw_ang_give=yaw6623_MiddleAngle;

//随动参数
Follow_6623_imu_fbangall_t follow_6623yaw_imu_data;
int yaw6623_AngleAdjust;
int yawimu_AngleAdjust;
int pitimu_AngleAdjust;

//自动瞄参数
Follow_6623_imu_fbangall_t autu_6623yaw_imu_data;
float autu_Gm6623_yaw_ang_set=0;
int autu_yaw6623_AngleAdjust;
int autu_yawimu_AngleAdjust;
float autu_Gm6623_pit_ang_set=0;
int autu_pit6623_AngleAdjust;
int autu_pitimu_AngleAdjust;
//限幅参数
int err;
int offset = yaw6623_MiddleAngle;
//键鼠变量
double km_nor_yaw_temang=0;
double km_nor_yaw_ang_fb=0;
double km_nor_yaw_spd_fb;
double km_nor_yaw_ang_set=yaw6623_MiddleAngle;
int32_t yaw_Nowangle = yaw6623_MiddleAngle;

double km_nor_pit_temang=0;
double km_nor_pit_ang_fb=0;
double km_nor_pit_spd_fb;
double km_nor_pit_ang_set=pit6623_MiddleAngle;
int32_t pit_Nowangle = pit6623_MiddleAngle;

double send_current_yaw;
double send_current_pit;

float a=0.02;
void cm_spd_current_ctr(pid_t *pid,float fbspeed, float setspeed)
{
    setspeed = a*setspeed + (1-a)*pid->spset;
    pid->spset = setspeed;
    pid_calc(pid, fbspeed, setspeed);
}
void cm_spd_ctr(void)
{
    for(int i=0; i<4; i++)
        pid_calc(&Cm3508_spd_pid[i], moto_chassis[i].speed_rpm, Cm3508_total_set[i]);

//	if(ReadData.cm_power<Limit_Power)
//	{
//		for(int i=0; i<4; i++)
//			pid_calc(&Cm3508_spd_pid[i], moto_chassis[i].speed_rpm, Cm3508_total_set[i]);
//	}
//	else if(ReadData.cm_power<Max_Power)
//	{
//		a = 0.03;
//		for(int i=0; i<4; i++)
//			cm_spd_current_ctr(&Cm3508_spd_pid[i], moto_chassis[i].speed_rpm, Cm3508_total_set[i]);
//	}
//	else
//	{
//		a = 0.01;
//		for(int i=0; i<4; i++)
//			cm_spd_current_ctr(&Cm3508_spd_pid[i], moto_chassis[i].speed_rpm, Cm3508_total_set[i]);
//	}
}

/**************************************/
//自动pitch
/**************************************/
/*
void km_au_pit_ctr(void)//相对位置式自动打击
{
    float C=0.35;
    int Nowangle;
    static float angle_pidout_temp=0;
    static double pit_auto_angle_set=pit6623_MiddleAngle;//对应中点
    float now_Tk1pointang=0;
    static float last_Tk1pointang=0;
    now_Tk1pointang=Tk1pointang[1];
    if(Tk1pointang[2]==0)
    {
        now_Tk1pointang=last_Tk1pointang;
    }
    Nowangle=moto_gym[1].angle;
    if(last_Tk1pointang!=now_Tk1pointang)
    {
        pit_auto_angle_set= Nowangle+(now_Tk1pointang-130)/100/360*8192/2;//单位换算比较奇妙
    }
    autu_pit6623_AngleAdjust 	= (imu_data.gx)/16.40;//单位为度/秒
    if(((pit_auto_angle_set - Nowangle)<10) && ((pit_auto_angle_set - Nowangle) > -10))
    {

        pid_calc(&Gm6623_pit_ang_pid_slow,Nowangle, pit_auto_angle_set );
        angle_pidout_temp = angle_pidout_temp*C + Gm6623_pit_ang_pid_slow.pos_out *(1-C);
        pid_calc(&Gm6623_pit_spd_pid,autu_pit6623_AngleAdjust,angle_pidout_temp);
    }
    else
    {

        pid_calc(&Gm6623_pit_ang_pid_quick,Nowangle, pit_auto_angle_set );
        angle_pidout_temp = angle_pidout_temp*C + Gm6623_pit_ang_pid_quick.pos_out *(1-C);
        pid_calc(&Gm6623_pit_spd_pid,autu_pit6623_AngleAdjust,angle_pidout_temp);
    }

    last_Tk1pointang=now_Tk1pointang;
}
*/




/**************************************/
//非自动pitch
/**************************************/
#define MAX_UP (4260-850)
#define MAX_DOWN (4260+240)
//低通滤波系数（0-1），越大滤波效果越好，但滞后越大
//#define C_Filter_speed 0.0
//#define C_Filter_angle 0.0
//#define SiQu 2

float C_Filter_speed_pitch = 0.2;
float C_Filter_angle_pitch = 0.5;
short SiQu_mouse_y=2;
float k_mouse_y=3.0;
short up=0;
short down=0;
float km_nor_pit_angle_set = pit6623_MiddleAngle;
float km_nor_pit_angle_set_temp = pit6623_MiddleAngle;

float km_nor_pit_spd_set = 0;

void km_nor_pit_ctr(Mouse*mouse)
{

    //static float km_nor_pit_angle_set = pit6623_MiddleAngle;
    //static float km_nor_pit_angle_set_temp = pit6623_MiddleAngle;
    //static float k=5;
    static float NowAngle=pit6623_MiddleAngle;
    km_nor_pit_spd_fb = (imu_data.gx)/16.40;//单位为度/秒  ,速度反馈值

    if(mouse->y < SiQu_mouse_y && mouse->y > (0-SiQu_mouse_y))
    {
        mouse->y = 0;
    }

    km_nor_pit_spd_set = (km_nor_pit_spd_set * C_Filter_speed_pitch) + k_mouse_y * mouse->y * (1 - C_Filter_speed_pitch);
    km_nor_pit_angle_set_temp = km_nor_pit_angle_set + km_nor_pit_spd_set;
    km_nor_pit_angle_set = km_nor_pit_angle_set * C_Filter_angle_pitch + km_nor_pit_angle_set_temp * (1 - C_Filter_angle_pitch);

    //上下角度限制，向上km_nor_pit_angle_set减小，
    if(km_nor_pit_angle_set < (MAX_UP-up))  km_nor_pit_angle_set = MAX_UP;
    if(km_nor_pit_angle_set > (MAX_DOWN+down))  km_nor_pit_angle_set = MAX_DOWN;

    pid_calc(&Gm6623_pit_ang_pid,moto_gym[1].angle,km_nor_pit_angle_set);
    pid_calc(&Gm6623_pit_spd_pid,km_nor_pit_spd_fb,Gm6623_pit_ang_pid.pos_out);


    /*
    if(mouse->y>2||mouse->y<-2)
    {
    	pid_calc(&Gm6623_pit_spd_pid,km_nor_pit_spd_fb,km_nor_pit_spd_set);
    	NowAngle=moto_gym[1].angle;
    	Gm6623_pit_ang_pid.pos_out=0;
    	Gm6623_pit_ang_pid.iout=0;
    }
    else
    {
    	pid_calc(&Gm6623_pit_ang_pid,moto_gym[1].angle,NowAngle);
    	pid_calc(&Gm6623_pit_spd_pid,km_nor_pit_spd_fb,Gm6623_pit_ang_pid.pos_out);
    }
    */
}


float auto_Piych_offset=-100;
float fact_shoot = 0.5;
void km_au_pit_ctr(void)//相对位置式自动打击
{
    float C=0.35;
    int Nowangle;
    static float angle_pidout_temp=0;
    static double pit_auto_angle_set=pit6623_MiddleAngle;//对应中点
    float now_Tk1pointang=0;
    static float last_Tk1pointang=0;
    now_Tk1pointang=Tk1pointang[1];
    if(Tk1pointang[2]==0)
    {
        now_Tk1pointang=last_Tk1pointang;
        km_nor_pit_ctr(&RC_CtrlData.mouse);
    }
    Nowangle=moto_gym[1].angle;//    /8192.0*360.0f;
    km_nor_pit_spd_fb = (imu_data.gx)/16.40;//单位为度/秒  ,速度反馈值
    autu_pit6623_AngleAdjust 	= (imu_data.gx)/16.40;//单位为度/秒

    if(last_Tk1pointang!=now_Tk1pointang)
    {
				now_Tk1pointang = last_Tk1pointang*0.2f + now_Tk1pointang	* 0.8f;
				pit_auto_angle_set= pit_auto_angle_set* 0.15 + (Nowangle+(now_Tk1pointang - auto_Piych_offset)*fact_shoot*0.113777777778)*0.85;//单位换算比较奇妙
        km_nor_pit_angle_set = pit_auto_angle_set;

        pid_calc(&Gm6623_pit_ang_pid_quick,Nowangle,pit_auto_angle_set);
        pid_calc(&Gm6623_pit_spd_pid,km_nor_pit_spd_fb,Gm6623_pit_ang_pid_quick.pos_out);
    }


    last_Tk1pointang=now_Tk1pointang;
}



/**************************************/
//手动自动切换函数
/**************************************/

void Gm6623_pitch_ang_ctr(void)
{
    //以下调用顺序不可变
    HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
    IMU_Get_Data();
    switch(modeswitch_flag[NOW])
    {
    case 1:
        break;
    case 2:
        break;
    case 3:
        km_nor_pit_ctr(&RC_CtrlData.mouse);
        break;//键盘随动模式
    case 4:
        km_au_pit_ctr();
        break;//相对位置自动瞄准模式break;
    default:
        break;
    }
}

//void km_nor_yaw_ctr(Mouse*mouse)
//{
//		//static int32_t Nowangle = yaw6623_MiddleAngle;
//		km_nor_yaw_ang_set 		-= mouse->x*0.1;			//系数为灵敏度
//		km_nor_yaw_spd_fb 		= (imu_data.gz)/16.40;//单位为度/秒,
//		imu_data.anglez 	+= km_nor_yaw_spd_fb*12/1000;//单位为度
//
////	if(mouse->x ==0 )//若有零飘可开此段
////	{
////		imu_data.anglez = 0;//清零imu积分误差
////		if(Nowangle > yaw6623_AngleAdjustLimit_left)
////			Nowangle = yaw6623_AngleAdjustLimit_left;
////		else if(Nowangle < yaw6623_AngleAdjustLimit_right)
////			Nowangle = yaw6623_AngleAdjustLimit_right;
////		//位置单环
////		pid_calc(&pid_yaw_pos,moto_gym[0].total_angle,Nowangle);
////		send_current_yaw = -pid_yaw_pos.pos_out;
////	}
////	else
//	{
////		Nowangle = moto_gym[0].total_angle;
//		km_nor_yaw_ang_fb  = imu_data.anglez*8192/360;
//		if(km_nor_yaw_ang_set > yaw6623_AngleAdjustLimit)
//			km_nor_yaw_ang_set = yaw6623_AngleAdjustLimit;
//		else if(km_nor_yaw_ang_set < -yaw6623_AngleAdjustLimit)
//			km_nor_yaw_ang_set = -yaw6623_AngleAdjustLimit;
//
//		//位置双环
//		pid_calc(&Gm6623_yaw_ang_pid,km_nor_yaw_ang_fb,km_nor_yaw_ang_set);
//		pid_calc(&Gm6623_yaw_spd_pid,km_nor_yaw_spd_fb,Gm6623_yaw_ang_pid.pos_out);
//		send_current_yaw = -Gm6623_yaw_spd_pid.pos_out;
//	}
//}


/**************************************/
//非自动yaw
/**************************************/

float C_Filter_speed_yaw = 0.2;
float C_Filter_angle_yaw = 0.5;
short SiQu_mouse_x=2;
float k_mouse_x=8.0;

void km_nor_yaw_ctr(Mouse*mouse)
{
    static float km_nor_yaw_spd_set=0;

    Follw_3508_pid.p = 6;


    km_nor_yaw_spd_fb 	= km_nor_yaw_spd_fb * 0.6  + (imu_data.gz)/16.40 * 0.4;//单位为度/秒,

    km_nor_yaw_ang_fb 	+= km_nor_yaw_spd_fb*12/1000;//单位为度
    km_nor_yaw_spd_set = k_mouse_x * (-mouse->x);

    if(mouse->x < SiQu_mouse_x && mouse->x > (0-SiQu_mouse_x))
    {
        mouse->x = 0;
    }

    km_nor_yaw_spd_set = (km_nor_yaw_spd_set * C_Filter_speed_yaw) + k_mouse_x * (-mouse->x) * (1 - C_Filter_speed_yaw);

    if(mouse->x > 3 || mouse->x < -3)
    {
        pid_calc(&Gm6623_yaw_spd_pid,km_nor_yaw_spd_fb,km_nor_yaw_spd_set);
        km_nor_yaw_ang_fb=0;
        Gm6623_yaw_ang_pid.pos_out=0;
        Gm6623_yaw_ang_pid.iout=0;
    }
    else
    {
        pid_calc(&Gm6623_yaw_ang_pid, km_nor_yaw_ang_fb, 0);
        pid_calc(&Gm6623_yaw_spd_pid, km_nor_yaw_spd_fb, Gm6623_yaw_ang_pid.pos_out);
    }
}

/**************************************/
//自动yaw
/**************************************/
/*
void km_au_yaw_ctr_yuanshi(void)//相对位置式自动打击
{
    float C=0.35;
    static float angle_pidout_temp=0;
    int Nowangle;
    static float yaw_auto_angle_set=yaw6623_MiddleAngle;//对应中点
    float now_Tk1pointang=0;
    static float last_Tk1pointang=0;
    now_Tk1pointang=Tk1pointang[0];
    if(Tk1pointang[2]==0)
    {
        now_Tk1pointang=last_Tk1pointang;
    }
    Nowangle=moto_gym[0].angle/8192.0*360.0f/0.012f;
    if(last_Tk1pointang!=now_Tk1pointang)
    {
        yaw_auto_angle_set= Nowangle-(now_Tk1pointang-200)/100*8192/360/2;//单位换算比较奇妙
    }
    autu_yaw6623_AngleAdjust 	= (imu_data.gz)/16.40;//单位为度/秒

    if(((yaw_auto_angle_set - Nowangle)<10) && ((yaw_auto_angle_set - Nowangle) > -10))
    {

        pid_calc(&Gm6623_yaw_ang_pid_slow,Nowangle, yaw_auto_angle_set );
        angle_pidout_temp = angle_pidout_temp*C + Gm6623_yaw_ang_pid_slow.pos_out *(1-C);
        pid_calc(&Gm6623_yaw_spd_pid,autu_yaw6623_AngleAdjust,angle_pidout_temp);
    }
    else
    {
        pid_calc(&Gm6623_yaw_ang_pid_quick,Nowangle, yaw_auto_angle_set );
        angle_pidout_temp = angle_pidout_temp*C + Gm6623_yaw_ang_pid_quick.pos_out *(1-C);
        pid_calc(&Gm6623_yaw_spd_pid,autu_yaw6623_AngleAdjust,angle_pidout_temp);
    }
    last_Tk1pointang=now_Tk1pointang;

}
*/

/**************************************/
//自动yaw
/**************************************/
float auto_yaw_offset=300;
float yaw_auto_angle_set=yaw6623_MiddleAngle/8192.0*360.0f;//对应中点
void km_au_yaw_ctr(void)
{
    static float km_nor_yaw_spd_set=0;

    float C=0.35;
    static float angle_pidout_temp=0;
    int Nowangle;

    float now_Tk1pointang=0;
    static float last_Tk1pointang=0;

    Follw_3508_pid.p = 1;

    now_Tk1pointang=Tk1pointang[0];
    km_nor_yaw_spd_fb 	= km_nor_yaw_spd_fb * 0.6  + (imu_data.gz)/16.40 * 0.4;//单位为度/秒,

    if(Tk1pointang[2]==0)
    {
        now_Tk1pointang=last_Tk1pointang;
        km_nor_yaw_ctr(&RC_CtrlData.mouse);
    }
    Nowangle=moto_gym[0].angle/8192.0*360.0f;

    if(last_Tk1pointang!=now_Tk1pointang)
    {
        //yaw_auto_angle_set= Nowangle-(now_Tk1pointang-200)/100*8192/360/2;//单位换算比较奇妙
        yaw_auto_angle_set= Nowangle-(now_Tk1pointang-auto_yaw_offset)*0.01;//单位换算比较奇妙
        pid_calc(&Gm6623_yaw_ang_pid_quick, Nowangle, yaw_auto_angle_set);
        pid_calc(&Gm6623_yaw_spd_pid, km_nor_yaw_spd_fb, Gm6623_yaw_ang_pid_quick.pos_out);
    }




    last_Tk1pointang=now_Tk1pointang;

}

/**************************************/
//模式切换函数yaw
/**************************************/
void Gm6623_yaw_ang_ctr(void)
{
    HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
    IMU_Get_Data();
    switch(modeswitch_flag[NOW])
    {
    case 1:
        break;
    case 2:
        break;
    case 3:
        km_nor_yaw_ctr(&RC_CtrlData.mouse);
        break;//键盘随动模式
    case 4:
        km_au_yaw_ctr();
        break;//相对位置自动瞄准模式break;
    default:
        break;
    }
}
//随动模式

float JiaJiao_now=0;
float JiaJiao_last=0;
float w_JiaJiao=0; //云台与底盘夹角变化速度。

void cm_follow(void)
{
    bool quitfollow_flag;

    JiaJiao_now = yaw6623_MiddleAngle - moto_gym[0].angle;
    w_JiaJiao = w_JiaJiao * 0.5f + (JiaJiao_now - JiaJiao_last) * 0.5f;
    JiaJiao_last = JiaJiao_now;

    pid_follow_calc(&Follw_3508_pid, moto_gym[0].total_angle, yaw6623_MiddleAngle);//角度环
    //pid_follow_calc(&Follw_3508_pid_speed, w_JiaJiao, Follw_3508_pid.pos_out);//角速度环
//	 if(abs(moto_gym[0].total_angle-yaw6623_MiddleAngle)<200)
//		 quitfollow_flag=0;
//	 else
    quitfollow_flag=1;

    Cm3508_total_set[0] = quitfollow_flag*Follw_3508_pid.pos_out+Cm3508_spd_set[0];
    Cm3508_total_set[1] = quitfollow_flag*Follw_3508_pid.pos_out+Cm3508_spd_set[1];
    Cm3508_total_set[2] = quitfollow_flag*Follw_3508_pid.pos_out+Cm3508_spd_set[2];
    Cm3508_total_set[3] = quitfollow_flag*Follw_3508_pid.pos_out+Cm3508_spd_set[3];

//	   Cm3508_total_set[0] = quitfollow_flag*Follw_3508_pid_speed.pos_out+Cm3508_spd_set[0];
//    Cm3508_total_set[1] = quitfollow_flag*Follw_3508_pid_speed.pos_out+Cm3508_spd_set[1];
//    Cm3508_total_set[2] = quitfollow_flag*Follw_3508_pid_speed.pos_out+Cm3508_spd_set[2];
//    Cm3508_total_set[3] = quitfollow_flag*Follw_3508_pid_speed.pos_out+Cm3508_spd_set[3];
}

/**************************************/
//底盘控制
/**************************************/
//分离模式
void cm_unfollow(void)
{
    Cm3508_total_set[0] = Cm3508_spd_set[0];
    Cm3508_total_set[1] = Cm3508_spd_set[1];
    Cm3508_total_set[2] = Cm3508_spd_set[2];
    Cm3508_total_set[3] = Cm3508_spd_set[3];
}
void CM_ctr(void)
{
    if(follow_flag)
        cm_follow();
    else
        cm_unfollow();
    cm_spd_ctr();
}

/**************************************/
//未使用
/**************************************/
void GM6623_yaw_autu_ctr(void)//绝对位置式自动打击
{
    autu_6623yaw_imu_data.Velocity[NOW] = (imu_data.gz - ImugzErr)/25 ;//粗略式去温差，滤波
    autu_6623yaw_imu_data.Angle[NOW] 	= autu_6623yaw_imu_data.Velocity[NOW] + autu_6623yaw_imu_data.Angle[LAST];
    //autu_yaw6623_AngleAdjust = moto_gym[0].angle - yaw6623_MiddleAngle;
    autu_yawimu_AngleAdjust 	= autu_6623yaw_imu_data.Angle[NOW];
    //Gm6623_yaw_ang_give = Gm6623_yaw_ang_set - yawimu_AngleAdjust * yawimu_AngleAdjustK;
    autu_pid_calc_yaw(&autu_Gm6623_yaw_ang_pid, autu_yawimu_AngleAdjust, autu_Gm6623_yaw_ang_set+autu_yawimu_AngleAdjust);//改变Gm6623_yaw_ang_set，底盘跟随云台，可使Gm6623_yaw_ang_give保持在4650（+-5）

    autu_6623yaw_imu_data.Velocity[LAST] = autu_6623yaw_imu_data.Velocity[NOW];
    autu_6623yaw_imu_data.Angle[LAST] 	 = autu_6623yaw_imu_data.Angle[NOW];

//		if(autu_Gm6623_yaw_ang_set-autu_yawimu_AngleAdjust<autuang_limit)
//		  autuaim_flag=0;
//		else
//			autuaim_flag=1;

}


