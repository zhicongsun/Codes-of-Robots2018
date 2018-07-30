#include "t_moto.h"
#include "m_moto.h"
#include "m_imu.h"
#include "pid.h"
#include "m_remote.h"
#include "t_tk1.h"

float Cm3508_spd_get[4]={0,0,0,0};
float Cm3508_spd_xset[4];//x方向分量速度
float Cm3508_spd_yset[4];//y方向分量速度
float Cm3505_spd_sqin[4];//自旋运动
float Cm3508_spd_set[4];//合成速度
float Cm3508_total_set[4];//总pid输出=底盘运动的pid输出+随动pid输出
float Cm3508_spd_offset[4]={0};
//底盘走直线的校验
int16_t offsetang;
int16_t lastoffsetang=0;

float Gm6623_pit_ang_set=2700;
float Gm6623_pit_spd_set=0;
float Gm6623_pit_ang_offset=25;//克服重力影响的偏差角，机械一调整需要重新矫正

float Gm6623_yaw_ang_set=4650;
float Gm6623_yaw_spd_set=0;
float Gm6623_yaw_ang_give=4650;

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
//限幅参数
int err;
int offset = yaw6623_MiddleAngle;

//走直线的校准，暂时未调用，别调用，否则会和随动冲突
void Cmoffset(void)
{
	offsetang=imu_data.gz+lastoffsetang;
	lastoffsetang=offsetang;	 
	pid_calc(&Cmsqinspd, offsetang, 0);//设定偏差角为零
	Cm3508_spd_offset[0]=(float)-Cmsqinspd.pos_out;
	Cm3508_spd_offset[1]=(float)Cmsqinspd.pos_out;
	Cm3508_spd_offset[2]=(float)-Cmsqinspd.pos_out;
	Cm3508_spd_offset[3]=(float)Cmsqinspd.pos_out;
}
extern float cyl_spd;

void Cm3508_spd_ctr(void)
{	
	for(int i=0; i<4; i++)
	{
		//pid_calc(&Cm3508_spd_pid[i], moto_chassis[i].speed_rpm, Cm3508_total_set[i]);
		pid_calc(&Cm3508_spd_pid[i], moto_chassis[i].speed_rpm, Cm3508_spd_set[i]);
	}			  
	pid_calc(&Cm3508_spd_pid[4], moto_chassis[4].speed_rpm,cyl_spd);
	pid_calc(&Cm3508_spd_pid[5], moto_chassis[5].speed_rpm,cyl_spd); 
}

void Gm6623_pitch_ang_ctr(void)
{
	//以下调用顺序不可变	
	
	static int NowAngle=pit6623_MiddleAngle;
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	IMU_Get_Data();
	pitimu_AngleAdjust 	= (imu_data.gx-14.5)/16.40;//单位为度/秒,14.5是误差

	
	if(Gm6623_pit_spd_set >2)
	{
		if(moto_gym[1].total_angle > pit6623_TopAngle)
			pid_calc(&Gm6623_pit_spd_pid,pitimu_AngleAdjust,Gm6623_pit_spd_set);
		NowAngle = moto_gym[1].total_angle;
		Gm6623_pit_ang_pid.pos_out=0;
		Gm6623_pit_ang_pid.iout=0;
	}
	else if(Gm6623_pit_spd_set < -2)
	{
		if(moto_gym[1].total_angle < pit6623_BottomAngle) 
			pid_calc(&Gm6623_pit_spd_pid,pitimu_AngleAdjust,Gm6623_pit_spd_set);
		NowAngle = moto_gym[1].total_angle;
		Gm6623_pit_ang_pid.pos_out=0;
		Gm6623_pit_ang_pid.iout=0;		
	} 
	else
	{
		if(moto_gym[1].total_angle > pit6623_BottomAngle) 
			NowAngle = pit6623_BottomAngle;
		else if(moto_gym[1].total_angle < pit6623_TopAngle)
			NowAngle = pit6623_TopAngle;
		pid_calc(&Gm6623_pit_ang_pid,moto_gym[1].total_angle , NowAngle );
		pid_calc(&Gm6623_pit_spd_pid,pitimu_AngleAdjust,Gm6623_pit_ang_pid.pos_out);
	}

}

void Gm6623_yaw_ang_ctr(void)
{
	static int32_t yaw_angle_set=yaw6623_MiddleAngle;
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	IMU_Get_Data();
		yawimu_AngleAdjust 	= (imu_data.gz)/16.40;//单位为度/秒
	if(RC_CtrlData.rc.s2 == 1)
	{
		yaw_angle_set = yaw6623_MiddleAngle +8192;
	}
	else if(RC_CtrlData.rc.s2 == 3)
	{
		yaw_angle_set = yaw6623_MiddleAngle;
	}
	else if(RC_CtrlData.rc.s2 == 2)
	{
		yaw_angle_set =	yaw6623_MiddleAngle -8192;
	}
		pid_calc(&Gm6623_yaw_ang_pid,moto_gym[0].total_angle,yaw_angle_set);
		pid_calc(&Gm6623_yaw_spd_pid,yawimu_AngleAdjust,Gm6623_yaw_ang_pid.pos_out);

}
//非随动模式（反馈为机械角度）
//void Gm6623_yaw_ang_ctr(void)
//{	//以下调用顺序不可变	
//	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
//	IMU_Get_Data();	
//	pid_calc_yaw(&Gm6623_yaw_ang_pid, moto_gym[0].angle, Gm6623_yaw_ang_set );	
//	//Set_6623_current(&hcan1,-(Gm6623_yaw_ang_pid.pos_out),0,0,0);//此值为正，6623顺时针旋转，云台PITCH轴电机往上摆，所以输出前面要加负号。	
//}
//随动模式
void CM_follow_gy_dataprocess(void)
{
	 bool quitfollow_flag;
	 pid_follow_calc(&Follw_3508_pid, moto_gym[0].angle, yaw6623_MiddleAngle);//角度环

//	 Follow_3508_squinspeed[0] = (float)  Follw_3508_pid.pos_out;//1s大约标准60度,15ms调用一次这个函数,机械转角大约200,200*10=2000
//	 Follow_3508_squinspeed[1] = (float)  Follw_3508_pid.pos_out;
//   Follow_3508_squinspeed[2] = (float)  Follw_3508_pid.pos_out;
//   Follow_3508_squinspeed[3] = (float)  Follw_3508_pid.pos_out;
	
	 if(abs(moto_gym[0].angle-yaw6623_MiddleAngle)<90)
		 quitfollow_flag=0;
	 else
		 quitfollow_flag=1;
	 
	 Cm3508_total_set[0] = quitfollow_flag*Follw_3508_pid.pos_out+Cm3508_spd_set[0];
	 Cm3508_total_set[1] = quitfollow_flag*Follw_3508_pid.pos_out+Cm3508_spd_set[1];
	 Cm3508_total_set[2] = quitfollow_flag*Follw_3508_pid.pos_out+Cm3508_spd_set[2];
	 Cm3508_total_set[3] = quitfollow_flag*Follw_3508_pid.pos_out+Cm3508_spd_set[3];
}
//自动瞄准模式
void autu_CM3508_dataprocess(void)
{
	 Cm3508_total_set[0] = Cm3508_spd_set[0];
	 Cm3508_total_set[1] = Cm3508_spd_set[1];
	 Cm3508_total_set[2] = Cm3508_spd_set[2];
	 Cm3508_total_set[3] = Cm3508_spd_set[3];
}
//void CM_follow_gy_dataprocess(void)
//{
//	 unsigned short i;
//	 pid_follow_calc(&Follw_3508_pid, moto_gym[0].angle, 0);
//	
//	 Follow_3508_squinspeed[0] = (float)-Follw_3508_pid.pos_out*10;//1s大约标准60度,15ms调用一次这个函数,机械转角大约200,200*10=2000
//	 Follow_3508_squinspeed[1] = (float) Follw_3508_pid.pos_out*10;
//   Follow_3508_squinspeed[2] = (float)-Follw_3508_pid.pos_out*10;
//   Follow_3508_squinspeed[3] = (float) Follw_3508_pid.pos_out*10;
//	 for(i=0;i<4;i++)
//	 {
//	    Cm3508_total_set[i]=Follow_3508_squinspeed[i] + Cm3508_spd_pid[i].pos_out;
//	 }
//}


