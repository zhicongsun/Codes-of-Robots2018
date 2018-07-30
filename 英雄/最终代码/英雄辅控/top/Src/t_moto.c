#include "t_moto.h"
#include "m_moto.h"
#include "m_imu.h"
#include "pid.h"
#include "m_remote.h"
#include "t_tk1.h"
#include "m_toggle.h"
extern float var[7];//调试用
float Cm3508_spd_get[4]={0,0,0,0};
float Cm3508_spd_xset[4];//x方向分量速度
float Cm3508_spd_yset[4];//y方向分量速度
float Cm3505_spd_sqin[4];//自旋运动
float Cm3508_spd_set[4];//合成速度
float Cm3508_total_set[4];//总pid输出=底盘运动的pid输出+随动pid输出
float Cm3508_spd_offset[4]={0};
float Cm3508_ang_set[5];
//底盘走直线的校验
int16_t offsetang;
int16_t lastoffsetang=0;

float Gm6623_pit_ang_set=pit6623_MiddleAngle;
float Gm6623_pit_spd_set=0;


float Gm6623_yaw_ang_set=yaw6623_MiddleAngle;
float Gm6623_yaw_spd_set=0;
float Gm6623_yaw_ang_give=yaw6623_MiddleAngle;

//随动参数
Follow_6623_imu_fbangall_t follow_6623yaw_imu_data;
int yaw6623_AngleAdjust;
int yawimu_AngleAdjust;
float pitimu_AngleAdjust; 

//自动瞄参数
Follow_6623_imu_fbangall_t autu_6623yaw_imu_data;
float autu_Gm6623_yaw_ang_set=0;
int autu_yaw6623_AngleAdjust;
int autu_yawimu_AngleAdjust;
float error1=0;
float error2=0;
int Toggler_working_time;
float  Toggler_ang;
float testi;
//取弹时调用
void Cm3508_ang_ctr(pid_t *Cm3508_ang_pid, pid_t *Cm3508_spd_pid,moto_measure_t *moto3508_elevator,float* g3508_angle_set)
{

	error2=error2*0.8+Toggler_ang-(moto_chassis[2].total_angle)*0.2;
	if((error2< Toggler_angle_zero)&&(error2> -Toggler_angle_zero))
		Toggler_working_time = 0;
	else
		Toggler_working_time++;
	

	if(Toggler_working_time > 30) 
	{
		if(((*g3508_angle_set)*19.0*8192.0/360.0)- moto_chassis[2].total_angle>Toggler_angle_one)
		(*g3508_angle_set) -= 72;
		Cm3508_spd_pid->pos_out=-5000;
		Toggler_working_time = 0;
		testi=((*g3508_angle_set)*19.0*8192.0/360.0)- moto_chassis[2].total_angle;
	}
	Toggler_ang=(*g3508_angle_set)*19.0*8192.0/360.0;
	pid_calc(Cm3508_ang_pid, moto3508_elevator->total_angle-moto3508_elevator->offset_angle, Toggler_ang);	//g3508_angle_set位为°
	pid_calc(Cm3508_spd_pid, moto3508_elevator->speed_rpm, Cm3508_ang_pid->pos_out);
	
}

void Cm3508_spd_ctr(void)
{	
	for(int i=0; i<4; i++)
	{
		pid_calc(&Cm3508_spd_pid[i], moto_chassis[i].speed_rpm, Cm3508_total_set[i]);
	}			
	
}

void Gm6623_pitch_ang_ctr(void)
{
	//以下调用顺序不可变	
	
	//static int NowAngle=pit6623_MiddleAngle;
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	IMU_Get_Data();
	pitimu_AngleAdjust = (imu_data.gx-ImuGxErr)/16.40;//单位为度/秒
	imu_data.anglex += pitimu_AngleAdjust;//单位不是°
	
	if(Gm6623_pit_spd_set >2 || Gm6623_pit_spd_set < -2)
	{
		pid_calc(&Gm6623_pit_spd_pid,pitimu_AngleAdjust,Gm6623_pit_spd_set);
		imu_data.anglex = 0;
		//NowAngle = 0;
		Gm6623_pit_ang_pid.pos_out=0; 
		Gm6623_pit_ang_pid.iout=0;
	}
	else
	{
		//pid_calc(&Gm6623_pit_ang_pid,moto_gym[1].total_angle , NowAngle);
		pid_calc(&Gm6623_pit_ang_pid,imu_data.anglex, -pit6623_MiddleAngle/2+angle_set);
		pid_calc(&Gm6623_pit_spd_pid,pitimu_AngleAdjust,Gm6623_pit_ang_pid.pos_out);
		//pid_calc(&Gm6623_pit_spd_pid,pitimu_AngleAdjust,angle_set);//先调速度环
	}
}

void GM6623_yaw_follow_ctr(void)
{  	 
		yawimu_AngleAdjust = (imu_data.gz-ImuGzErr)/16.40;//单位为度/秒
    imu_data.anglez += yawimu_AngleAdjust;//单位不是°
	
	if(Gm6623_yaw_spd_set >2 || Gm6623_yaw_spd_set < -2)
	{
		pid_calc(&Gm6623_yaw_spd_pid,yawimu_AngleAdjust,Gm6623_yaw_spd_set);
		imu_data.anglez=0;
		Gm6623_yaw_ang_pid.pos_out=0;
		Gm6623_yaw_ang_pid.iout=0;
	}
	else
	{
		pid_calc(&Gm6623_yaw_ang_pid,imu_data.anglez, 0);
		pid_calc(&Gm6623_yaw_spd_pid,yawimu_AngleAdjust,Gm6623_yaw_ang_pid.pos_out);
//		pid_calc(&Gm6623_yaw_spd_pid,yawimu_AngleAdjust,angle_set);//先调速度环
		
	}
}
void Gm6623_yaw_ang_ctr(void)
{
	HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	IMU_Get_Data();

		GM6623_yaw_follow_ctr();//随动模式
}
//随动模式
void CM_follow_gy_dataprocess(void)
{
	 bool quitfollow_flag;
	 pid_follow_calc(&Follw_3508_pid, moto_gym[0].total_angle, yaw6623_MiddleAngle);//角度环

//	 Follow_3508_squinspeed[0] = (float)  Follw_3508_pid.pos_out;//1s大约标准60度,15ms调用一次这个函数,机械转角大约200,200*10=2000
//	 Follow_3508_squinspeed[1] = (float)  Follw_3508_pid.pos_out;
//   Follow_3508_squinspeed[2] = (float)  Follw_3508_pid.pos_out;
//   Follow_3508_squinspeed[3] = (float)  Follw_3508_pid.pos_out;
	
	 if(abs(moto_gym[0].total_angle-yaw6623_MiddleAngle)<90)
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


