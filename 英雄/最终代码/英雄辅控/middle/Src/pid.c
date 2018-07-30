#include "pid.h"
#include "m_imu.h"


//丝杠
float P_cyl_spd=20,I_cyl_spd=0.4,D_cyl_spd=0;
float P_cyl_ang=0.025,I_cyl_ang=0,D_cyl_ang=0; 
//拨弹3508
float P_btog_spd=12,I_btog_spd=0.4,D_btog_spd=0;
float P_btog_ang=0.025,I_btog_ang=0,D_btog_ang=0; 


float P_Follow=4,I_Follow=0.01,D_Follow=4;
/*************  云台yaw  *********/
//float P_6623_yaw_ang=0.1,I_6623_yaw_ang=0,D_6623_yaw_ang=0;
//float P_yaw_spd= 6,I_yaw_spd=0.03,D_yaw_spd=2;
float P_6623_yaw_ang=1,I_6623_yaw_ang=0,D_6623_yaw_ang=2;
float P_yaw_spd= 25,I_yaw_spd=0.5,D_yaw_spd=5;
float P_autu_6623_yaw_ang=12,I_autu_6623_yaw_ang=0,D_autu_6623_yaw_ang=0;

/*************  云台pitch  *********/
float P_6623_pit_spd=28,I_6623_pit_spd=2.7,D_6623_pit_spd=7;
float P_6623_pit_ang=0.35,I_6623_pit_ang=0,D_6623_pit_ang=0.4;
float autu_P_6623_pit_ang=8,autu_I_6623_pit_ang=0,autu_D_6623_pit_ang=0;

/*************  拨弹  *********/

float P_tog_ang = 0.2, 	I_tog_ang = 0,D_tog_ang = 0.14;
float P_tog_spd = 5, 		I_tog_spd = 0.1,D_tog_spd = 0.1;


pid_t Power_pid;
pid_t Cmsqinspd;
pid_t Cm3508_ang_pid[6];
pid_t Cm3508_spd_pid[6];
pid_t Gm6623_pit_ang_pid;
pid_t Gm6623_pit_spd_pid;

pid_t Gm6623_yaw_ang_pid;//随动位置环yaw
pid_t Gm6623_yaw_spd_pid;//随动速度环yaw
pid_t autu_Gm6623_yaw_ang_pid;//自动瞄准位置环yaw

pid_t Gm6623_pit_ang_pid;//随动pitch
pid_t autu_Gm6623_pit_ang_pid;//自动瞄准pitch

pid_t autu_Gm6623_yaw_spd_pid;

pid_t pid_Toggler_ang;
pid_t pid_Toggler_spd;

pid_t Toggler_spd_pid;
pid_t pid_yaw_Iner;
pid_t pid_pit_Iner;

pid_t Follw_3508_pid;//随动的3508pid

void abs_limit(float *a, float ABS_MAX){
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}
/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
    pid_t *pid, 
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;
    
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
    
}
/*中途更改参数设定(调试)------------------------------------------------------------*/
//static void pid_reset(pid_t	*pid, float kp, float ki, float kd)
//{
//    pid->p = kp;
//    pid->i = ki;
//    pid->d = kd;
//}

void pid_reset(pid_t	*pid, float kp, float ki, float kd)
{
    pid->p = kp;
    pid->i = ki;
    pid->d = kd;
}

/**
    *@bref. calculate delta PID and position PID
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_calc(pid_t* pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure  
    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
			return 0;
	  if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
			return 0;
    
    if(pid->pid_mode == POSITION_PID) //位置式p
    {
        pid->pout = pid->p * pid->err[NOW];
        pid->iout += pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - pid->err[LAST] );
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    }
    else if(pid->pid_mode == DELTA_PID)//增量式P
    {
        pid->pout = pid->p * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->i * pid->err[NOW];
        pid->dout = pid->d * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);
        
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }
    
    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->get[LLAST] = pid->get[LAST];
    pid->get[LAST] = pid->get[NOW];
    pid->set[LLAST] = pid->set[LAST];
    pid->set[LAST] = pid->set[NOW];
    return pid->pid_mode==POSITION_PID ? pid->pos_out : pid->delta_out;
//	
}
//随动3508pid模式一（假双环）
float pid_follow_calc(pid_t* pid_position, float get, float set)
{		
	pid_position->IntegralLimit=2047;//2047对应90度的机械转角	
	  pid_position->MaxOutput=2047;
/***************************************	外环	******************************************/	
    pid_position->get[NOW] = get;
    pid_position->set[NOW] = set;
    pid_position->err[NOW] = set - get;	//set - measure  
    
	  pid_position->pout = pid_position->p * pid_position->err[NOW];
    pid_position->iout += pid_position->i * pid_position->err[NOW];
    pid_position->dout = -pid_position->d * imu_data.gz;				
    abs_limit(&(pid_position->iout), pid_position->IntegralLimit);
    pid_position->pos_out = pid_position->pout + pid_position->iout + pid_position->dout;
    abs_limit(&(pid_position->pos_out), pid_position->MaxOutput);
    pid_position->err[LAST] = pid_position->err[NOW];
    return  pid_position->pos_out;	
}
//随动3508pid模式二（真双环）
//float pid_follow_calc(pid_t* pid_position, float get, float set)
//{	
//	  pid_yaw_Iner.p=2;
//	  pid_yaw_Iner.i=0;
//	  pid_yaw_Iner.d=0;
//  	pid_position->IntegralLimit=700;	
//	  pid_position->MaxOutput=3500;
//	  pid_yaw_Iner.IntegralLimit=0;
//	  pid_yaw_Iner.MaxOutput=6000;
//	
///***************************************	外环	******************************************/	
//    pid_position->get[NOW] = get;
//    pid_position->set[NOW] = set;
//    pid_position->err[NOW] = set - get;	//set - measure  
//    
//	  pid_position->pout = pid_position->p * pid_position->err[NOW];
//    pid_position->iout += pid_position->i * pid_position->err[NOW];
//    pid_position->dout = -pid_position->d * imu_data.gz;				
//    abs_limit(&(pid_position->iout), pid_position->IntegralLimit);
//    pid_position->pos_out = pid_position->pout + pid_position->iout + pid_position->dout;
//    abs_limit(&(pid_position->pos_out), pid_position->MaxOutput);
//    pid_position->err[LAST] = pid_position->err[NOW];
///***************************************	内环	******************************************/
//	  pid_yaw_Iner.err[NOW] = pid_position->pos_out-imu_data.gz;
//		pid_yaw_Iner.pout = pid_yaw_Iner.p * pid_yaw_Iner.err[NOW];
//    pid_yaw_Iner.iout += pid_yaw_Iner.i * pid_yaw_Iner.err[NOW];
//		abs_limit(&(pid_yaw_Iner.iout), pid_yaw_Iner.IntegralLimit);
//		pid_yaw_Iner.dout = pid_yaw_Iner.d * (pid_yaw_Iner.err[NOW] - pid_yaw_Iner.err[LAST] );
//		pid_yaw_Iner.pos_out = pid_yaw_Iner.pout + pid_yaw_Iner.iout + pid_yaw_Iner.dout;
//    abs_limit(&(pid_yaw_Iner.pos_out), pid_yaw_Iner.MaxOutput);
//    pid_yaw_Iner.err[LAST] = pid_yaw_Iner.err[NOW];
///***************************************	返回	******************************************/		
//	  pid_position->pos_out = pid_yaw_Iner.pos_out;
//    return  pid_position->pos_out;	
//}

//随动yaw
float pid_calc_yaw(pid_t* pid_position, float get, float set)
{	
	  pid_yaw_Iner.p=1;
	  pid_yaw_Iner.i=0;
	  pid_yaw_Iner.d=0;
  	  pid_position->IntegralLimit=700;	
	  pid_position->MaxOutput=1000;
	  pid_yaw_Iner.IntegralLimit=0;
	  pid_yaw_Iner.MaxOutput=3000;
	  
//	  pid_yaw_Iner.p=1;
//	  pid_yaw_Iner.i=0;
//	  pid_yaw_Iner.d=0;
//  	pid_position->IntegralLimit=100;	
//	  pid_position->MaxOutput=1000;
//	  pid_yaw_Iner.IntegralLimit=0;
//	  pid_yaw_Iner.MaxOutput=5000;
	
/***************************************	外环	******************************************/	
    pid_position->get[NOW] = get;
    pid_position->set[NOW] = set;
    pid_position->err[NOW] = set - get;	//set - measure  
    
	  pid_position->pout = pid_position->p * pid_position->err[NOW];
    pid_position->iout += pid_position->i * pid_position->err[NOW];
    pid_position->dout = -pid_position->d * imu_data.gz;						//yaw
    abs_limit(&(pid_position->iout), pid_position->IntegralLimit);
    pid_position->pos_out = pid_position->pout + pid_position->iout + pid_position->dout;
    abs_limit(&(pid_position->pos_out), pid_position->MaxOutput);

    pid_position->err[LAST] = pid_position->err[NOW];
/***************************************	内环	******************************************/
	  pid_yaw_Iner.err[NOW] = pid_position->pos_out-imu_data.gz;
		pid_yaw_Iner.pout = pid_yaw_Iner.p * pid_yaw_Iner.err[NOW];
    pid_yaw_Iner.iout += pid_yaw_Iner.i * pid_yaw_Iner.err[NOW];
		abs_limit(&(pid_yaw_Iner.iout), pid_yaw_Iner.IntegralLimit);
		pid_yaw_Iner.dout = pid_yaw_Iner.d * (pid_yaw_Iner.err[NOW] - pid_yaw_Iner.err[LAST] );
		pid_yaw_Iner.pos_out = pid_yaw_Iner.pout + pid_yaw_Iner.iout + pid_yaw_Iner.dout;
    abs_limit(&(pid_yaw_Iner.pos_out), pid_yaw_Iner.MaxOutput);
    pid_yaw_Iner.err[LAST] = pid_yaw_Iner.err[NOW];
/***************************************	返回	******************************************/		
	  pid_position->pos_out = pid_yaw_Iner.pos_out;
    return  pid_position->pos_out;	
}
//自动瞄准yaw
float autu_pid_calc_yaw(pid_t* pid_position, float get, float set)
{	
	  pid_yaw_Iner.p=5;
	  pid_yaw_Iner.i=0;
	  pid_yaw_Iner.d=0;
  	pid_position->IntegralLimit=700;	
	  pid_position->MaxOutput=1000;
	  pid_yaw_Iner.IntegralLimit=0;
	  pid_yaw_Iner.MaxOutput=3000;
	  
//	  pid_yaw_Iner.p=1;
//	  pid_yaw_Iner.i=0;
//	  pid_yaw_Iner.d=0;
//  	pid_position->IntegralLimit=100;	
//	  pid_position->MaxOutput=1000;
//	  pid_yaw_Iner.IntegralLimit=0;
//	  pid_yaw_Iner.MaxOutput=5000;
	
/***************************************	外环	******************************************/	
    pid_position->get[NOW] = get;
    pid_position->set[NOW] = set;
    pid_position->err[NOW] = set - get;	//set - measure  
    
	  pid_position->pout = pid_position->p * pid_position->err[NOW];
    pid_position->iout += pid_position->i * pid_position->err[NOW];
    pid_position->dout = -pid_position->d * imu_data.gz;						//yaw
    abs_limit(&(pid_position->iout), pid_position->IntegralLimit);
    pid_position->pos_out = pid_position->pout + pid_position->iout + pid_position->dout;
    abs_limit(&(pid_position->pos_out), pid_position->MaxOutput);

    pid_position->err[LAST] = pid_position->err[NOW];
/***************************************	内环	******************************************/
	  pid_yaw_Iner.err[NOW] = pid_position->pos_out-imu_data.gz;
		pid_yaw_Iner.pout = pid_yaw_Iner.p * pid_yaw_Iner.err[NOW];
    pid_yaw_Iner.iout += pid_yaw_Iner.i * pid_yaw_Iner.err[NOW];
		abs_limit(&(pid_yaw_Iner.iout), pid_yaw_Iner.IntegralLimit);
		pid_yaw_Iner.dout = pid_yaw_Iner.d * (pid_yaw_Iner.err[NOW] - pid_yaw_Iner.err[LAST] );
		pid_yaw_Iner.pos_out = pid_yaw_Iner.pout + pid_yaw_Iner.iout + pid_yaw_Iner.dout;
    abs_limit(&(pid_yaw_Iner.pos_out), pid_yaw_Iner.MaxOutput);
    pid_yaw_Iner.err[LAST] = pid_yaw_Iner.err[NOW];
/***************************************	返回	******************************************/		
	  pid_position->pos_out = pid_yaw_Iner.pos_out;
    return  pid_position->pos_out;	
}
//非随动yaw
//float pid_calc_yaw(pid_t* pid_position, float get, float set)
//{	
//	  pid_yaw_Iner.p=2;
//	  pid_yaw_Iner.i=0;
//	  pid_yaw_Iner.d=0;
//  	pid_position->IntegralLimit=700;	
//	  pid_position->MaxOutput=3500;
//	  pid_yaw_Iner.IntegralLimit=0;
//	  pid_yaw_Iner.MaxOutput=6000;
//	  
////	  pid_yaw_Iner.p=1;
////	  pid_yaw_Iner.i=0;
////	  pid_yaw_Iner.d=0;
////  	pid_position->IntegralLimit=100;	
////	  pid_position->MaxOutput=1000;
////	  pid_yaw_Iner.IntegralLimit=0;
////	  pid_yaw_Iner.MaxOutput=5000;
//	
///***************************************	外环	******************************************/	
//    pid_position->get[NOW] = get;
//    pid_position->set[NOW] = set;
//    pid_position->err[NOW] = set - get;	//set - measure  
//    
//	  pid_position->pout = pid_position->p * pid_position->err[NOW];
//    pid_position->iout += pid_position->i * pid_position->err[NOW];
//    pid_position->dout = -pid_position->d * imu_data.gz;						//yaw
//    abs_limit(&(pid_position->iout), pid_position->IntegralLimit);
//    pid_position->pos_out = pid_position->pout + pid_position->iout + pid_position->dout;
//    abs_limit(&(pid_position->pos_out), pid_position->MaxOutput);

//    pid_position->err[LAST] = pid_position->err[NOW];
///***************************************	内环	******************************************/
//	  pid_yaw_Iner.err[NOW] = pid_position->pos_out-imu_data.gz;
//		pid_yaw_Iner.pout = pid_yaw_Iner.p * pid_yaw_Iner.err[NOW];
//    pid_yaw_Iner.iout += pid_yaw_Iner.i * pid_yaw_Iner.err[NOW];
//		abs_limit(&(pid_yaw_Iner.iout), pid_yaw_Iner.IntegralLimit);
//		pid_yaw_Iner.dout = pid_yaw_Iner.d * (pid_yaw_Iner.err[NOW] - pid_yaw_Iner.err[LAST] );
//		pid_yaw_Iner.pos_out = pid_yaw_Iner.pout + pid_yaw_Iner.iout + pid_yaw_Iner.dout;
//    abs_limit(&(pid_yaw_Iner.pos_out), pid_yaw_Iner.MaxOutput);
//    pid_yaw_Iner.err[LAST] = pid_yaw_Iner.err[NOW];
///***************************************	返回	******************************************/		
//	  pid_position->pos_out = pid_yaw_Iner.pos_out;
//    return  pid_position->pos_out;	
//}
//    PitchOPID.P = 8;
//    PitchOPID.I = 0.015;
//    PitchOPID.D = 0;
//    PitchOPID.IMax = 700;
//    PitchOPID.PIDMax = 1300;
//    
//    PitchIPID.P = 3;
//    PitchIPID.I = 0;
//    PitchIPID.D = 0;
//    PitchIPID.IMax = 0;
//    PitchIPID.PIDMax = 5000;
float pid_calc_pitch(pid_t* pid_position, float get, float set)
{	
	pid_pit_Iner.p=2.5;
	pid_pit_Iner.i=0;
	pid_pit_Iner.d=0;

	pid_position->IntegralLimit=700;	
	pid_position->MaxOutput=1000;

	pid_pit_Iner.IntegralLimit=0;
	pid_pit_Iner.MaxOutput=3000;
	
//	pid_pit_Iner.p=2;
//	pid_pit_Iner.i=0;
//	pid_pit_Iner.d=0;

//	pid_position->IntegralLimit=700;	
//	pid_position->MaxOutput=1300;

//	pid_pit_Iner.IntegralLimit=0;
//	pid_pit_Iner.MaxOutput=5000;
	
/***************************************	外环	******************************************/	
    pid_position->get[NOW] = get;
    pid_position->set[NOW] = set;
    pid_position->err[NOW] = set - get;	//set - measure  
    pid_position->pout = pid_position->p * pid_position->err[NOW];
    pid_position->iout += pid_position->i * pid_position->err[NOW];
    pid_position->dout = -pid_position->d * imu_data.gx;
    abs_limit(&(pid_position->iout), pid_position->IntegralLimit);
    pid_position->pos_out = pid_position->pout + pid_position->iout + pid_position->dout;
    abs_limit(&(pid_position->pos_out), pid_position->MaxOutput);
    pid_position->err[LAST] = pid_position->err[NOW];
/***************************************	内环	******************************************/
		pid_pit_Iner.err[NOW] = pid_position->pos_out-imu_data.gx;
		pid_pit_Iner.pout = pid_pit_Iner.p * pid_pit_Iner.err[NOW];
		pid_pit_Iner.iout += pid_pit_Iner.i * pid_pit_Iner.err[NOW];
		abs_limit(&(pid_pit_Iner.iout), pid_pit_Iner.IntegralLimit);
		pid_pit_Iner.dout = pid_pit_Iner.d * (pid_pit_Iner.err[NOW] - pid_pit_Iner.err[LAST] );
		pid_pit_Iner.pos_out = pid_pit_Iner.pout + pid_pit_Iner.iout + pid_pit_Iner.dout;
		abs_limit(&(pid_pit_Iner.pos_out), pid_pit_Iner.MaxOutput);
		pid_pit_Iner.err[LAST] = pid_pit_Iner.err[NOW];
/***************************************	返回	******************************************/		
	  pid_position->pos_out = pid_pit_Iner.pos_out;
    return  pid_position->pos_out;	
}
float autu_pid_calc_pitch(pid_t* pid_position, float get, float set)
{	
	pid_pit_Iner.p=2.5;
	pid_pit_Iner.i=0;
	pid_pit_Iner.d=0;

	pid_position->IntegralLimit=700;	
	pid_position->MaxOutput=1000;

	pid_pit_Iner.IntegralLimit=0;
	pid_pit_Iner.MaxOutput=3000;
	
//	pid_pit_Iner.p=2;
//	pid_pit_Iner.i=0;
//	pid_pit_Iner.d=0;

//	pid_position->IntegralLimit=700;	
//	pid_position->MaxOutput=1300;

//	pid_pit_Iner.IntegralLimit=0;
//	pid_pit_Iner.MaxOutput=5000;
	
/***************************************	外环	******************************************/	
    pid_position->get[NOW] = get;
    pid_position->set[NOW] = set;
    pid_position->err[NOW] = set - get;	//set - measure  
    pid_position->pout = pid_position->p * pid_position->err[NOW];
    pid_position->iout += pid_position->i * pid_position->err[NOW];
    pid_position->dout = -pid_position->d * imu_data.gx;
    abs_limit(&(pid_position->iout), pid_position->IntegralLimit);
    pid_position->pos_out = pid_position->pout + pid_position->iout + pid_position->dout;
    abs_limit(&(pid_position->pos_out), pid_position->MaxOutput);
    pid_position->err[LAST] = pid_position->err[NOW];
/***************************************	内环	******************************************/
		pid_pit_Iner.err[NOW] = pid_position->pos_out-imu_data.gx;
		pid_pit_Iner.pout = pid_pit_Iner.p * pid_pit_Iner.err[NOW];
		pid_pit_Iner.iout += pid_pit_Iner.i * pid_pit_Iner.err[NOW];
		abs_limit(&(pid_pit_Iner.iout), pid_pit_Iner.IntegralLimit);
		pid_pit_Iner.dout = pid_pit_Iner.d * (pid_pit_Iner.err[NOW] - pid_pit_Iner.err[LAST] );
		pid_pit_Iner.pos_out = pid_pit_Iner.pout + pid_pit_Iner.iout + pid_pit_Iner.dout;
		abs_limit(&(pid_pit_Iner.pos_out), pid_pit_Iner.MaxOutput);
		pid_pit_Iner.err[LAST] = pid_pit_Iner.err[NOW];
/***************************************	返回	******************************************/		
	  pid_position->pos_out = pid_pit_Iner.pos_out;
    return  pid_position->pos_out;	
}
//登岛180度旋转双环
float pid_calc_3508_180(moto_measure_t* m3508ang, pid_t* pidO, pid_t* pidI, float set)
{	
/***************************************	外环	******************************************/	
    pidO->get[NOW] = m3508ang->total_angle;
    pidO->set[NOW] = set;
    pidO->err[NOW] = pidO->set[NOW] - pidO->get[NOW];	//set - measure  


		pidO->pout = pidO->p * pidO->err[NOW];
		pidO->iout += pidO->i * pidO->err[NOW];
		abs_limit(&(pidO->iout), pidO->IntegralLimit);
		pidO->dout = -pidO->d * (pidO->err[NOW] - pidO->err[LAST]);						        
		pidO->pos_out = pidO->pout + pidO->iout + pidO->dout;
		abs_limit(&(pidO->pos_out), pidO->MaxOutput);

    pidO->err[LAST] = pidO->err[NOW];
/***************************************	内环	******************************************/
		pidI->err[NOW] = pidO->pos_out - m3508ang->speed_rpm;
	
		pidI->pout = pidI->p * pidI->err[NOW];
        pidI->iout += pidI->i * pidI->err[NOW];
		abs_limit(&(pidI->iout), pidI->IntegralLimit);
		pidI->dout = pidI->d * (pidI->err[NOW] - pidI->err[LAST]);
		pidI->pos_out = pidI->pout + pidI->iout + pidI->dout;
		abs_limit(&(pidI->pos_out), pidI->MaxOutput);
		//abs_sp(&(pidI->pos_out), 300, 150);

    pidI->err[LAST] = pidI->err[NOW];
/***************************************	返回	******************************************/		
//	pidO->pos_out = pidI->pos_out;
//    return  pidO->pos_out;	
		return	pidI->pos_out;
}
//拨弹电机45度双环
float pid_calc_toggle_ang(moto_measure_t* mtoggle, pid_t* pidO, pid_t* pidI, float set)
{	
/***************************************	外环	******************************************/	
    pidO->get[NOW] = mtoggle->total_angle;
    pidO->set[NOW] = set;
    pidO->err[NOW] = pidO->set[NOW] - pidO->get[NOW];	//set - measure  


		pidO->pout = pidO->p * pidO->err[NOW];
		pidO->iout += pidO->i * pidO->err[NOW];
		abs_limit(&(pidO->iout), pidO->IntegralLimit);
		pidO->dout = -pidO->d * (pidO->err[NOW] - pidO->err[LAST]);						        
		pidO->pos_out = pidO->pout + pidO->iout + pidO->dout;
		abs_limit(&(pidO->pos_out), pidO->MaxOutput);

    pidO->err[LAST] = pidO->err[NOW];
/***************************************	内环	******************************************/
		pidI->err[NOW] = pidO->pos_out - mtoggle->speed_rpm;
	
		pidI->pout = pidI->p * pidI->err[NOW];
        pidI->iout += pidI->i * pidI->err[NOW];
		abs_limit(&(pidI->iout), pidI->IntegralLimit);
		pidI->dout = pidI->d * (pidI->err[NOW] - pidI->err[LAST]);
		pidI->pos_out = pidI->pout + pidI->iout + pidI->dout;
		abs_limit(&(pidI->pos_out), pidI->MaxOutput);
		//abs_sp(&(pidI->pos_out), 300, 150);

    pidI->err[LAST] = pidI->err[NOW];
/***************************************	返回	******************************************/		
//	pidO->pos_out = pidI->pos_out;
//    return  pidO->pos_out;	
		return	pidI->pos_out;
}
/**
    *@bref. special calculate position PID @attention @use @gyro data!!
    *@param[in] set： target
    *@param[in] real	measure
    */
float pid_sp_calc(pid_t* pid, float get, float set)
{
    pid->get[NOW] = get;
    pid->set[NOW] = set;
    pid->err[NOW] = set - get;	//set - measure
    
    
    
        pid->pout = pid->p * pid->err[NOW];
        pid->dout = -pid->d * imu_data.gx/100.0f;	
        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->pos_out = pid->pout + pid->iout + pid->dout;
        abs_limit(&(pid->pos_out), pid->MaxOutput);
        pid->last_pos_out = pid->pos_out;	//update last time 
    

    return pid->pos_out;
}
/*pid总体初始化-----------------------------------------------------------------*/
void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd)
{
    /*init function pointer*/
    pid->f_param_init = pid_param_init;
    pid->f_pid_reset = pid_reset;
//	pid->f_cal_pid = pid_calc;	
//	pid->f_cal_sp_pid = pid_sp_calc;	//addition
		
    /*init pid param */
    pid->f_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd);
	
}


//pid_t pid_rol = {0};
//pid_t pid_pit = {0};
//pid_t pid_yaw = {0};
//pid_t pid_yaw_omg = {0};//角速度环
//pid_t pid_pit_omg = {0};//角速度环
//pid_t pid_yaw_alfa = {0};		//angle acce

//pid_t pid_chassis_angle={0};
//pid_t pid_poke = {0};
//pid_t pid_poke_omg = {0};
//pid_t pid_imu_tmp;
//pid_t pid_x;
//pid_t pid_cali_bby;
//pid_t pid_cali_bbp;

pid_t pid_omg;
pid_t pid_pos;
pid_t pid_spd[4];


void pid_test_init(){
	

	//为了解决上位机调参的时候第一次赋值的时候清零其他参数， 应该提前把参数表填充一下！

}
