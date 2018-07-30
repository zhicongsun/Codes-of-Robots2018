#ifndef __pid_H
#define __pid_H
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "mytype.h"
#include <math.h>
#include "m_moto.h"

#define ABS(x)		((x>0)? (x): (-x)) 
//丝杠
extern float P_cyl_spd,I_cyl_spd,D_cyl_spd;
extern float P_cyl_ang,I_cyl_ang,D_cyl_ang; 

extern float P_3508_spd,I_3508_spd,D_3508_spd;
//普通pitch
extern float P_6623_pit_ang,I_6623_pit_ang,D_6623_pit_ang;
extern float P_6623_pit_spd,I_6623_pit_spd,D_6623_pit_spd;

//自动pitch
extern float autu_P_6623_pit_ang,autu_I_6623_pit_ang,autu_D_6623_pit_ang;
//随动yaw
extern float P_6623_yaw_ang,I_6623_yaw_ang,D_6623_yaw_ang;
extern float P_yaw_spd,I_yaw_spd,D_yaw_spd;
//自动瞄准yaw
extern float P_autu_6623_yaw_ang,I_autu_6623_yaw_ang,D_autu_6623_yaw_ang;
//拨弹
extern float P_tog_ang,I_tog_ang,D_tog_ang;
extern float P_tog_spd,I_tog_spd,D_tog_spd;
//extern float P_Toggler_spd,I_Toggler_spd,D_Toggler_spd;
extern float P_Power,I_Power,D_Power;
extern float P_Follow,I_Follow,D_Follow;

enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
    
    POSITION_PID,
    DELTA_PID,
};
typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
    float set[3];				//目标值,包含NOW， LAST， LLAST上上次
    float get[3];				//测量值
    float err[3];				//误差
	
    
    float pout;							//p输出
    float iout;							//i输出
    float dout;							//d输出
    
    float pos_out;						//本次位置式输出
    float last_pos_out;				//上次输出
    float delta_u;						//本次增量值
    float delta_out;					//本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;
    
	  float max_err;
	  float deadband;				//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;				//输出限幅
    uint32_t IntegralLimit;		//积分限幅
    
    void (*f_param_init)(struct __pid_t *pid,  //PID参数初始化
                    uint32_t pid_mode,
                    uint32_t maxOutput,
                    uint32_t integralLimit,
                    float p,
                    float i,
                    float d);
    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid三个参数修改

}pid_t;


extern pid_t Power_pid;
extern pid_t Heat_pid;
extern pid_t Cmsqinspd;
extern pid_t Cm3508_spd_pid[];
extern pid_t Gm6623_pit_spd_pid;
extern pid_t Gm6623_pit_ang_pid;//随动pitch
extern pid_t autu_Gm6623_pit_ang_pid;//自动瞄准pitch

extern pid_t Gm6623_yaw_ang_pid;//随动位置环yaw
extern pid_t autu_Gm6623_yaw_ang_pid;//自动瞄准yaw
extern pid_t Gm6623_yaw_spd_pid;//随动速度环yaw

extern pid_t Toggler_spd_pid;

extern pid_t Follw_3508_pid;//随动的3508pid

extern void PID_struct_init(
    pid_t* pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,
    
    float 	kp, 
    float 	ki, 
    float 	kd);
    
extern float pid_calc(pid_t* pid, float fdb, float ref);
extern float pid_calc_pitch(pid_t* pidO, float get, float set);
extern void pid_reset(pid_t	*pid, float kp, float ki, float kd);
//随动yaw
extern float pid_calc_yaw(pid_t* pid_position, float get, float set);
//自动瞄准yaw
extern float autu_pid_calc_yaw(pid_t* pid_position, float get, float set);
//随动pitch
extern float pid_calc_pit(pid_t* pid_position, float get, float set);
//自动pitch
extern float autu_pid_calc_pitch(pid_t* pid_position, float get, float set);

extern float pid_follow_calc(pid_t* pid_position, float get, float set);

extern float pid_calc_toggle_ang(moto_measure_t* mtoggle, pid_t* pidO, pid_t* pidI, float set);

extern pid_t pid_Toggler_ang;
extern pid_t pid_Toggler_spd;
		
extern pid_t pid_rol;
extern pid_t pid_pit;
extern pid_t pid_yaw;
extern pid_t pid_pit_omg;
extern pid_t pid_yaw_omg;	
extern pid_t pid_spd[4];
extern pid_t pid_yaw_alfa;
extern pid_t pid_chassis_angle;
extern pid_t pid_poke;
extern pid_t pid_poke_omg;
extern pid_t pid_imu_tmp;		//imu_temperature
extern pid_t pid_cali_bby;	//big buff yaw
extern pid_t pid_cali_bbp;
extern pid_t pid_omg;
extern pid_t pid_pos;

#endif

