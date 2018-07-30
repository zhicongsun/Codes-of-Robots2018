#ifndef _T_MOTO_H
#define _T_MOTO_H

#define ANGLE_MAX 1200//1300
#define ANGLE_MIN 200//100
//yaw左转~gz++,ang++，pitch下转~gx++,ang++
#define ImuGzErr 13
#define yaw6623_MiddleAngle 3750
#define yaw6623_AngleAdjustLimit 1500
#define ImuGxErr 0
#define pit6623_MiddleAngle 5500
#define pit6623_AngleAdjustLimit 500

#define yawimu_AngleAdjustK 1

typedef struct Follow_6623_imu_fbangall
{
	int Velocity[3];
	int Angle[3];
	
}Follow_6623_imu_fbangall_t;
//随动
extern Follow_6623_imu_fbangall_t follow_6623yaw_imu_data;
extern int yaw6623_AngleAdjust;
extern int yawimu_AngleAdjust;
extern float pitimu_AngleAdjust;
//自动瞄准
extern Follow_6623_imu_fbangall_t autu_6623yaw_imu_data;
extern int autu_yaw6623_AngleAdjust;
extern int autu_yawimu_AngleAdjust;
extern float autu_Gm6623_yaw_ang_set;

extern float Cm3508_ang_set[];
extern float Cm3508_spd_get[4];
extern float Cm3508_spd_xset[4];//x方向分量速度
extern float Cm3508_spd_yset[4];//y方向分量速度
extern float Cm3505_spd_sqin[4];//自旋运动
extern float Cm3508_spd_set[4];//合成速度
extern float Cm3508_spd_offset[4];

extern float Gm6623_yaw_ang_set;
extern float Gm6623_yaw_ang_give;
extern float Gm6623_yaw_spd_set;

extern float Gm6623_pit_ang_set;
extern float Gm6623_pit_ang_give;
extern float Gm6623_pit_spd_set;

//随动参数
extern float Follow_6623_imu_fbang;//本次云台跟随反馈角
extern float Follow_6623_imu_lastfbang;//上次云台跟随反馈角
extern float Cm3508_total_set[4];//总pid输出=底盘运动的pid输出+随动pid输出


extern void Cm3508_spd_ctr(void);
extern void Gm6623_pitch_ang_ctr(void);
extern void Gm6623_yaw_ang_ctr(void);
extern void CM_follow_gy_dataprocess(void);
void autu_CM3508_dataprocess(void);


extern void GM6623_yaw_follow_ctr(void);
extern void GM6623_yaw_autu_ctr(void);

#endif

