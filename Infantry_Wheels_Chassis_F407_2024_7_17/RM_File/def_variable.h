#pragma once

#include "RM_StaticTime.h"//静态定时器
#include "RM_Clicker.h"//遥控器
#include "RM_PID.h"
#include "ladrc.h"
#include "RM_Motor.h"//RM电机
#include "RM_Can.h"
#include "RM_stm32fxxx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "RM_Wheel.h"
#include "RM_PM01.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdarg.h>
#include "RM_RefereeSystem.h"
#include "DM.h"
/*
														操作控制策划														

x代表无效
y代表有效
数字代表值

左手                                     右手
ch3:前后			ch2:左右			s1开关132			ch1:前后			ch0:左右			s2开关132
y             x            1             x             y            1             控底盘前后左右,不控云台,不改表俯仰角（1）
x             x            1             y             y            3             控云台yaw/pitch,不控底盘（2）
x							x						 1						 x						 x					  2							待定（3）

y             x            3             y             y            1             底盘跟随云台,俯仰角5度（4）
y             x            3             y             y            3             底盘跟随云台,俯仰角15度（5）
x             x            3             x             x            2             待定（6）

x							x						 2						 x						 x						1							待定（7）
x							x						 2						 x						 x						3							待定（8）
x							x						 2						 x						 x						2							急停（9）
*/

/*遥控器信号源切换*/
/*
	CONTROL_SIG 0 遥控器
	CONTROL_SIG 1 上下板
*/
#define CONTROL_SIG 1
#if CONTROL_SIG == 0
	#define chassis_follow_gimbal (RM_Clicker::RC_Ctl.rc.s1 == 1 && RM_Clicker::RC_Ctl.rc.s2 == 1)//（1）
	#define stop_mode (RM_Clicker::RC_Ctl.rc.s1 == 2 && RM_Clicker::RC_Ctl.rc.s2 == 2)//（9）
	#define chassis_RC_LX (RC_LX)//左手x
	#define chassis_RC_LY (RC_LY)//左手y
#elif CONTROL_SIG == 1
	#define chassis_RC_LX (Gimbal_to_Chassis_Data.int16_RC_LX)//左手x
	#define chassis_RC_LY (Gimbal_to_Chassis_Data.int16_RC_LY)//左手y
	#define stop_mode (RM_Clicker::RC_Ctl.rc.s1 == 2 && RM_Clicker::RC_Ctl.rc.s2 == 2)//（9）
#endif

#define YAW_E (Gimbal_to_Chassis_Data.yaw_encoder_e)//底盘跟随云台误差
#define CHASSIS_FOLLOW_GIMBAL (Gimbal_to_Chassis_Data.chassis_follow_gimbal)//底盘跟随云台模式
#define GIMBAL_HOST_CHASSIS (Gimbal_to_Chassis_Data.gimbal_host_chassis)//底盘跟随云台模式
#define CHASSIS_GYRO (Gimbal_to_Chassis_Data.chassis_gyro)//底盘跟随云台模式
#define CHASSIS_TOP (Gimbal_to_Chassis_Data.chassis_top)//底盘跟随云台模式
#define MAX_CHASSIS_SPEED 16384.0//最大速度
#define CHASSIS_SPEED_ZOOM_VW 1.0//放大因子
#define CHASSIS_SPEED_ZOOM_VXY 0.5//放大因子
#define GY_V_SET 600//小陀螺+-速度
#define W_V_Init 5000 //小陀螺初始转速
#define YD_V_SET 0.12//平移+-速度
#define YD_V_Init 1 //平移初始转速

#define Chassis_angle_Init_0x205 7088
#define Chassis_angle_Init_0x206 6445
#define Chassis_angle_Init_0x207 1622
#define Chassis_angle_Init_0x208 5093
/***************************变量声明*********************************/
//期望值
float tar_AGV_angle[4];
float tar_AGV_speed[4];
float Zero_crossing[4];

//串口打印
#define Send_Usart_Data_Huart huart1

//云台到底盘
#define Send_Gimbal_to_Chassis_Huart huart1
#define Send_Gimbal_to_Chassis_Huart_LEN 12

//底盘跟随云台旋转量
float cos_xita = 0,sin_xita = 0;

//旋转量
float vw = 0,kvw = 0.6;//kw旋转系数
float vx = 0,vy = 0;float vxy_kvw = 0;//旋转的时候降低转速保功率

//误差夹角
float yaw_e_xita = 0,yaw_e_radian = 0;

/***************************底盘*********************************/
SteeringWheel_t AGV_wheel;


bool switch_sin = 0;
bool is_ude = 0;
float sintt, sin_out,pitch_out, yaw_out, zero_yaw, tar_angle, last_sin, der_sin, sin_k;

float HZ = 0.1;



DM dm_yaw;
bool dm_is_open;//达妙启动标志位

uint8_t send_GIMBAL_motor_size;//用于3508与6020错开发送
uint32_t send_DM_motor_size;

RM_PID YAW_pid;			//YAW轴		6020
Kpid_t YAW_pid_Init(0.7, 0.5, 0);

RM_StaticTime Total_tasks_staticTime;//控制时间

WheatWheel_t wheel;
WheatWheel_t wheel_e;

float diagonally_factor = 0;//斜向因子
LADRC_quadratic wheel_ladrc_left_1 (TD_quadratic(50,0,0),8,400,60,16384);
LADRC_quadratic wheel_ladrc_left_2 (TD_quadratic(50,0,0),8,400,60,16384);
LADRC_quadratic wheel_ladrc_right_1(TD_quadratic(50,0,0),8,400,60,16384);
LADRC_quadratic wheel_ladrc_right_2(TD_quadratic(50,0,0),8,400,60,16384);

Kpid_t AGV_pid_Init(0.15, 0.003, 0);
Kpid_t AGV_angle_speed_Init(200, 0, 0);

TD_quadratic Speed_0x205(100);

RM_PID AGV_pid[4];
RM_PID AGV_angle_speed_pid[4];

UDE ude_chassis_0x205(0, 0, 0, 0);

RM_PID AGV_speed[4];
Kpid_t AGV_speed_Init(5, 0, 0);



TD_quadratic wheel_td_left_1 (150);
TD_quadratic wheel_td_left_2 (150);
TD_quadratic wheel_td_right_1(150);
TD_quadratic wheel_td_right_2(150);

TD_quadratic td_vw(10);
TD_quadratic td_vx(40);
TD_quadratic td_vy(40);
TD_quadratic td_zero(80);

float shift_vxy_zoom = 1;//留给shift冲刺模式，速度因子
uint32_t Send_ms;
/***************************云台到底盘数据*********************************/
struct Gimbal_to_Chassis_Data_t
{
	uint8_t head;//帧头
	
	bool notation_RC_LX;//RC_LX符号位
	uint8_t _RC_LX;//RC_LX
	
	bool notation_RC_LY;//RC_LY符号位
	uint8_t _RC_LY;//RC_LY
	
	int16_t yaw_encoder_angle_e;//底盘跟随云台误差
	int16_t yaw_encoder_e;
	
	int16_t int16_RC_LX;//转换之后的RC_LX
	int16_t int16_RC_LY;//转换之后的RC_LY
	
	bool notation_ER;//ER符号位，小陀螺+-速度
	uint8_t _ER;//ER
	
	int8_t int8_ER;//ER
	
	bool notation_ZX;//ZX符号位，底盘平移+-速度
	uint8_t _ZX;//ZX
	
	int8_t int8_ZX;//ZX
	
	bool notation_pitch_cai;//pitch符号位，小陀螺+-数度
	uint8_t _pitch_cai;//pitch
	
	int8_t int8_pitch_cai;//pitch
	
	bool chassis_follow_gimbal;//底盘跟随云台
	bool gimbal_host_chassis;//主云台模式
	bool chassis_gyro;//小陀螺
	bool chassis_top;//遥控器小陀螺
	bool up_ui;//刷新ui
	bool stop;//停止
	bool MCL_of;//摩擦轮是否开启
	bool CM_of;//仓门是否开启
	bool v_of;//视觉自瞄是否开启
	bool bp_of;//拨盘是否开启
	uint8_t pData[50];
	bool dir;
	RM_StaticTime dir_time;
};
//数据
Gimbal_to_Chassis_Data_t Gimbal_to_Chassis_Data; 

bool dir;

int16_t dianlu;
TD_quadratic dianlutd(100);
int16_t dianlupid;

/***************************UI*********************************/
RM_Key Ctrl_Key;//Ctrl键触发
RM_Key MCL_Key;//摩擦轮键触发
RM_Key CM_Key;//摩擦轮键触发
RM_Key ZM_Key;//视觉自瞄键触发
RM_Key BP_Key;//拨盘键触发
int8_t now_er_v,last_er_v;//转速,修改转速ui
int8_t now_zx_v,last_zx_v;//移速，修改移速ui
bool is_up_ui;//ui刷新标记
void set_send_graphic_queue_is_Delete_all();//设置删除全部图层
void set_gy_v(int Operate,float v);//修改转速字符串
void set_yd_v(int Operate,float v);//修改转速字符串
void set_mcl_of(int Operate,bool is);//修改摩擦开关
void set_cm_of(int Operate,bool is);//修改仓门开关
void set_bp_of(int Operate,bool is);//修改拨盘开关
void set_cd_of(int Operate,float cd);//超电当前电压
void set_zm_of(int Operate,bool is);//修改自瞄开关
void darw_graphic_static_ui_init();//静态ui初始化
void darw_graphic_ui();//绘制ui
