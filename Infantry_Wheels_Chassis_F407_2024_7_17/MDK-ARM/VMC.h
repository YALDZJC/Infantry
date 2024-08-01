#pragma once

#include "stdint.h"
#include "math.h"

#define pi 3.14159265352979

typedef struct
{
	/*左右两腿的公共参数，固定不变*/
	float l5;//AE长度 //单位为m
	float	l1;//单位为m
	float l2;//单位为m
	float l3;//单位为m
	float l4;//单位为m
	
	float XB,YB;//B点的坐标
	float XD,YD;//D点的坐标
	
	float XC,YC;//C点的直角坐标
	float L0,phi0;//C点的极坐标
	float alpha;
	float d_alpha;	
	
	float lBD;//BD两点的距离
	
	float d_phi0;//现在C点角度phi0的变换率
	float last_phi0;//上一次C点角度，用于计算角度phi0的变换率d_phi0

	float A0,B0,C0;//中间变量
	float phi2,phi3;
	float phi1,phi4;
	
	float j11,j12,j21,j22;//笛卡尔空间力到关节空间的力的雅可比矩阵系数
	float torque_set[2];

	float F0;
	float Tp;
	
	float theta;
	float d_theta;//theta的一阶导数
	float last_d_theta;
	float dd_theta;//theta的二阶导数
	
	float d_L0;//L0的一阶导数
	float dd_L0;//L0的二阶导数
	float last_L0;
	float last_d_L0;
	
	float FN;//支持力
	
	uint8_t first_flag;
	uint8_t leg_flag;//腿长完成标志
}VMC_t;

class VMC_leg_t
{
public:
	VMC_t VMC_data;

	VMC_leg_t(float l1, float l2, float l3, float l4, float l5)
	{
        VMC_data.l1 = l1;
        VMC_data.l2 = l2;
        VMC_data.l3 = l3;
        VMC_data.l4 = l4;
        VMC_data.l5 = l5;
	}

	void VMC_leg_t::Up_Left(float pitch_Angle, float pitch_Gyro, float dt);
	void VMC_leg_t::Up_Right(float pitch_Angle, float pitch_Gyro, float dt);
	void VMC_leg_t::Jacobian();

   uint8_t VMC_leg_t::ground_detection_L();
   uint8_t VMC_leg_t::ground_detection_R();

};

void VMC_leg_t::Up_Left(float pitch_Angle, float pitch_Gyro, float dt)
{
    static float Pitch_L=0.0f;
	static float Pith_GyroL=0.0f;
	Pitch_L = pitch_Angle;
	Pith_GyroL = pitch_Gyro;

	this->VMC_data.YD = this->VMC_data.l4 * sin(this->VMC_data.phi4);//D的y坐标
	this->VMC_data.YB = this->VMC_data.l1 * sin(this->VMC_data.phi1);//B的y坐标
	this->VMC_data.XD = this->VMC_data.l5 + this->VMC_data.l4 * cos(this->VMC_data.phi4);//D的x坐标
	this->VMC_data.XB = this->VMC_data.l1 * cos(this->VMC_data.phi1); //B的x坐标
			
	this->VMC_data.lBD = sqrt((this->VMC_data.XD - this->VMC_data.XB)*(this->VMC_data.XD - this->VMC_data.XB) + (this->VMC_data.YD - this->VMC_data. YB) * (this->VMC_data.YD - this->VMC_data.YB));
	
	this->VMC_data.A0 = 2*this->VMC_data.l2 * (this->VMC_data.XD - this->VMC_data.XB);
	this->VMC_data.B0 = 2*this->VMC_data.l2 * (this->VMC_data.YD - this->VMC_data.YB);
	this->VMC_data.C0 = this->VMC_data.l2 * this->VMC_data.l2 + this->VMC_data.lBD*this->VMC_data.lBD - this->VMC_data.l3 * this->VMC_data.l3;
	this->VMC_data.phi2 = 2*atan2f((this->VMC_data.B0 + sqrt(this->VMC_data.A0 * this->VMC_data.A0 + this->VMC_data.B0 * this->VMC_data.B0 - this->VMC_data.C0 * this->VMC_data.C0)), this->VMC_data.A0 + this->VMC_data.C0);			
	this->VMC_data.phi3 = atan2f(this->VMC_data.YB - this->VMC_data.YD + this->VMC_data.l2 * sin(this->VMC_data.phi2), this->VMC_data.XB - this->VMC_data.XD + this->VMC_data.l2 * cos(this->VMC_data.phi2));
	//C点直角坐标
	this->VMC_data.XC = this->VMC_data.l1 * cos(this->VMC_data.phi1) + this->VMC_data.l2 * cos(this->VMC_data.phi2);
	this->VMC_data.YC = this->VMC_data.l1 * sin(this->VMC_data.phi1) + this->VMC_data.l2  *sin(this->VMC_data.phi2);
	//C点极坐标
	this->VMC_data.L0 = sqrt((this->VMC_data.XC - this->VMC_data.l5/2.0f) * (this->VMC_data.XC - this->VMC_data.l5/2.0f) + this->VMC_data.YC * this->VMC_data.YC);
		
	this->VMC_data.phi0 = atan2f(this->VMC_data.YC,(this->VMC_data.XC - this->VMC_data.l5/2.0f));//phi0用于计算lqr需要的theta		
	this->VMC_data.alpha = pi/2.0f-this->VMC_data.phi0 ;
		
	if(this->VMC_data.first_flag == 0)
	{
		this->VMC_data.last_phi0 = this->VMC_data.phi0 ;
		this->VMC_data.first_flag = 1;
	}

	this->VMC_data.d_phi0 = (this->VMC_data.phi0 - this->VMC_data.last_phi0)/dt;//计算phi0变化率，d_phi0用于计算lqr需要的d_theta
	this->VMC_data.d_alpha = 0.0f - this->VMC_data.d_phi0 ;
		
	this->VMC_data.theta = pi/2.0f-Pitch_L - this->VMC_data.phi0;//得到状态变量1
	this->VMC_data.d_theta = (-Pith_GyroL - this->VMC_data.d_phi0);//得到状态变量2
		
	this->VMC_data.last_phi0 = this->VMC_data.phi0 ;
    
	this->VMC_data.d_L0=(this->VMC_data.L0 - this->VMC_data.last_L0)/dt;//腿长L0的一阶导数
    this->VMC_data.dd_L0=(this->VMC_data.d_L0 - this->VMC_data.last_d_L0)/dt;//腿长L0的二阶导数
		
	this->VMC_data.last_d_L0 = this->VMC_data.d_L0;
	this->VMC_data.last_L0 = this->VMC_data.L0;
		
	this->VMC_data.dd_theta = (this->VMC_data.d_theta - this->VMC_data.last_d_theta)/dt;
	this->VMC_data.last_d_theta = this->VMC_data.d_theta;
}

void VMC_leg_t::Up_Right(float pitch_Angle, float pitch_Gyro, float dt)
{
    static float Pitch_R=0.0f;
	static float Pith_GyroR=0.0f;
	Pitch_R = pitch_Angle;
	Pith_GyroR = pitch_Gyro;

	this->VMC_data.YD = this->VMC_data.l4 * sin(this->VMC_data.phi4);//D的y坐标
	this->VMC_data.YB = this->VMC_data.l1 * sin(this->VMC_data.phi1);//B的y坐标
	this->VMC_data.XD = this->VMC_data.l5 + this->VMC_data.l4 * cos(this->VMC_data.phi4);//D的x坐标
	this->VMC_data.XB = this->VMC_data.l1 * cos(this->VMC_data.phi1); //B的x坐标
			
	this->VMC_data.lBD = sqrt((this->VMC_data.XD - this->VMC_data.XB)*(this->VMC_data.XD - this->VMC_data.XB) + (this->VMC_data.YD - this->VMC_data. YB) * (this->VMC_data.YD - this->VMC_data.YB));
	
	this->VMC_data.A0 = 2*this->VMC_data.l2 * (this->VMC_data.XD - this->VMC_data.XB);
	this->VMC_data.B0 = 2*this->VMC_data.l2 * (this->VMC_data.YD - this->VMC_data.YB);
	this->VMC_data.C0 = this->VMC_data.l2 * this->VMC_data.l2 + this->VMC_data.lBD*this->VMC_data.lBD - this->VMC_data.l3 * this->VMC_data.l3;
	this->VMC_data.phi2 = 2*atan2f((this->VMC_data.B0 + sqrt(this->VMC_data.A0 * this->VMC_data.A0 + this->VMC_data.B0 * this->VMC_data.B0 - this->VMC_data.C0 * this->VMC_data.C0)), this->VMC_data.A0 + this->VMC_data.C0);			
	this->VMC_data.phi3 = atan2f(this->VMC_data.YB - this->VMC_data.YD + this->VMC_data.l2 * sin(this->VMC_data.phi2), this->VMC_data.XB - this->VMC_data.XD + this->VMC_data.l2 * cos(this->VMC_data.phi2));
	//C点直角坐标
	this->VMC_data.XC = this->VMC_data.l1 * cos(this->VMC_data.phi1) + this->VMC_data.l2 * cos(this->VMC_data.phi2);
	this->VMC_data.YC = this->VMC_data.l1 * sin(this->VMC_data.phi1) + this->VMC_data.l2  *sin(this->VMC_data.phi2);
	//C点极坐标
	this->VMC_data.L0 = sqrt((this->VMC_data.XC - this->VMC_data.l5/2.0f) * (this->VMC_data.XC - this->VMC_data.l5/2.0f) + this->VMC_data.YC * this->VMC_data.YC);
		
	this->VMC_data.phi0 = atan2f(this->VMC_data.YC,(this->VMC_data.XC - this->VMC_data.l5/2.0f));//phi0用于计算lqr需要的theta		
	this->VMC_data.alpha = pi/2.0f-this->VMC_data.phi0 ;
		
	if(this->VMC_data.first_flag == 0)
	{
		this->VMC_data.last_phi0 = this->VMC_data.phi0 ;
		this->VMC_data.first_flag = 1;
	}

	this->VMC_data.d_phi0 = (this->VMC_data.phi0 - this->VMC_data.last_phi0)/dt;//计算phi0变化率，d_phi0用于计算lqr需要的d_theta
	this->VMC_data.d_alpha = 0.0f - this->VMC_data.d_phi0 ;
		
	this->VMC_data.theta = pi/2.0f-Pitch_R - this->VMC_data.phi0;//得到状态变量1
	this->VMC_data.d_theta = (-Pith_GyroR - this->VMC_data.d_phi0);//得到状态变量2
		
	this->VMC_data.last_phi0 = this->VMC_data.phi0 ;
    
	this->VMC_data.d_L0=(this->VMC_data.L0 - this->VMC_data.last_L0)/dt;//腿长L0的一阶导数
    this->VMC_data.dd_L0=(this->VMC_data.d_L0 - this->VMC_data.last_d_L0)/dt;//腿长L0的二阶导数
		
	this->VMC_data.last_d_L0 = this->VMC_data.d_L0;
	this->VMC_data.last_L0 = this->VMC_data.L0;
		
	this->VMC_data.dd_theta = (this->VMC_data.d_theta - this->VMC_data.last_d_theta)/dt;
	this->VMC_data.last_d_theta = this->VMC_data.d_theta;
}

void VMC_leg_t::Jacobian()
{
	this->VMC_data.j11 = (this->VMC_data.l1 * sin(this->VMC_data.phi0 - this->VMC_data.phi3) * sin(this->VMC_data.phi1 - this->VMC_data.phi2)) / sin(this->VMC_data.phi3-this->VMC_data.phi2);
	this->VMC_data.j12 = (this->VMC_data.l1 * cos(this->VMC_data.phi0 - this->VMC_data.phi3) * sin(this->VMC_data.phi1 - this->VMC_data.phi2)) / (this->VMC_data.L0 * sin(this->VMC_data.phi3 - this->VMC_data.phi2));
	this->VMC_data.j21 = (this->VMC_data.l4 * sin(this->VMC_data.phi0 - this->VMC_data.phi2) * sin(this->VMC_data.phi3 - this->VMC_data.phi4)) / sin(this->VMC_data.phi3-this->VMC_data.phi2);
	this->VMC_data.j22 = (this->VMC_data.l4 * cos(this->VMC_data.phi0 - this->VMC_data.phi2) * sin(this->VMC_data.phi3 - this->VMC_data.phi4)) / (this->VMC_data.L0 * sin(this->VMC_data.phi3 - this->VMC_data.phi2));
	
	this->VMC_data.torque_set[0] = this->VMC_data.j11 * this->VMC_data.F0 + this->VMC_data.j12 * this->VMC_data.Tp;//得到RightFront的输出轴期望力矩，F0为五连杆机构末端沿腿的推力 
	this->VMC_data.torque_set[1] = this->VMC_data.j21 * this->VMC_data.F0 + this->VMC_data.j22 * this->VMC_data.Tp;//得到RightBack的输出轴期望力矩，Tp为沿中心轴的力矩 
}

uint8_t VMC_leg_t::ground_detection_R()
{
	this->VMC_data.FN = this->VMC_data.F0 * cos(this->VMC_data.theta)+this->VMC_data.Tp*sin(this->VMC_data.theta)/this->VMC_data.L0+6.0f;//腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
//	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0
//+0.6f*(ins->MotionAccel_n[2]-vmc->dd_L0*arm_cos_f32(vmc->theta)+2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));
 
	if(this->VMC_data.FN<5.0f)
	{
        //离地了
	  return 1;
	}
	else
	{
	  return 0;	
	}
}

uint8_t VMC_leg_t::ground_detection_L()
{
	this->VMC_data.FN = this->VMC_data.F0 * cos(this->VMC_data.theta)+this->VMC_data.Tp*sin(this->VMC_data.theta)/this->VMC_data.L0+6.0f;//腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
//	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0
//+0.6f*(ins->MotionAccel_n[2]-vmc->dd_L0*arm_cos_f32(vmc->theta)+2.0f*vmc->d_L0*vmc->d_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->dd_theta*arm_sin_f32(vmc->theta)+vmc->L0*vmc->d_theta*vmc->d_theta*arm_cos_f32(vmc->theta));
 
	if(this->VMC_data.FN<5.0f)
	{
        //离地了
	  return 1;
	}
	else
	{
	  return 0;	
	}
}

float LQR_K_calc(float *coe,float len)
{
  return coe[0]*len*len*len+coe[1]*len*len+coe[2]*len+coe[3];
}
