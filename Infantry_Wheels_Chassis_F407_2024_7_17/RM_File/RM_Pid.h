#pragma once
#include "RM_stdxxx.h"
#include "RM_Filter.h"
#include "ladrc.h"

//调参kp,ki,kd结构体
struct Kpid_t
{
	double kp,ki,kd;
	Kpid_t(double kp = 0,double ki = 0,double kd = 0)
		:kp(kp),ki(ki),kd(kd)
	{}
		Kpid_t(Kpid_t* kpid)
		:kp(kpid->kp),ki(kpid->ki),kd(kpid->kd)
	{}
};

//修正kpid
Kpid_t AddKpid(Kpid_t Kpid,double kp = 0,double ki = 0,double kd = 0)
{		
	return Kpid_t(Kpid.kp + kp,Kpid.ki + ki,Kpid.kd + kd);
}

typedef struct
{
    //期望，实际
    double cin,cout,feedback;
    //p,i,d计算
    double p,i,d;
		//Delta,p,i,d计算
    double Dp,Di,Dd;
    //误差
    double last_e,last_last_e,now_e,IerrorA,IerrorB;
		//分频频率
		unsigned char Frequency,_Frequency_;
		//td跟踪微分器，跟踪误差
		TD_quadratic td_e;
		//限幅
		double MixI;
}Pid_t;

class RM_PID
{
private:
  /* data */
public:
	Pid_t pid;
	RM_PID()
	{
		this->pid.td_e.r = 100;
	}
	//积分上限和变速积分
	void SetMixI(double maxi,double IerrorA,double IerrorB);
	//分频设置
	void SetFrequency(double Frequency);
	//位置式pid获取
	double GetPidPos(Kpid_t kpid,double cin,double feedback,double max);
	//清除
	void clearPID();
	//清除增量
	void PidRstDelta();
	//增量式pid获取
	double GetPidDelta(Kpid_t kpid,double cin,double feedback,double max);
};

inline void RM_PID::SetMixI(double maxi, double IerrorA, double IerrorB)
{
  this->pid.MixI = maxi;
	this->pid.IerrorA = IerrorA;
	this->pid.IerrorB = IerrorB;
}

inline void RM_PID::SetFrequency(double Frequency)
{
  this->pid.Frequency = Frequency;
}

inline double RM_PID::GetPidPos(Kpid_t kpid, double cin, double feedback, double max)
{
  //分频比较
	if(this->pid.Frequency > this->pid._Frequency_){this->pid._Frequency_++;return this->pid.cout;}
	//分频计数
	this->pid._Frequency_ = 0;
	//输入
	this->pid.cin = cin;
	//反馈
	this->pid.feedback = feedback;
	//跟踪误差
	this->pid.td_e.td_quadratic(cin - feedback);
   //输入误差
   this->pid.now_e = this->pid.td_e.x1;
   //p值
   this->pid.p = kpid.kp * this->pid.now_e;
   //变速积分
//	if(fabs(this->pid.now_e) <= this->pid.IerrorB)
//	{
//		this->pid.Di = this->pid.now_e * 0.001;
//	}
//	if((this->pid.IerrorB < fabs(this->pid.now_e)) & (fabs(this->pid.now_e) <= this->pid.IerrorA + this->pid.IerrorB))
//	{
//		this->pid.Di = (this->pid.IerrorA - fabs(this->pid.now_e) + this->pid.IerrorB) / this->pid.IerrorA * this->pid.now_e * 0.001;
//	}
//	if(fabs(this->pid.now_e) > (this->pid.IerrorA + this->pid.IerrorB))
//	{
//		this->pid.Di = 0;
//	}
	//积分计算
	this->pid.Di += pid.now_e;
	this->pid.i = this->pid.Di * kpid.ki;
	//积分限幅
	if(this->pid.i > this->pid.MixI) this->pid.i = this->pid.MixI;
	if(this->pid.i < -this->pid.MixI) this->pid.i = -this->pid.MixI;
  //d值
  this->pid.d = kpid.kd * this->pid.td_e.x2;
//   //上一次误差
//   this->pid.last_e = this->pid.now_e;
	//清除积分输出
	if(kpid.ki == 0.0f) this->pid.i = 0;
   //输出值
   this->pid.cout = this->pid.p + this->pid.i + this->pid.d;
   //pid限幅
   if(this->pid.cout > max) this->pid.cout = max;
   if(this->pid.cout < -max) this->pid.cout = -max;
   return this->pid.cout;
}

inline void RM_PID::clearPID()
{
	this->pid.p = 0;
	this->pid.i = 0;
	this->pid.d = 0;
	this->pid.cout = 0;
}

inline void RM_PID::PidRstDelta()
{
	this->pid.p = this->pid.i = this->pid.d = 0;
	this->pid.Dp = this->pid.Di = this->pid.Dd = 0;
}

inline double RM_PID::GetPidDelta(Kpid_t kpid, double cin, double feedback, double max)
{
	//分频比较
	if(this->pid.Frequency > this->pid._Frequency_){this->pid._Frequency_++;return this->pid.cout;}
	//分频计数
	this->pid._Frequency_ = 0;
	//输入
	this->pid.cin = cin;
	//反馈
	this->pid.feedback = feedback;
	//输入误差
  this->pid.now_e = this->pid.cin - this->pid.feedback;
	//Delta,p计算
	this->pid.Dp = kpid.kp * (this->pid.now_e - this->pid.last_e);
	//Delta,i计算
	//变速积分
	if(fabs(this->pid.now_e) <= this->pid.IerrorB)this->pid.Di = 1 * this->pid.now_e * 0.001;
	if((this->pid.IerrorB < fabs(this->pid.now_e)) & (fabs(this->pid.now_e) <= this->pid.IerrorA + this->pid.IerrorB))
		this->pid.Di = (this->pid.IerrorA - fabs(this->pid.now_e) + this->pid.IerrorB) / this->pid.IerrorA * this->pid.now_e * 0.001;
	if(fabs(this->pid.now_e) > (this->pid.IerrorA + this->pid.IerrorB))this->pid.Di = 0 * this->pid.now_e * 0.001;
	//Delta,d计算
	this->pid.Dd = kpid.kd * (this->pid.now_e - 2 * this->pid.last_e + this->pid.last_last_e);
	//增量p计算
	this->pid.p += this->pid.Dp;
	//增量i计算
	this->pid.i += this->pid.Di * kpid.ki;
	//积分限幅
	if(this->pid.i > this->pid.MixI) this->pid.i = this->pid.MixI;
	if(this->pid.i < -this->pid.MixI) this->pid.i = -this->pid.MixI;
	//增量d计算
  this->pid.d += this->pid.Dd;
	//上上一次误差
  this->pid.last_last_e = this->pid.last_e;
	//上一次误差
  this->pid.last_e = this->pid.now_e;
	//清除积分输出
	if(kpid.ki == 0.0f) this->pid.i = 0;
	//输出值
	this->pid.cout = this->pid.p + this->pid.i + this->pid.d;
	//pid限幅
	if(this->pid.cout > max) this->pid.cout = max;
	if(this->pid.cout < -max) this->pid.cout = -max;
	return this->pid.cout;
}

//前馈系统
struct FeedForward_t
{
	//上一次目标
	float last_target;
	//输出
	float cout,cout_k;
	//目标误差
	float target_e,max_target_e;
	//衰减时间
	float t,dt,k,max_cout;	
	float s;
	FeedForward_t(float dt,float k,float max_target_e,float max_cout,float cout_k)
	:dt(dt),k(k),max_target_e(max_target_e),max_cout(max_cout),cout_k(cout_k){}
};
class RM_FeedForward
{
public:
	FeedForward_t feedForward;
	RM_FeedForward(float dt = 0,float k = 0,float max_target_e = 0,float max_cout = 0,float cout_k = 1)
	:feedForward(FeedForward_t(dt,k,max_target_e,max_cout,cout_k))
	{};
	float UpData(float target, float k);
	float UpData2(float target);
};

float RM_FeedForward::UpData(float target, float k)
{
	//a是调节这一次与上一次的系数
	//k是放大比例
	this->feedForward.cout = k * (target - this->feedForward.last_target);
	this->feedForward.last_target = target;
	
	return this->feedForward.cout;
}

float RM_FeedForward::UpData2(float target)
{
	if(target != this->feedForward.last_target)
	{
		this->feedForward.target_e = (target - this->feedForward.last_target) * this->feedForward.k;
		//误差限幅
		if(this->feedForward.target_e >= this->feedForward.max_target_e)this->feedForward.target_e = this->feedForward.max_target_e;
		else if(this->feedForward.target_e <= -this->feedForward.max_target_e)this->feedForward.target_e = -this->feedForward.max_target_e;
		this->feedForward.t = 0;
	}
	this->feedForward.last_target = target;
	this->feedForward.t += this->feedForward.dt;//时间步长
	this->feedForward.cout = this->feedForward.target_e * powf(2.7182818,-this->feedForward.t);
	//输出限幅
	if(this->feedForward.cout >= this->feedForward.max_cout)this->feedForward.cout = this->feedForward.max_cout;
	else if(this->feedForward.cout <= -this->feedForward.max_cout)this->feedForward.cout = -this->feedForward.max_cout;
	if(this->feedForward.t == 0)this->feedForward.cout = 0;
	this->feedForward.cout *= this->feedForward.cout_k;//最后的输出增益
	return this->feedForward.cout;
}

//UDE系统
typedef struct
{
	float Xnt;
	float I_u;
	float ft;
	float f;
	float k;
	float B;
	float max;
	float separate_break;
	double cout;
}UDE_t;

class UDE
{
public:
	UDE_t ude;

	double UDE::UpData(double Xnt, double u, double k, double B, double max, double err);
};

double UDE::UpData(double Xnt, double u, double k, double B, double max, double err)
{
	this->ude.Xnt = Xnt;
	this->ude.I_u += ((u/1.52587891)*0.00018311*0.741) * B;
	
	//积分限幅
	if(this->ude.I_u > this->ude.max) this->ude.I_u = this->ude.max;
	if(this->ude.I_u < -this->ude.max) this->ude.I_u = -this->ude.max;
	
	if(err > this->ude.separate_break)
	{
		this->ude.I_u = this->ude.I_u*0.7 ;
	}

	this->ude.k = k;
	this->ude.B = B;
	this->ude.max = max;
	this->ude.ft = k*(this->ude.Xnt - this->ude.I_u);
	
	this->ude.cout = this->ude.ft / B;
	
	return this->ude.cout;
}
