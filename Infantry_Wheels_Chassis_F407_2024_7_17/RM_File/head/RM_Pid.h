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
