#include "RM_stdxxx.h"
#include "RM_Filter.h"
#include "ladrc.h"

//����kp,ki,kd�ṹ��
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

//����kpid
Kpid_t AddKpid(Kpid_t Kpid,double kp = 0,double ki = 0,double kd = 0)
{		
	return Kpid_t(Kpid.kp + kp,Kpid.ki + ki,Kpid.kd + kd);
}

typedef struct
{
    //������ʵ��
    double cin,cout,feedback;
    //p,i,d����
    double p,i,d;
		//Delta,p,i,d����
    double Dp,Di,Dd;
    //���
    double last_e,last_last_e,now_e,IerrorA,IerrorB;
		//��ƵƵ��
		unsigned char Frequency,_Frequency_;
		//td����΢�������������
		TD_quadratic td_e;
		//�޷�
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
	//�������޺ͱ��ٻ���
	void SetMixI(double maxi,double IerrorA,double IerrorB);
	//��Ƶ����
	void SetFrequency(double Frequency);
	//λ��ʽpid��ȡ
	double GetPidPos(Kpid_t kpid,double cin,double feedback,double max);
	//���
	void clearPID();
	//�������
	void PidRstDelta();
	//����ʽpid��ȡ
	double GetPidDelta(Kpid_t kpid,double cin,double feedback,double max);
};
