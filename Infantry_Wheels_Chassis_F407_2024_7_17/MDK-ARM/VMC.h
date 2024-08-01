#pragma once

typedef struct
{
    float L0;

    float l1;
    float l2;
    float l3;
    float l4;
    float l5;
    float lBD;

    typedef struct
    {
        float phi0;
    }Angle;

    typedef struct
    {
        float A0, B0, C0;
        float phi2,phi3;
        float phi1,phi4;
    }Process;

    typedef struct
    {
        float j11;
        float j12;
        float j21;
        float j22;
    }Jacobian;


    typedef struct
    {
        float theta;
        float d_theta;//theta的一阶导数
        float last_d_theta;
        float dd_theta;//theta的二阶导数

        float d_L0;//L0的一阶导数
        float dd_L0;//L0的二阶导数
        float last_L0;
        float last_d_L0;
    }Derivative;

	float F0;
	float Tp;
	float FN;//支持力
	float torque_set[2];

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
};

void VMC_leg_t::Up_Left(float pitch_Angle, float pitch_Gyro, float dt)
{
    
}
