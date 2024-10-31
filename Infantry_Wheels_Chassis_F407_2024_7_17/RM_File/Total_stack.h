#pragma once

#include "def_variable.h"

struct SEND_GRAPHIC_QUEUE;//发送数据队列
extern SEND_GRAPHIC_QUEUE send_graphic_queue;
uint8_t send_str2[64];

/***************************函数声明*********************************/

//主跑初始化
void Total_tasks_Init();

//主跑函数
void Total_tasks_Run();

//发送3508数据
void Send_3508_CAN();

//急停模式控制所有pid
void control_pid_0_speed();

//清空数据数据
void Clear_ALL_Data();

//获取云台到底盘数据初始化
void Get_Gimbal_to_Chassis_Init();

//获取云台到底盘数据
void Get_Gimbal_to_Chassis(UART_HandleTypeDef* huart);

//发送3508数据
void Send_3508_CAN()
{
	//发送
	RM_FDorCAN_Send(&hcan1,SEND_MOTOR_ID_3508,msd_3508_2006.Data,CAN_TX_MAILBOX1);
}

//发送6020数据
void Send_6020_CAN()
{
	//发送
	RM_FDorCAN_Send(&hcan1,SEND_MOTOR_ID_6020,msd_6020.Data,CAN_TX_MAILBOX1);
}

//发送6020数据
void Send_DM_CAN()
{
	//发送
	RM_FDorCAN_Send(&hcan2,DM_YAW_CAN_ID,dm_yaw.send_data,CAN_TX_MAILBOX0);//发送
}

//急停模式控制所有pid
void control_pid_0_speed()
{

}

//清空数据数据
void Clear_ALL_Data()
{
	
}

//主跑初始化
void Total_tasks_Init()
{
	RM_FDorCAN_Init();//can配置初始化
	
	rmClicker.Init();//遥控器串口配置初始化
	
	Get_Gimbal_to_Chassis_Init();//串口初始化
	
	RM_RefereeSystem::RM_RefereeSystemInit();//串口初始化
		
	darw_graphic_static_ui_init();
	
	pm01.PM01Init();
	
	dm_yaw.off();//大喵电机初始化	
	Send_DM_CAN();
	
//	ude_chassis[0].ude.separate_break = 400;
//	ude_chassis[1].ude.separate_break = 400;
//	ude_chassis[2].ude.separate_break = 400;
//	ude_chassis[3].ude.separate_break = 400;
	
	HAL_Delay(10);
	
	//记录上一次时间
	uint64_t time_adrc = HAL_GetTick();
	
	
	//adrc收敛期
	while(1)
	{

		if(HAL_GetTick() - time_adrc > 500)
		{
			break;
		}
	}
}
RM_PID lsd;

float now_vxy[4],num_vx,num_vy,now_angle,e_vx,e_vy,k_e_vxy = 0.001,expectations_vx,expectations_vy,vxy_zero_s;


//can_filo0中断接收
CAN_RxHeaderTypeDef RxHeader;	//can接收数据
uint8_t RxHeaderData[8] = { 0 };
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//接受信息 
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxHeader,RxHeaderData);
	if(hcan == &hcan1)
	{
		Motor3508.Parse(RxHeader,RxHeaderData);
		Motor6020.Parse(RxHeader,RxHeaderData);
	}
	if(hcan == &hcan2)
	{
		pm01.PM01Parse(RxHeader,RxHeaderData);
	}
}

//UART空闲中断接收
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	rmClicker.Parse(huart,Size);//遥控器解析
}

//UART中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	Get_Gimbal_to_Chassis(huart);
//	RM_RefereeSystem::RM_RefereeSystemParse(huart);
		rmClicker.Parse(huart, 18);//遥控器解析

}

//获取云台到底盘数据初始化
void Get_Gimbal_to_Chassis_Init()
{
//	HAL_UART_Receive_IT(&Send_Gimbal_to_Chassis_Huart,&Gimbal_to_Chassis_Data.head,1);//先使用中断接收到帧头
}

uint8_t wshu = 0;
//获取云台到底盘数据
void Get_Gimbal_to_Chassis(UART_HandleTypeDef* huart)
{
	if(huart == &Send_Gimbal_to_Chassis_Huart)
	{
		
		//获取帧头
		if(Gimbal_to_Chassis_Data.head == 0xAA)
		{
			HAL_UART_Receive_DMA(&Send_Gimbal_to_Chassis_Huart,Gimbal_to_Chassis_Data.pData,Send_Gimbal_to_Chassis_Huart_LEN);//开启dma接收
			
			//修正帧尾
			if(Gimbal_to_Chassis_Data.pData[Send_Gimbal_to_Chassis_Huart_LEN - 1] != 0xAA)
			{
				Gimbal_to_Chassis_Data.head = 0x00;
				Get_Gimbal_to_Chassis_Init();
			}
			else
			{
				Gimbal_to_Chassis_Data.notation_RC_LY = Gimbal_to_Chassis_Data.pData[0] >> 7;//符号位
				Gimbal_to_Chassis_Data._RC_LY = Gimbal_to_Chassis_Data.pData[0] & 0x7f;//数据
				Gimbal_to_Chassis_Data.int16_RC_LY  = Gimbal_to_Chassis_Data._RC_LY * 6;//_RC_LY解算
				if(Gimbal_to_Chassis_Data.notation_RC_LY == 0)Gimbal_to_Chassis_Data.int16_RC_LY *= -1;//符号位解算
				
				
				Gimbal_to_Chassis_Data.notation_RC_LX = Gimbal_to_Chassis_Data.pData[1] >> 7;//符号位
				Gimbal_to_Chassis_Data._RC_LX = Gimbal_to_Chassis_Data.pData[1] & 0x7f;//数据
				Gimbal_to_Chassis_Data.int16_RC_LX  = Gimbal_to_Chassis_Data._RC_LX * 6;//_RC_LX解算
				if(Gimbal_to_Chassis_Data.notation_RC_LX == 0)Gimbal_to_Chassis_Data.int16_RC_LX *= -1;//符号位解算
				
				Gimbal_to_Chassis_Data.yaw_encoder_angle_e = *((int16_t*)&Gimbal_to_Chassis_Data.pData[2]);
				Gimbal_to_Chassis_Data.yaw_encoder_e = *((int16_t*)&Gimbal_to_Chassis_Data.pData[9]);
				
				Gimbal_to_Chassis_Data.stop = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x80);//停止
				Gimbal_to_Chassis_Data.chassis_follow_gimbal = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x40);//底盘跟随云台
				Gimbal_to_Chassis_Data.gimbal_host_chassis = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x20);//主云台
				Gimbal_to_Chassis_Data.chassis_gyro = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x10) | (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x08);//小陀螺	
				Gimbal_to_Chassis_Data.chassis_top = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x10);//遥控器小陀螺
				Gimbal_to_Chassis_Data.up_ui = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x04);//刷新ui						
				Gimbal_to_Chassis_Data.MCL_of = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x02);//摩擦轮
				Gimbal_to_Chassis_Data.CM_of = (bool)(Gimbal_to_Chassis_Data.pData[4] & 0x01);//仓门

				Gimbal_to_Chassis_Data.notation_ER = Gimbal_to_Chassis_Data.pData[5] >> 2;//符号位
				Gimbal_to_Chassis_Data.int8_ER = Gimbal_to_Chassis_Data._ER = (Gimbal_to_Chassis_Data.pData[5] & 0x02) | (Gimbal_to_Chassis_Data.pData[5] & 0x01);//数据
				if(Gimbal_to_Chassis_Data.notation_ER == 1)Gimbal_to_Chassis_Data.int8_ER *= -1;//符号位解算
				Gimbal_to_Chassis_Data.bp_of = (bool)(Gimbal_to_Chassis_Data.pData[8] & 0x01);
				
				Gimbal_to_Chassis_Data.notation_pitch_cai = Gimbal_to_Chassis_Data.pData[6] >> 4;//符号位
				Gimbal_to_Chassis_Data._pitch_cai = Gimbal_to_Chassis_Data.pData[6] & 0x0f;//数据
				Gimbal_to_Chassis_Data.int8_pitch_cai  = Gimbal_to_Chassis_Data._pitch_cai * 1.56;//pitch_cai解算
				if(Gimbal_to_Chassis_Data.notation_pitch_cai == 1)Gimbal_to_Chassis_Data.int8_pitch_cai *= -1;//符号位解算
				
				Gimbal_to_Chassis_Data.notation_ZX = Gimbal_to_Chassis_Data.pData[7] >> 2;//符号位
				Gimbal_to_Chassis_Data.int8_ZX = Gimbal_to_Chassis_Data._ER = (Gimbal_to_Chassis_Data.pData[7] & 0x02) | (Gimbal_to_Chassis_Data.pData[7] & 0x01);//数据
				if(Gimbal_to_Chassis_Data.notation_ZX == 1)Gimbal_to_Chassis_Data.int8_ZX *= -1;//符号位解算
				
				
				Ctrl_Key.UpKey(Gimbal_to_Chassis_Data.up_ui);//刷新ui
				if(Ctrl_Key.RisingEdge)
				{
					set_send_graphic_queue_is_Delete_all();//删除全部图层
					darw_graphic_static_ui_init();//静态ui初始化
				}
				
//				last_er_v = now_er_v;//转速获取
//				now_er_v = Gimbal_to_Chassis_Data.int8_ER;
//				if(now_er_v != last_er_v)
//				{
					//set_gy_v(RM_RefereeSystem::OperateRevise,W_V_Init + now_er_v * GY_V_SET);//修改转速
//				}
				
				last_zx_v = now_zx_v;//速度获取
				now_zx_v = Gimbal_to_Chassis_Data.int8_ZX;
				if(now_zx_v != last_zx_v)
				{
					set_yd_v(RM_RefereeSystem::OperateRevise,shift_vxy_zoom);//修改速度
					
					set_gy_v(RM_RefereeSystem::OperateRevise,(YD_V_Init+(now_zx_v * YD_V_SET)-0.2));//修改转速
				}
				
				
				MCL_Key.UpKey(Gimbal_to_Chassis_Data.MCL_of);//刷新摩擦轮开关
				if(MCL_Key.RisingEdge | MCL_Key.FallingEdge)
				{
					set_mcl_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.MCL_of);//摩擦轮开关
				}
				
				CM_Key.UpKey(Gimbal_to_Chassis_Data.CM_of);//刷新仓门开关
				if(CM_Key.RisingEdge | CM_Key.FallingEdge)
				{
					set_cm_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.CM_of);//仓门开关
				}
				BP_Key.UpKey(Gimbal_to_Chassis_Data.bp_of);
				if(BP_Key.RisingEdge | BP_Key.FallingEdge)
				{
					set_bp_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.bp_of);//拨盘开关
				}
				//set_cd_of(RM_RefereeSystem::OperateRevise,pm01.cin_voltage);
				//ZM_Key.UpKey(Gimbal_to_Chassis_Data.v_of);
				//wwww = ZM_Key.NowKey;
//				if(ZM_Key.RisingEdge | ZM_Key.FallingEdge)
//				{
//					set_zm_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.v_of);//仓门开关
//				}
//				if(ZM_Key.RisingEdge | ZM_Key.FallingEdge)
//				{
//					set_100w_of(RM_RefereeSystem::OperateRevise,Gimbal_to_Chassis_Data.v_of);//100w开关
//				}
			}				
		}
		else
		{
			HAL_UART_Receive_IT(&Send_Gimbal_to_Chassis_Huart,&Gimbal_to_Chassis_Data.head,1);//先使用中断接收到帧头
		}
		Gimbal_to_Chassis_Data.dir_time.UpLastTime();//串口更新时间
	}
}
float speed;
float avg_cur;
void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef *htim)//回调函数
{
	if(htim == &htim6)
	{							
		if(dir == false)
		{
			vx = vy = vw = 0;
			yaw_e_xita = -YAW_E * 0.043950;//θ误差计算
			yaw_e_radian = (yaw_e_xita + 90.0) * 0.017453;//转弧度,+90是因为修正原本的角度
			cos_xita = cosf(yaw_e_radian);//计算cos
			sin_xita = sinf(yaw_e_radian);//计算sin
			if(CHASSIS_FOLLOW_GIMBAL == true/*底盘跟随云台*/ || 
				 GIMBAL_HOST_CHASSIS == true/*主云台模式*/ || 
				 CHASSIS_GYRO == true/*小陀螺*/ || CHASSIS_TOP == true/*遥控器小陀螺*/)
			{
				if(CHASSIS_FOLLOW_GIMBAL == true/*底盘跟随云台*/)
				{					
					vw = Gimbal_to_Chassis_Data.yaw_encoder_angle_e * kvw*-1;	//vw计算
					td_vw.td_quadratic(vw,false);//vw跟踪微方器
				}
				else if(GIMBAL_HOST_CHASSIS == true/*主云台模式*/)
				{
//					vw = 0;//旋转量
					vw = RC_RX*10;
					td_vw.td_quadratic(vw);//vw跟踪微方器
				}
				if(CHASSIS_GYRO == true/*小陀螺*/)
				{
					vw = W_V_Init*(YD_V_Init+(now_zx_v * YD_V_SET)-0.2);//旋转量
					td_vw.td_quadratic(vw);//vw跟踪微方器
				}
				else if(CHASSIS_TOP == true/*遥控器小陀螺*/)
				{
					vw = Gimbal_to_Chassis_Data.int16_RC_LX*1.5;
					td_vw.td_quadratic(vw);//vw跟踪微方器
				}
				if(CHASSIS_FOLLOW_GIMBAL == true/*底盘跟随云台，遥控器或者键鼠shift*/)
				{
					shift_vxy_zoom = YD_V_Init+(now_zx_v * YD_V_SET)+0.4;
					;//修改速度正比
//					td_vx.r = td_vy.r = 10;//修改速度跟踪
				}
				else
				{
					
					shift_vxy_zoom = YD_V_Init+(now_zx_v * YD_V_SET);//修改速度正比
//					td_vx.r = td_vy.r = 10;//修改速度跟踪
				}
				
				//旋转矩阵
				vx = (td_vx.x1 * cos_xita - td_vy.x1 * sin_xita) * shift_vxy_zoom;
				vy = (td_vx.x1 * sin_xita + td_vy.x1 * cos_xita) * shift_vxy_zoom;
			}
			if(abs(chassis_RC_LX) > 15 || abs(chassis_RC_LY) > 15)
			{
				if(CHASSIS_GYRO==true)
				{
					vxy_kvw = 1.3;//保底盘功率
				}
				else
				{
					vxy_kvw = 3.0;
				}
			}
			else
			{
				if(CHASSIS_GYRO==true)
				{
					vxy_kvw = 1.3;//保底盘功率
				}
				else
				{
					vxy_kvw = 1.5;
				}
			}
			
			td_vx.td_quadratic(RC_LX , true);//chassis_RC_LX跟踪微方器
			td_vy.td_quadratic(RC_LY , true);//chassis_RC_LY跟踪微方器
			td_vw.td_quadratic(RC_RX , true);
			
//				wheel.UpData(vx * 24.8242 * CHASSIS_SPEED_ZOOM_VXY,vy * 24.8242 * CHASSIS_SPEED_ZOOM_VXY,td_vw.x1 * CHASSIS_SPEED_ZOOM_VW * vxy_kvw,MAX_CHASSIS_SPEED);//底盘解算
//			
//				AGV_wheel.UpData(vx * 24.8242 * CHASSIS_SPEED_ZOOM_VXY,vy * 24.8242 * CHASSIS_SPEED_ZOOM_VXY,td_vw.x1 * CHASSIS_SPEED_ZOOM_VW * vxy_kvw,MAX_CHASSIS_SPEED);//底盘解算
//				vw = RC_RX*10;
//				td_vw.td_quadratic(vw);//vw跟踪微方器
			
				AGV_wheel.UpData(td_vx.x1* CHASSIS_SPEED_ZOOM_VXY, td_vy.x1 * CHASSIS_SPEED_ZOOM_VXY,td_vw.x1 * CHASSIS_SPEED_ZOOM_VW * vxy_kvw,MAX_CHASSIS_SPEED);//底盘解算
//				AGV_wheel.UpData(RC_LX * 24.8242 * CHASSIS_SPEED_ZOOM_VXY,RC_LY * 24.8242 * CHASSIS_SPEED_ZOOM_VXY,td_vw.x1 * CHASSIS_SPEED_ZOOM_VW * vxy_kvw,MAX_CHASSIS_SPEED);//底盘解算
				//速度期望值赋值
				tar_AGV_speed[0] = (float)(AGV_wheel.speed[0]);
				tar_AGV_speed[1] = (float)(AGV_wheel.speed[1]);
				tar_AGV_speed[2] = (float)(AGV_wheel.speed[2]);
				tar_AGV_speed[3] = (float)(AGV_wheel.speed[3]);
			
				//角度期望值赋值
				tar_AGV_angle[0] = Motor6020.MinPosHelm(AGV_wheel.angle[0]+Chassis_angle_Init_0x205, Motor6020.GetMotorDataPos(0x205), &tar_AGV_speed[0], 16384, 8192);
				tar_AGV_angle[1] = Motor6020.MinPosHelm(AGV_wheel.angle[1]+Chassis_angle_Init_0x206, Motor6020.GetMotorDataPos(0x206), &tar_AGV_speed[1], 16384, 8192);
				tar_AGV_angle[2] = Motor6020.MinPosHelm(AGV_wheel.angle[2]+Chassis_angle_Init_0x207, Motor6020.GetMotorDataPos(0x207), &tar_AGV_speed[2], 16384, 8192);
				tar_AGV_angle[3] = Motor6020.MinPosHelm(AGV_wheel.angle[3]+Chassis_angle_Init_0x208, Motor6020.GetMotorDataPos(0x208), &tar_AGV_speed[3], 16384, 8192);
		}
		if(dir == true)
		{
			tar_AGV_speed[0] = tar_AGV_speed[1] = tar_AGV_speed[2] = tar_AGV_speed[3] = 0;
			
			tar_AGV_angle[0] = Chassis_angle_Init_0x205;
			tar_AGV_angle[1] = Chassis_angle_Init_0x206;
			tar_AGV_angle[2] = Chassis_angle_Init_0x207;
			tar_AGV_angle[3] = Chassis_angle_Init_0x208;
		}
		
		if(switch_sin == true)
		{
			sintt += 0.001;

			sin_out = 3000*sinf(2*3.1415926*sintt*HZ)+4000;
		}
		else
		{
				sin_out = tar_AGV_angle[0];
		}
		
//		Speed_0x205.td_quadratic(tar_AGV_angle[0]);
		der_sin = (sin_out - last_sin)*sin_k;
//	last_sin = Get_Derivative(tar_AGV_angle[0], sin_k);

		if(der_sin >= 380)
			der_sin = 380;
		if(der_sin <= -380)
			der_sin = -380;
		
		//过零处理
//		avg_cur = Motor6020.GetMotorDataPos(0x205);
//		Handle_Angle8191_PID_Over_Zero(&tar_AGV_angle[0], &avg_cur);
		Zero_crossing[0] = Motor6020.Zero_crossing_processing(sin_out, Motor6020.GetMotorDataPos(0x205), 8192);
		Zero_crossing[1] = Motor6020.Zero_crossing_processing(tar_AGV_angle[1], Motor6020.GetMotorDataPos(0x206), 8192);
		Zero_crossing[2] = Motor6020.Zero_crossing_processing(tar_AGV_angle[2], Motor6020.GetMotorDataPos(0x207), 8192);
		Zero_crossing[3] = Motor6020.Zero_crossing_processing(tar_AGV_angle[3], Motor6020.GetMotorDataPos(0x208), 8192);

		//PID运算		
		AGV_pid[0].GetPidPos(AGV_pid_Init, Zero_crossing[0], Motor6020.GetMotorDataPos(0x205), 30000);
		AGV_pid[1].GetPidPos(AGV_pid_Init, Zero_crossing[1], Motor6020.GetMotorDataPos(0x206), 30000);
		AGV_pid[2].GetPidPos(AGV_pid_Init, Zero_crossing[2], Motor6020.GetMotorDataPos(0x207), 30000);
		AGV_pid[3].GetPidPos(AGV_pid_Init, Zero_crossing[3], Motor6020.GetMotorDataPos(0x208), 30000);
		
		AGV_angle_speed_pid[0].GetPidPos(AGV_angle_speed_Init, AGV_pid[0].pid.cout + Get_Derivative(tar_AGV_angle[0], 10, 0), Motor6020.GetMotorDataSpeed(0x205), 30000);
		AGV_angle_speed_pid[1].GetPidPos(AGV_angle_speed_Init, AGV_pid[1].pid.cout + Get_Derivative(tar_AGV_angle[1], 10, 1), Motor6020.GetMotorDataSpeed(0x206), 30000);
		AGV_angle_speed_pid[2].GetPidPos(AGV_angle_speed_Init, AGV_pid[2].pid.cout + Get_Derivative(tar_AGV_angle[2], 10, 2), Motor6020.GetMotorDataSpeed(0x207), 30000);
		AGV_angle_speed_pid[3].GetPidPos(AGV_angle_speed_Init, AGV_pid[3].pid.cout + Get_Derivative(tar_AGV_angle[3], 10, 3) , Motor6020.GetMotorDataSpeed(0x208), 30000);

		//UDE设置
//			ude_chassis_0x205.UpData(((float)(Angle_speed.x1)) , ((AGV_pid[0].pid.cout*0.546133)*0.741)*0.00001, AGV_pid[0].pid.now_e);
//		ude_chassis[1].UpData(((float)(Motor6020.GetMotorDataSpeed(0x206)) / 60)*2 , AGV_pid[1].pid.cout, 18, 0.01, 3000, AGV_pid[1].pid.now_e);
//		ude_chassis[2].UpData(((float)(Motor6020.GetMotorDataSpeed(0x207)) / 60)*2 , AGV_pid[2].pid.cout, 18, 0.01, 3000, AGV_pid[2].pid.now_e);
//		ude_chassis[3].UpData(((float)(Motor6020.GetMotorDataSpeed(0x208)) / 60)*2 , AGV_pid[3].pid.cout, 18, 0.01, 3000, AGV_pid[3].pid.now_e);

		//PID更新		
		AGV_speed[0].GetPidPos(AGV_speed_Init, (-tar_AGV_speed[0]), Motor3508.GetMotorDataSpeed(0x201), 16384);
		AGV_speed[1].GetPidPos(AGV_speed_Init, (-tar_AGV_speed[1]), Motor3508.GetMotorDataSpeed(0x202), 16384);
		AGV_speed[2].GetPidPos(AGV_speed_Init, (tar_AGV_speed[2]), Motor3508.GetMotorDataSpeed(0x203), 16384);
		AGV_speed[3].GetPidPos(AGV_speed_Init, (tar_AGV_speed[3]), Motor3508.GetMotorDataSpeed(0x204), 16384);
				
		setMSD(&msd_3508_2006, AGV_speed[0].pid.cout, 1);
		setMSD(&msd_3508_2006, AGV_speed[1].pid.cout, 2);
		setMSD(&msd_3508_2006, AGV_speed[2].pid.cout, 3);
		setMSD(&msd_3508_2006, AGV_speed[3].pid.cout, 4);

		setMSD(&msd_6020, AGV_angle_speed_pid[0].pid.cout, Get_MOTOR_SET_ID_6020(0x205));
		setMSD(&msd_6020, AGV_angle_speed_pid[1].pid.cout, Get_MOTOR_SET_ID_6020(0x206));
		setMSD(&msd_6020, AGV_angle_speed_pid[2].pid.cout, Get_MOTOR_SET_ID_6020(0x207));
		setMSD(&msd_6020, AGV_angle_speed_pid[3].pid.cout, Get_MOTOR_SET_ID_6020(0x208));

	}
	if(htim == &htim7)
	{


	}
	
	if(htim == &htim5)
	{
		//发送数据
		if(Send_ms == 0)
		{
			Send_6020_CAN();
		}
		else if(Send_ms == 2)																																																																																																																																																																																																																																																																																																																																																																																																			
		{
			Send_3508_CAN();
		}
		Send_ms++;
		Send_ms %= 3;
		
		*((float*)&send_str2[0]) = sin_out;
		*((float*)&send_str2[4]) = Motor6020.GetMotorDataPos(0x205);
		*((float*)&send_str2[8]) = tar_AGV_angle[0];
//		*((float*)&send_str2[12]) = ude_chassis_0x205.ude.Xnt;

		*((float*)&send_str2[12]) = 0;
//		*((float*)&send_str2[20]) = Motor6020.GetMotorDataSpeed(0x205);

		*((uint32_t*)&send_str2[sizeof(float) * (7)]) = 0x7f800000;
		HAL_UART_Transmit_DMA(&Send_Usart_Data_Huart, send_str2, sizeof(float) * (7 + 1));

	}
}

RM_StaticTime darw_graphic_ui_time;//绘制ui定时器10hz
struct SEND_GRAPHIC_QUEUE//发送数据队列
{
	RM_RefereeSystem::graphic_data_struct_t graphic_data_struct[50];//图层数据
	//图案
	int8_t size;
	int8_t send_graphic_data_struct_size;
	//优先把所有文字显示出来
	RM_RefereeSystem::ext_client_custom_character_t ext_client_custom_character[20];//文字数据
	//文字
	int8_t wz_size;
	//刷新图层
	bool is_Delete_all;
	void add(RM_RefereeSystem::graphic_data_struct_t graphic_data_struct_temp)
	{		
		if(size >= 49)return;
		memcpy((void*)&graphic_data_struct[size],(void*)&graphic_data_struct_temp,sizeof(RM_RefereeSystem::graphic_data_struct_t));
		size++;
	}
	void add_wz(RM_RefereeSystem::ext_client_custom_character_t ext_client_custom_character_temp)
	{		
		if(wz_size >= 19)return;
		memcpy((void*)&ext_client_custom_character[wz_size],(void*)&ext_client_custom_character_temp,sizeof(RM_RefereeSystem::ext_client_custom_character_t));
		wz_size++;
	}
	bool send()
	{
		if(is_Delete_all == true)return false;		
		if(wz_size != 0)return false;
		if(size == 0)return true;
		if(!darw_graphic_ui_time.ISOne(100))return false;		
		if(size >= 7)send_graphic_data_struct_size = 7;
		else if(size > 2)send_graphic_data_struct_size = 5;
		else if(size == 2)send_graphic_data_struct_size = 2;
		else if(size == 1)send_graphic_data_struct_size = 1;
		else send_graphic_data_struct_size = 0;
		if(send_graphic_data_struct_size != 0)
		{
			RM_RefereeSystem::RM_RefereeSystemSendDataN(graphic_data_struct,send_graphic_data_struct_size);
			size -= send_graphic_data_struct_size;	
		}
		if(size < 0)size = 0;
		memcpy(graphic_data_struct,(void*)&graphic_data_struct[send_graphic_data_struct_size],sizeof(RM_RefereeSystem::graphic_data_struct_t) * (size));	
		memset((void*)&graphic_data_struct[size],0,sizeof(RM_RefereeSystem::graphic_data_struct_t) * (send_graphic_data_struct_size));	
		return true;
	}
	bool send_wz()
	{
		if(is_Delete_all == true)return false;		
		if(wz_size == 0)return true;
		if(!darw_graphic_ui_time.ISOne(100))return false;				
		if(wz_size != 0)
		{
			RM_RefereeSystem::RM_RefereeSystemSendStr(ext_client_custom_character[wz_size - 1]);
			wz_size--;	
		}
		if(wz_size < 0)wz_size = 0;
		return true;
	}
	bool send_delet_all()
	{
		if(is_Delete_all == false)return true;
		if(!darw_graphic_ui_time.ISOne(100))return false;	
		if(is_Delete_all == true)
		{
			RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::DeleteAll);
			RM_RefereeSystem::RM_RefereeSystemDelete(RM_RefereeSystem::DeleteAll,0);
			RM_RefereeSystem::RM_RefereeSystemClsToop();
			is_Delete_all = false;
		}
		return true;
	}
}send_graphic_queue;

//超电位置
#define CD_X 1260
#define CD_Y 240
#define CD_W 200
#define CD_H 20
#define CD_WZ_X 1000 //文字
#define CD_WZ_Y 250
//超电线
#define CDL_X 1260
#define CDL_Y 240
#define CDL_H 50
//底盘位置
#define CHASSIS_X 760
#define CHASSIS_Y 180
#define CHASSIS_W 50
#define CHASSIS_H 50
//底盘碰撞位置
#define collide_1 550
#define collide_2 1370
#define collide_magnify 3 //放大倍率
//下面的瞄准线
#define aim_x 700
#define aim_y 465


//GY_V小陀螺转速
#define GY_V_X 20
#define GY_V_Y 840
//yd_V小陀螺速速
#define YD_V_X 20
#define YD_V_Y 790
//摩擦轮on_off
#define MCL_of_X 20
#define MCL_of_Y 740
//仓门on_off
#define CM_of_X 20
#define CM_of_Y 690
//拨盘on_off
#define BP_of_X 20
#define BP_of_Y 640
//超电on_off
#define CD_of_X 20
#define CD_of_Y 340
//视觉自瞄
#define ZM_of_X 20
#define ZM_of_Y 560
//设置删除全部图层
void set_send_graphic_queue_is_Delete_all()
{
	send_graphic_queue.is_Delete_all = true;
}

//修改转速字符串
void set_gy_v(int Operate,float v)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	char str[10] = { 0 };
	sprintf(str,"XTL:%d",int(W_V_Init*v));
	send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("gyv",1,str,GY_V_X,GY_V_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
//修改移速字符串
void set_yd_v(int Operate,float v)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	char str[10] = { 0 };
	sprintf(str,"YD:%d",int(4000*v));
	send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("rpm",1,str,YD_V_X,YD_V_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
//修改摩擦开关
void set_mcl_of(int Operate,bool is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mcl",1,"MCL:ON",MCL_of_X,MCL_of_Y));
	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("mcl",1,"MCL:OFF",MCL_of_X,MCL_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}

//修改仓门开关
void set_cm_of(int Operate,bool is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("cm",1,"CM:ON",CM_of_X,CM_of_Y));
	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("cm",1,"CM:OFF",CM_of_X,CM_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
//修改拨盘开关
void set_bp_of(int Operate,bool is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("bp",1,"BP:ON",BP_of_X,BP_of_Y));
	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("bp",1,"BP:OFF",BP_of_X,BP_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
//修改超电开关
void set_cd_of(int Operate,float cd)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(30);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(3);
	char str[10] = { 0 };
	sprintf(str,"CD:%f",cd);
	send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("cd_num",1,str,CD_of_X,CD_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}



//修改自瞄开关
void set_zm_of(int Operate,bool is)
{
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(40);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("zm",1,"ZM:ON",ZM_of_X,ZM_of_Y));
	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("zm",1,"ZM:OFF",ZM_of_X,ZM_of_Y));
	RM_RefereeSystem::RM_RefereeSystemClsToop();
}
////修改100w开关
//void set_100w_of(int Operate,bool is)
//{
//	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(Operate);//设置修改
//	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
//	RM_RefereeSystem::RM_RefereeSystemSetStringSize(40);
//	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
//	if(is == true)send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("100w",1,"100W:ON",ZM_of_X,ZM_of_Y));
//	else send_graphic_queue.add_wz(RM_RefereeSystem::RM_RefereeSystemSetStr("100w",1,"100W:OFF",ZM_of_X,ZM_of_Y));
//	RM_RefereeSystem::RM_RefereeSystemClsToop();
//}
void darw_graphic_static_ui_init()//静态ui初始化
{
	is_up_ui = false;//上锁
	send_graphic_queue.size = 0;//复位
//	//规则
//	//0号图层给静态图层使用，用于绘制静态不改图形
	RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateAdd);
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(1);
	//瞄准线
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("W",0,568,Win_H * 0.5,      1350,Win_H * 0.5));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("H",0,Win_W * 0.5,          0,Win_W * 0.5,Win_H));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("high", 0,aim_x,aim_y,aim_x+520,aim_y));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("mid", 0,aim_x+100,aim_y-75,aim_x+420,aim_y-75));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("low", 0,aim_x+160,aim_y-150,aim_x+360,aim_y-150));

	/***************************动态UI初始化***************************/
	//pitch初始化
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorCyan);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(25);		
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("pitch_Init",1,50,130,960,540,380,380));
	
	//超电初始化
	RM_RefereeSystem::RM_RefereeSystemSetWidth(15);		
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("cd_Init",3,271,310,960,540,380,380));
	
	//小陀螺初始化
	RM_RefereeSystem::RM_RefereeSystemSetWidth(25);		
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("gyro_Init",2,0,360,1600,750,80,80));
	
	//转速条初始化
	RM_RefereeSystem::RM_RefereeSystemSetWidth(35);		
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp1",0,1600,690,1600, 750+61));
	
	/***************************绘制静态UI***************************/
	//pitch刻度
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
		RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_130",0,50,51,960,540,380,380));
		RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_120",1,60,61,960,540,380,380));
		RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_110",2,70,71,960,540,380,380));
		RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_100",3,80,81,960,540,380,380));
		RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_90",4,90,91,960,540,380,380));
		RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_80",5,100,101,960,540,380,380));
		RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_60",6,110,111,960,540,380,380));
		RM_RefereeSystem::RM_RefereeSystemSetWidth(15);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_50",7,120,121,960,540,380,380));
		RM_RefereeSystem::RM_RefereeSystemSetWidth(25);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("angle_50",8,130,131,960,540,380,380));
	
	//pitch刻度数字
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
	RM_RefereeSystem::RM_RefereeSystemSetStringSize(10);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(2);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("write_130", 0, 130,1210,770));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("write_110", 1, 110,1270,660));
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("write_90", 2, 90,1300,540)); 
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("write_70", 3, 70,1280,420)); 
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetInt("write_70", 4, 50,1210,310)); 
		
	//超电上限位
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("cd1",2, 570, 540, 595, 540));
	//超电下限位
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(5);
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("cd2",2, 673, 800, 690, 785));
	 
	//小陀螺内圆
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(1);		
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetCircle("xtl_in",2,1600,750,67));
	
	RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorWhite);
	RM_RefereeSystem::RM_RefereeSystemSetWidth(1);		
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetRectangle("dp9",2,1600-20,750-61,1600+20, 750+62));
	

	
	//小陀螺转速显示
	set_gy_v(RM_RefereeSystem::OperateAdd,(YD_V_Init+(now_zx_v * YD_V_SET)-0.2));
	//平移速度显示
	set_yd_v(RM_RefereeSystem::OperateAdd,shift_vxy_zoom);
	//摩擦轮开启关闭显示
	set_mcl_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.MCL_of);
	//仓门开启关闭显示
	set_cm_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.CM_of);
	//拨盘开启关闭显示
	set_bp_of(RM_RefereeSystem::OperateAdd,Gimbal_to_Chassis_Data.bp_of);
	
	is_up_ui = true;
}
float angle_ly,ly;//角度，长度
void darw_graphic_ui()//绘制ui
{
	if(send_graphic_queue.send_delet_all() == true && is_up_ui == true && send_graphic_queue.send_wz() == true && send_graphic_queue.send() == true)
	{
		angle_ly = 31.466 + Gimbal_to_Chassis_Data.int8_pitch_cai;
		ly = (asin(angle_ly / 180.0 * 3.1415926) * 43) * collide_magnify;
		
		yaw_e_radian = (Gimbal_to_Chassis_Data.yaw_encoder_angle_e * 0.043950) * 0.017453;//转弧度,+90是因为修正原本的角度
		float cos_xita = cosf(yaw_e_radian);//计算cos
		float sin_xita = sinf(yaw_e_radian);//计算sin
		
		RM_RefereeSystem::RM_RefereeSystemSetOperateTpye(RM_RefereeSystem::OperateRevise);

		//绘制pitch指示
		RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorPink);
		RM_RefereeSystem::RM_RefereeSystemSetWidth(25);		
		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("pitch_Init",1,pitch_out,pitch_out + 2,960,540,380,380));
		
					//绘制小陀螺指示																											 
			RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorPink);
			RM_RefereeSystem::RM_RefereeSystemSetWidth(25);		
			send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("gyro_Init",2,yaw_out+30,yaw_out-30,1600,750,80,80));

		//绘制超电能量调
		if(sin_out < 290)
		{
			RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorRedAndBlue);
		}
		else
		{
			RM_RefereeSystem::RM_RefereeSystemSetColor(RM_RefereeSystem::ColorGreen);
		}
		
		RM_RefereeSystem::RM_RefereeSystemSetWidth(15);		
		send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetArced("cd_Init",3,271,sin_out,960,540,380,380));
																																					
	RM_RefereeSystem::RM_RefereeSystemSetWidth(35);		
	send_graphic_queue.add(RM_RefereeSystem::RM_RefereeSystemSetLine("dp1",0,1600,690,1600, sin_out+470));		
		}
}


void Gpio_led_init(void)
{
    __HAL_RCC_GPIOG_CLK_ENABLE();
	
		GPIO_InitTypeDef GPIO_InitStruct = {0};
	
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
}

