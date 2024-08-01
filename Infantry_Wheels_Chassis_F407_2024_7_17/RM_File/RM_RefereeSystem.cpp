#include "RM_RefereeSystem.h"
#include "RM_StaticTime.h"

#define RM_RefereeSystemHuart huart6
using namespace RM_RefereeSystem;
using namespace RM_RefereeSystemCRC;
//死亡时间
RM_StaticTime dirTime;
//�.�连标记
bool RM_RefereeSystemDirFlag = 0;
//接收数据
uint8_t RM_RefereeSystemp8Data = 0;
//设置颜色与线粗
RM_RefereeSystemToop_t RM_RefereeSystemToop = { 1,1,1,1 };
//数据接收格式变量
RM_RefereeSystemData_t RM_RefereeSystemData = { 0 };

RM_RefereeSystemData_t RM_RefereeSystemData01 = { 0 };
//0x0201 机器人状态数据
ext_game_robot_status_t ext_power_heat_data_0x0201 = { 0 };
//0x0202 实时功率热量数据
ext_power_heat_data_t ext_power_heat_data_0x0202 = { 0 };
//0x0303 机器人间交互数据
map_command_t map_command_0x0303 = { 0 };

//发送数据
uint8_t tx_buf[128] = { 0 } ;

uint8_t daohang_tx_buf[128] = { 0 } ;
namespace RM_RefereeSystem
{
//初始化
void RM_RefereeSystemInit()
{
	HAL_UART_Receive_IT(&RM_RefereeSystemHuart,&RM_RefereeSystemp8Data,sizeof(RM_RefereeSystemp8Data));
}
//设置颜色
void RM_RefereeSystemSetColor(int color)
{
	RM_RefereeSystemToop.color = color;
}
//设置线粗
void RM_RefereeSystemSetWidth(int width) 
{
	RM_RefereeSystemToop.width = width;
}
//设置字体
void RM_RefereeSystemSetStringSize(int size)
{
	RM_RefereeSystemToop.strsize = size;
}
//设置图形操作
void RM_RefereeSystemSetOperateTpye(int operate_tpye)
{
	RM_RefereeSystemToop.operate_tpye = operate_tpye;
}
//工具清空
void RM_RefereeSystemClsToop()
{
	RM_RefereeSystemToop.color = RM_RefereeSystemToop.width = RM_RefereeSystemToop.strsize = RM_RefereeSystemToop.operate_tpye = 1;
}
//机器人获取id
int RM_RefereeSystemGetRobotId()
{
	uint16_t receiver_ID = 0x0000;
	//红色
	if(ext_power_heat_data_0x0201.robot_id < 100)
	{
		switch(ext_power_heat_data_0x0201.robot_id)
		{
			case 1:/*英雄机器人*/							    receiver_ID = 0x0101;       break;
			case 2:/*工程机器人*/                 receiver_ID = 0x0102;       break;
			case 3:/*步兵机器人*/									receiver_ID = 0x0103;       break;
			case 4:/*步兵机器人*/									receiver_ID = 0x0104;       break;
			case 5:/*步兵机器人*/   							receiver_ID = 0x0105;       break;
			case 6:/*空中机器人*/                 receiver_ID = 0x0106;       break;
		}
	}
	//蓝色
	if(ext_power_heat_data_0x0201.robot_id > 100)
	{
		switch(ext_power_heat_data_0x0201.robot_id)
		{
			case 101:/*英雄机器人*/							  receiver_ID = 0x0165;       break;
			case 102:/*工程机器人*/               receiver_ID = 0x0166;       break;
			case 103:/*步兵机器人*/								receiver_ID = 0x0167;       break;
			case 104:/*步兵机器人*/								receiver_ID = 0x0168;       break;
			case 105:/*步兵机器人*/   						receiver_ID = 0x0169;       break;
			case 106:/*空中机器人*/               receiver_ID = 0x016A;       break;
		}
	}
	return receiver_ID;
}
//绘画直线/*名字，图层，x1，y1，x2，y2*/
graphic_data_struct_t RM_RefereeSystemSetLine(char* name,uint32_t layer,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y)
{
	graphic_data_struct_t graphic_data_struct = { 0 };
	memcpy((void*)graphic_data_struct .graphic_name,name,3);
	graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
	graphic_data_struct.graphic_tpye = TypeLine;
	graphic_data_struct.layer = layer;
	graphic_data_struct.color = RM_RefereeSystemToop.color;
	graphic_data_struct.width = RM_RefereeSystemToop.width;
	graphic_data_struct.start_x = start_x;
	graphic_data_struct.start_y = start_y;
	graphic_data_struct.end_x = end_x;
	graphic_data_struct.end_y = end_y;
	return graphic_data_struct;
}
//绘画矩形/*名字，图层，x1，y1，x2，y2*/
graphic_data_struct_t RM_RefereeSystemSetRectangle(char* name,uint32_t layer,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y)
{
	graphic_data_struct_t graphic_data_struct = { 0 };
	memcpy((void*)graphic_data_struct .graphic_name,name,3);
	graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
	graphic_data_struct.graphic_tpye = TypeRectangle;
	graphic_data_struct.layer = layer;
	graphic_data_struct.color = RM_RefereeSystemToop.color;
	graphic_data_struct.width = RM_RefereeSystemToop.width;
	graphic_data_struct.start_x = start_x;
	graphic_data_struct.start_y = start_y;
	graphic_data_struct.end_x = end_x;
	graphic_data_struct.end_y = end_y;
	return graphic_data_struct;
}
//绘画圆/*名字，图层，x1，y1，半径*/
graphic_data_struct_t RM_RefereeSystemSetCircle(char* name,uint32_t layer,uint32_t start_x,uint32_t start_y,uint32_t radius)
{
	graphic_data_struct_t graphic_data_struct = { 0 };
	memcpy((void*)graphic_data_struct .graphic_name,name,3);
	graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
	graphic_data_struct.graphic_tpye = TypeCircle;
	graphic_data_struct.layer = layer;
	graphic_data_struct.color = RM_RefereeSystemToop.color;
	graphic_data_struct.width = RM_RefereeSystemToop.width;
	graphic_data_struct.start_x = start_x;
	graphic_data_struct.start_y = start_y;
	graphic_data_struct.radius = radius;
	return graphic_data_struct;
}
//绘画椭圆/*名字，图层，x1，y1，x2，y2*/
graphic_data_struct_t RM_RefereeSystemSetElliptic(char* name,uint32_t layer,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y)
{
	graphic_data_struct_t graphic_data_struct = { 0 };
	memcpy((void*)graphic_data_struct .graphic_name,name,3);
	graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
	graphic_data_struct.graphic_tpye = TypeElliptic;
	graphic_data_struct.layer = layer;
	graphic_data_struct.color = RM_RefereeSystemToop.color;
	graphic_data_struct.width = RM_RefereeSystemToop.width;
	graphic_data_struct.start_x = start_x;
	graphic_data_struct.start_y = start_y;
	graphic_data_struct.end_x = end_x;
	graphic_data_struct.end_y = end_y;
	return graphic_data_struct;
}
//绘画圆弧/*名字，图层，起始角度，终止角度，x1，y1，x2，y2*/
graphic_data_struct_t RM_RefereeSystemSetArced(char* name,uint32_t layer,uint32_t start_angle,uint32_t end_angle,uint32_t start_x,uint32_t start_y,uint32_t end_x,uint32_t end_y)
{
	graphic_data_struct_t graphic_data_struct = { 0 };
	memcpy((void*)graphic_data_struct .graphic_name,name,3);
	graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
	graphic_data_struct.graphic_tpye = TypeArced;
	graphic_data_struct.layer = layer;
	graphic_data_struct.color = RM_RefereeSystemToop.color;
	graphic_data_struct.width = RM_RefereeSystemToop.width;
	graphic_data_struct.start_angle = start_angle;
	graphic_data_struct.end_angle = end_angle;
	graphic_data_struct.start_x = start_x;
	graphic_data_struct.start_y = start_y;
	graphic_data_struct.end_x = end_x;
	graphic_data_struct.end_y = end_y;
	return graphic_data_struct;
}
//绘画浮点数/*名字，图层，浮点数，浮点数长度，x1，y1*/
graphic_data_struct_t RM_RefereeSystemSetFzloat(char* name,uint32_t layer,float x,uint32_t start_x,uint32_t start_y)
{
	graphic_data_struct_t graphic_data_struct = { 0 };
	memcpy((void*)graphic_data_struct .graphic_name,name,3);
	graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
	graphic_data_struct.graphic_tpye = TypeFloat;
	graphic_data_struct.layer = layer;
	graphic_data_struct.color = RM_RefereeSystemToop.color;
	graphic_data_struct.width = RM_RefereeSystemToop.width;
	graphic_data_struct.start_angle = RM_RefereeSystemToop.strsize;
	graphic_data_struct.end_angle = 3;
	graphic_data_struct.start_x = start_x;
	graphic_data_struct.start_y = start_y;
	graphic_data_struct.radius |= ((int32_t)(x*1000) >> 0 ) & 0x3ff;
	graphic_data_struct.end_x  |= ((int32_t)(x*1000) >> 10) & 0x7ff;
	graphic_data_struct.end_y  |= ((int32_t)(x*1000) >> (10 + 11)) & 0x7ff;
	return graphic_data_struct;
}
//绘画整数/*名字，图层，整数，浮点数长度，x1，y1*/
graphic_data_struct_t RM_RefereeSystemSetInt(char* name,uint32_t layer,int32_t x,uint32_t start_x,uint32_t start_y)
{
	graphic_data_struct_t graphic_data_struct = { 0 };
	memcpy((void*)graphic_data_struct .graphic_name,name,3);
	graphic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
	graphic_data_struct.graphic_tpye = TypeInt;
	graphic_data_struct.layer = layer;
	graphic_data_struct.color = RM_RefereeSystemToop.color;
	graphic_data_struct.width = RM_RefereeSystemToop.width;
	graphic_data_struct.start_angle = RM_RefereeSystemToop.strsize;
	graphic_data_struct.start_x = start_x;
	graphic_data_struct.start_y = start_y;
	graphic_data_struct.radius |= (x >> 0 ) & 0x3ff;
	graphic_data_struct.end_x  |= (x >> 10) & 0x7ff;
	graphic_data_struct.end_y  |= (x >> (10 + 11)) & 0x7ff;
	return graphic_data_struct;
}
//绘画字符串/*名字，图层，字符串，x1，y1*/
ext_client_custom_character_t RM_RefereeSystemSetStr(char* name,uint32_t layer,char* str,uint32_t start_x,uint32_t start_y)
{
	ext_client_custom_character_t ext_client_custom_character = { 0 };
	memcpy((void*)ext_client_custom_character.grapic_data_struct.graphic_name,name,3);
	ext_client_custom_character.grapic_data_struct.operate_tpye = RM_RefereeSystemToop.operate_tpye;
	ext_client_custom_character.grapic_data_struct.graphic_tpye = TypeStr;
	ext_client_custom_character.grapic_data_struct.layer = layer;
	ext_client_custom_character.grapic_data_struct.color = RM_RefereeSystemToop.color;
	ext_client_custom_character.grapic_data_struct.width = RM_RefereeSystemToop.width;
	ext_client_custom_character.grapic_data_struct.start_angle = RM_RefereeSystemToop.strsize;
	ext_client_custom_character.grapic_data_struct.end_angle = strlen(str);
	ext_client_custom_character.grapic_data_struct.start_x = start_x;
	ext_client_custom_character.grapic_data_struct.start_y = start_y;
	memcpy((void*)ext_client_custom_character.data,str,ext_client_custom_character.grapic_data_struct.end_angle);
	return ext_client_custom_character;
}
//数据发送客户端绘制删除N个图层
void RM_RefereeSystemDelete(const char operate,const char number)
{
	RM_RefereeSystemData_t RM_RefereeSystemDataTemp = { 0 };
	//id设置
	ext_student_interactive_header_data_t ext_student_interactive_header_data = { 0 };
	ext_student_interactive_header_data.data_cmd_id = 0x0100;
	ext_student_interactive_header_data.sender_ID = ext_power_heat_data_0x0201.robot_id;
	ext_student_interactive_header_data.receiver_ID = RM_RefereeSystemGetRobotId();
	//数据格式
	RM_RefereeSystemDataTemp.SOF = 0xA5;
	RM_RefereeSystemDataTemp.data_length = 6+2;
	RM_RefereeSystemDataTemp.cmd_id = 0x0301;
	memcpy(&tx_buf,0,sizeof(tx_buf));
	//数据=id数据+删除图像
	int idx = 0;
	memcpy((void*)RM_RefereeSystemDataTemp.data,&ext_student_interactive_header_data,sizeof(ext_student_interactive_header_data));
	idx += sizeof(ext_student_interactive_header_data);
	memcpy((void*)(RM_RefereeSystemDataTemp.data + idx),&operate,sizeof(operate));
	idx += sizeof(operate);
	memcpy((void*)(RM_RefereeSystemDataTemp.data + idx),&number,sizeof(number));
	memcpy(tx_buf,&RM_RefereeSystemDataTemp,sizeof(RM_RefereeSystemDataTemp));
	Append_CRC8_Check_Sum(tx_buf,CRC8LEN);
	Append_CRC16_Check_Sum(tx_buf,CRC16LEN(RM_RefereeSystemDataTemp.data_length));
	HAL_UART_Transmit_IT( &RM_RefereeSystemHuart, tx_buf, sizeof(tx_buf) );
}
//数据发送客户端绘制一个图形
void RM_RefereeSystemSendData1(const graphic_data_struct_t graphic_data_struct)
{
	RM_RefereeSystemData_t RM_RefereeSystemDataTemp = { 0 };
	//id设置
	ext_student_interactive_header_data_t ext_student_interactive_header_data = { 0 };
	ext_student_interactive_header_data.data_cmd_id = 0x0101;
	ext_student_interactive_header_data.sender_ID = ext_power_heat_data_0x0201.robot_id;
	ext_student_interactive_header_data.receiver_ID = RM_RefereeSystemGetRobotId();
	//数据格式
	RM_RefereeSystemDataTemp.SOF = 0xA5;
	RM_RefereeSystemDataTemp.data_length = 6+15;
	RM_RefereeSystemDataTemp.cmd_id = 0x0301;
	memcpy(&tx_buf,0,sizeof(tx_buf));
	//数据=id数据+图像数据
	int idx = 0;
	memcpy((void*)RM_RefereeSystemDataTemp.data,&ext_student_interactive_header_data,sizeof(ext_student_interactive_header_data));
	idx += sizeof(ext_student_interactive_header_data);
	memcpy((void*)(RM_RefereeSystemDataTemp.data + idx),&graphic_data_struct,sizeof(graphic_data_struct));
	memcpy(tx_buf,&RM_RefereeSystemDataTemp,sizeof(RM_RefereeSystemDataTemp));
	Append_CRC8_Check_Sum(tx_buf,CRC8LEN);
	Append_CRC16_Check_Sum(tx_buf,CRC16LEN(RM_RefereeSystemDataTemp.data_length));
	HAL_UART_Transmit_IT( &RM_RefereeSystemHuart, tx_buf, sizeof(tx_buf) );
}
//数据发送客户端绘制1,2,5,7个图形
void RM_RefereeSystemSendDataN(const graphic_data_struct_t graphic_data_struct[],int size)
{
	RM_RefereeSystemData_t RM_RefereeSystemDataTemp = { 0 };
	//id设置
	ext_student_interactive_header_data_t ext_student_interactive_header_data = { 0 };
	switch(size)
	{
		case 1:ext_student_interactive_header_data.data_cmd_id = 0x0101;break;
		case 2:ext_student_interactive_header_data.data_cmd_id = 0x0102;break;
		case 5:ext_student_interactive_header_data.data_cmd_id = 0x0103;break;
		case 7:ext_student_interactive_header_data.data_cmd_id = 0x0104;break;
	}
	ext_student_interactive_header_data.sender_ID = ext_power_heat_data_0x0201.robot_id;
	ext_student_interactive_header_data.receiver_ID = RM_RefereeSystemGetRobotId();
	//数据格式
	RM_RefereeSystemDataTemp.SOF = 0xA5;
	switch(size)
	{
		case 1:RM_RefereeSystemDataTemp.data_length = 6+15;break;
		case 2:RM_RefereeSystemDataTemp.data_length = 6+30;break;
		case 5:RM_RefereeSystemDataTemp.data_length = 6+75;break;
		case 7:RM_RefereeSystemDataTemp.data_length = 6+105;break;
	}
	RM_RefereeSystemDataTemp.cmd_id = 0x0301;
	memcpy(&tx_buf,0,sizeof(tx_buf));
	//数据=id数据+图像数据
	int idx = 0;
	memcpy((void*)RM_RefereeSystemDataTemp.data,&ext_student_interactive_header_data,sizeof(ext_student_interactive_header_data));
	idx += sizeof(ext_student_interactive_header_data);
	for(int i = 0;i < size;i++)
	{
		memcpy((void*)(RM_RefereeSystemDataTemp.data + idx),&graphic_data_struct[i],sizeof(graphic_data_struct_t));
		idx += sizeof(graphic_data_struct_t);
	}
	memcpy(tx_buf,&RM_RefereeSystemDataTemp,sizeof(RM_RefereeSystemDataTemp));
	Append_CRC8_Check_Sum(tx_buf,CRC8LEN);
	Append_CRC16_Check_Sum(tx_buf,CRC16LEN(RM_RefereeSystemDataTemp.data_length));
	HAL_UART_Transmit_DMA( &RM_RefereeSystemHuart, tx_buf, sizeof(tx_buf) );
}
//数据发送客户端绘制字符串
void RM_RefereeSystemSendStr(const ext_client_custom_character_t ext_client_custom_character)
{
	RM_RefereeSystemData_t RM_RefereeSystemDataTemp = { 0 };
	//id设置
	ext_student_interactive_header_data_t ext_student_interactive_header_data = { 0 };
	ext_student_interactive_header_data.data_cmd_id = 0x0110;
	ext_student_interactive_header_data.sender_ID = ext_power_heat_data_0x0201.robot_id;
	ext_student_interactive_header_data.receiver_ID = RM_RefereeSystemGetRobotId();
	//数据格式
	RM_RefereeSystemDataTemp.SOF = 0xA5;
	RM_RefereeSystemDataTemp.data_length = 6+45;
	RM_RefereeSystemDataTemp.cmd_id = 0x0301;
	memcpy(&tx_buf,0,sizeof(tx_buf));
	//数据=id数据+图像数据
	int idx = 0;
	memcpy((void*)RM_RefereeSystemDataTemp.data,&ext_student_interactive_header_data,sizeof(ext_student_interactive_header_data));
	idx += sizeof(ext_student_interactive_header_data);
	memcpy((void*)(RM_RefereeSystemDataTemp.data + idx),&ext_client_custom_character.grapic_data_struct,sizeof(ext_client_custom_character.grapic_data_struct));
	idx += sizeof(ext_client_custom_character.grapic_data_struct);
	memcpy((void*)(RM_RefereeSystemDataTemp.data + idx),(void*)ext_client_custom_character.data,ext_client_custom_character.grapic_data_struct.end_angle);
	memcpy(tx_buf,&RM_RefereeSystemDataTemp,sizeof(RM_RefereeSystemDataTemp));
	Append_CRC8_Check_Sum(tx_buf,CRC8LEN);
	Append_CRC16_Check_Sum(tx_buf,CRC16LEN(RM_RefereeSystemDataTemp.data_length));
	HAL_UART_Transmit_IT( &RM_RefereeSystemHuart, tx_buf, sizeof(tx_buf) );
}



/*typedef __packed struct
{
uint16_t target_robot_id;
float target_position_x;
float target_position_y;
}map_robot_data_t;
#define CRC8LEN (5)
#define CRC16LEN(x) (x + 5 + 2 + 2)*/
uint8_t rrrdata[20] = {0xA5,0X0A,0X00,0X00,0XA9,0X05,0X03,0X03,0X00,0X11,0X6E,0X29,0X42,0XAB,0X0C,0X4B,0X41,0XC3,0X9D};
map_robot_data_t map_robot_data;
void RM_daohangtx(const map_robot_data_t map_robot_data)
{
	RM_RefereeSystemData_t RM_RefereeSystemDataTemp = { 0 };
	//数据格式
	RM_RefereeSystemDataTemp.SOF = 0xA5;
	RM_RefereeSystemDataTemp.data_length = 10;
	RM_RefereeSystemDataTemp.cmd_id = 0x0305;
	memcpy(&rrrdata,0,sizeof(rrrdata));
	
	
	memcpy((void*)(RM_RefereeSystemDataTemp.data),&map_robot_data,sizeof(map_robot_data));
	memcpy(rrrdata,&RM_RefereeSystemDataTemp,sizeof(RM_RefereeSystemDataTemp));
	Append_CRC8_Check_Sum(rrrdata,CRC8LEN);
	Append_CRC16_Check_Sum(rrrdata,CRC16LEN(RM_RefereeSystemDataTemp.data_length));
}
void RM_daohangtxParsea(uint8_t* data)
{
	RM_RefereeSystemData01.SOF = data[0];
	RM_RefereeSystemData01.data_length = data[1] | data[2] << 8;
	RM_RefereeSystemData01.seq = data[3];
	RM_RefereeSystemData01.CRC8 = data[4];
	RM_RefereeSystemData01.cmd_id = data[5] | data[6] << 8;
	for(uint16_t i = 0;i < RM_RefereeSystemData01.data_length;i++)
	{
		RM_RefereeSystemData01.data[i] = data[7 + i];
	}
	if(!Verify_CRC8_Check_Sum(data,CRC8LEN) || !Verify_CRC16_Check_Sum(data,CRC16LEN(RM_RefereeSystemData01.data_length)))
		return;
	//memcpy(&map_robot_data_1,(void*)RM_RefereeSystemData01.data,sizeof(map_robot_data_1));
}

//数据解析
void RM_RefereeSystemParseData(uint8_t* RM_pDatas,int size)
{
	RM_RefereeSystemData.SOF = RM_pDatas[0];
	RM_RefereeSystemData.data_length = RM_pDatas[1] | RM_pDatas[2] << 8;
	RM_RefereeSystemData.seq = RM_pDatas[3];
	RM_RefereeSystemData.CRC8 = RM_pDatas[4];
	RM_RefereeSystemData.cmd_id = RM_pDatas[5] | RM_pDatas[6] << 8;
	if(RM_RefereeSystemData.data_length > 50)
		return;
	for(uint16_t i = 0;i < RM_RefereeSystemData.data_length;i++)
	{
		RM_RefereeSystemData.data[i] = RM_pDatas[7 + i];
	}
	RM_RefereeSystemData.frame_tail = RM_pDatas[7 + RM_RefereeSystemData.data_length];
	if(!Verify_CRC8_Check_Sum(RM_pDatas,CRC8LEN) || !Verify_CRC16_Check_Sum(RM_pDatas,CRC16LEN(RM_RefereeSystemData.data_length)))
		return;
	switch (RM_RefereeSystemData.cmd_id)
	{
	case 0x0201:
		memcpy(&ext_power_heat_data_0x0201,(void*)RM_RefereeSystemData.data,sizeof(ext_power_heat_data_0x0201));
		break;
	case 0x0202:
		memcpy(&ext_power_heat_data_0x0202,(void*)RM_RefereeSystemData.data,sizeof(ext_power_heat_data_0x0202));
		break;
	case 0x0303:
		memcpy(&map_command_0x0303,(void*)RM_RefereeSystemData.data,sizeof(map_command_0x0303));
		break;
	default:
		break;
	}
} 
//消息获取
void RM_RefereeSystemGetData(uint8_t RM_pData)
{
	static int idx = 0,RM_pDataSize = 0;
	static uint8_t RM_pDataS[50] = { 0 };
	if(RM_pData == 0xA5)
	{
			RM_pDataSize++;
	}
	if(RM_pDataSize == 2)
	{
		RM_RefereeSystemParseData(RM_pDataS,idx);
		RM_pDataSize = 1;
		idx = 0;
	}
	if(RM_pDataSize == 1)
	{
		RM_pDataS[idx++] = RM_pData;
	}
}
//断连
bool RM_RefereeSystemDir()
{
	RM_RefereeSystemDirFlag = dirTime.ISDir(1000);
	if(RM_RefereeSystemDirFlag == false)RM_RefereeSystemInit();
	return RM_RefereeSystemDirFlag;
}
//解析
void RM_RefereeSystemParse(UART_HandleTypeDef *huart)
{
	if(huart == &RM_RefereeSystemHuart)
	{
			//更新断连时间
			dirTime.UpLastTime();
			//获取解析数据
			RM_RefereeSystemGetData(RM_RefereeSystemp8Data);
			HAL_UART_Receive_DMA(&RM_RefereeSystemHuart,&RM_RefereeSystemp8Data,sizeof(RM_RefereeSystemp8Data));
	}
}
}
