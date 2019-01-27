#ifndef __MAINIFOLD_H
#define __MAINIFOLD_H
#include "main.h"
typedef struct
{
//	float Yaw_angle_Q;//校准点坐标
//	float Pitch_angle_Q;
//	float Yaw_angle_S;
//	float Pitch_angle_S;
//	float Yaw_angle_C;
//	float Pitch_angle_C;
//	int16_t Mode;				//没有检测到为0 若检测到则为对应序号
//	uint8_t Used_Flag;		//妙算信息被使用过的标志 1：使用过   0：未使用
//	uint32_t TimeStamp;		//识别成功的时间戳
		float Yaw_Now;	//未处理的值
	  float Pitch_Now;
	  uint32_t Distance_Now;	//单位：厘米
	  uint8_t Flag_Get;
	uint8_t Used_Flag;		//妙算信息被使用过的标志 1：使用过   0：未使用
	uint32_t TimeStamp;		//识别成功的时间戳


}MiniPC_Res;

extern MiniPC_Res MiniPC_Data;

extern uint8_t Mainifold_count;


void Mainifold_Res_Anl(void);
void Mainifold_Res_Task(void);
void Send_MiniPC(void);

#endif

