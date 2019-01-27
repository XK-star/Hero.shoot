#ifndef __CHASSISTASK_H
#define __CHASSISTASK_H
#include "main.h"
#include "Commondefine.h"

#define Ang2Rad(m)  (m/180.0f*M_PI)

//键盘模式底盘速度
#define NORMAL_FORWARD_BACK_SPEED 			400
#define NORMAL_LEFT_RIGHT_SPEED   			300
#define HIGH_FORWARD_BACK_SPEED 			600
#define HIGH_LEFT_RIGHT_SPEED   			500
#define LOW_FORWARD_BACK_SPEED 				200
#define LOW_LEFT_RIGHT_SPEED   				150


typedef __packed struct
{
	int16_t Y;
	int16_t X;
	int16_t Omega;
}ChassisSpeed_Ref_TypeDef;

typedef struct
{
	float ChassisAngle;				
	int16_t ChassisWheelSpeedRef[4];//车右前方逆时针数分别为1 2 3 4号电机
	ChassisSpeed_Ref_TypeDef ChassisSpeedRef;
}ChassisDataTypeDef;//底盘数据结构

extern ChassisDataTypeDef ChassisData;

void CM_SetOutput(void);
void CMControlLoop(void);
void CM_PID_Cal(void);


#endif
