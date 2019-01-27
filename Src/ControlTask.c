#include "ControlTask.h"
#include "Driver_Chassis.h"
#include "Driver_Gimbal.h"
#include "StatusMachine.h"
#include "imu.h"
#include "Driver_Beep.h"
#include "SuperviseTask.h"
#include "Driver_FriMotor.h"
#include "IOTask.h"
#include "CanBusTask.h"
#include "Ramp.h"

//extern AppParam_t gAppParamStruct;					//配置信息,这里保存着最新的校准值，并且与Flash中的内容同步
extern RampGen_t GMPitchRamp;
extern RampGen_t GMYawRamp;

uint32_t time_task_ms=0;
void ControlLoopTaskInit(void)
{
	time_task_ms=0;
	AppParamInit();
	Sensor_Offset_Param_Init(&gAppParamStruct);  
	
	GMPitchEncoder.round_cnt = 0;
	GMPitchEncoder.ecd_angle = 0;
	GMYawEncoder.round_cnt = 0;
	GMYawEncoder.ecd_angle = 0;
	//设置工作模式
	SetWorkState(PREPARE_STATE);
	//斜坡初始化
	GMPitchRamp.SetScale(&GMPitchRamp, PITCH_PREPARE_TIME_TICK_MS);
	GMYawRamp.SetScale(&GMYawRamp, YAW_PREPARE_TIME_TICK_MS);
	GMPitchRamp.ResetCounter(&GMPitchRamp);
	GMYawRamp.ResetCounter(&GMYawRamp);

	//云台给定角度初始化
	GimbalRef.Pitch = 0.0f;
	GimbalRef.Yaw = 0.0f;
	//Yaw轴角度初始化
	yaw_angle = 0;
	//yaw_angle_out=0;
	
  StatusMachine_Init();
}

void Control_Task(void)
{
			time_task_ms++;
	    CaliLoop();
	  	StatusMachine_Update();
     	Task_Monitor();
	    BEEP_Task();
	    LASER_Task();
    	IMU_Task();
    	GMControlLoop();
    	CMControlLoop();
	    FriMotor_Control();
	    Turntable_Control();
	
}