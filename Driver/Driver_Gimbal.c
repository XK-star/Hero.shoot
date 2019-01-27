#include "Driver_Gimbal.h"
#include "CanBusTask.h"
#include "pid.h"
#include "Remote.h"
#include "imu_out.h"
#include "StatusMachine.h"
#include "tim.h"
#include "Commondefine.h"
#include "Ramp.h" 
#include "gpio.h"
#include "Driver_Minifold.h"
#include "DM9015.h"
#include "test_imu.h"
#include "IOTask.h"

#define ANGLE_Test  //测试角度

RampGen_t  GMPitchRamp=RAMP_GEN_DAFAULT;
RampGen_t  GMYawRamp=RAMP_GEN_DAFAULT;

PID_Regulator_t GMPPositionPID = GIMBAL_MOTOR_PITCH_POSITION_PID_DEFAULT;
PID_Regulator_t GMPSpeedPID = GIMBAL_MOTOR_PITCH_SPEED_PID_DEFAULT;
PID_Regulator_t GMYPositionPID = GIMBAL_MOTOR_YAW_POSITION_PID_DEFAULT;
PID_Regulator_t GMYSpeedPID = GIMBAL_MOTOR_YAW_SPEED_PID_DEFAULT;


Gimbal_Ref_t GimbalRef;
extern float yaw_angle,pitch_angle,roll_angle; //使用到的角度值
/**
  * @note   modified
  * @brief  云台Yaw角度设置
  * @param  目标角度
  * @param  模式 AngleMode_REL(相对当前位置 每次调用 都会根据当前角度和输入参数来改变参考角度)     AngleMode_ABS     AngleMode_ECD 编码器角度
  * @retval void

  */
void Gimbal_YawAngleSet(float Target, AngleMode_Enum mode)
{
	if (mode == AngleMode_REL)
	{
		Target += yaw_angle;
	}
	if (mode == AngleMode_ECD)
	{
		Target += yaw_angle;
		Target += GMYawEncoder.ecd_angle;
	}
	GimbalRef.Yaw = Target;
}

/**
  * @note   modified
  * @brief  云台Pitch角度及模式设置
  * @param  目标角度(绝对角度）
  * @retval void
  * @note   Pitch
  */
void Gimbal_PitchAngleSet(float Target, AngleMode_Enum mode)
{
	if (mode == AngleMode_REL)
	{
		Target = GMPitchEncoder.ecd_angle-Target ;
	}
	GimbalRef.Pitch = Target;
}

//插值
//遥控器帧率70Hz 控制频率1K  插值倍率14.2857
#define INSERT_YAW 0
#define INSERT_PITCH 1
//Mode 1输入 0获取
float Value_Insert(uint8_t WHO, float value, uint8_t Mode)
{
	static float last_value_yaw = 0, now_value_yaw = 0;
	static uint8_t yaw_count = 0;
	static float last_value_pitch = 0, now_value_pitch = 0;
	static uint8_t pitch_count = 0;
	if(Mode == 1)//输入新值
	{
		if(WHO == INSERT_YAW)
		{
			last_value_yaw = now_value_yaw;
			now_value_yaw = value;
			yaw_count = 0;
		}
		else	//INSERT_PITCH
		{
			last_value_pitch = now_value_pitch;
			now_value_pitch = value;
			pitch_count = 0;
		}
		return 0;
	}
	else//获取插值后数据
	{
		if(WHO == INSERT_YAW)
		{
			float result = last_value_yaw + (now_value_yaw - last_value_yaw)*(yaw_count)/14.0f;
			if(yaw_count<14)
				yaw_count++;
			return result;
		}
		else	//INSERT_PITCH
		{
			float result = last_value_pitch + (now_value_pitch - last_value_pitch)*(pitch_count)/14.0f;
			if(pitch_count<14)
				pitch_count++;
			return result;	
		}
	}
}
static void Ref_Gimbal_Prepare(void)//恢复正中斜坡函数
{
 	Gimbal_PitchAngleSet(GMPitchEncoder.ecd_angle * GMPitchRamp.Calc(&GMPitchRamp), AngleMode_REL);
	Gimbal_YawAngleSet(GMYawEncoder.ecd_angle * GMYawRamp.Calc(&GMYawRamp), AngleMode_REL);

		
}

static void Ref_UpdataFromRCStick(void)
{
		Gimbal_PitchAngleSet(GimbalRef.Pitch + (RC_CtrlData.rc.ch3) * abs(RC_CtrlData.rc.ch3) / 6600.0 * STICK_TO_PITCH_ANGLE_INC_FACT, AngleMode_ABS);
		Gimbal_YawAngleSet(GimbalRef.Yaw + (RC_CtrlData.rc.ch2) * abs((RC_CtrlData.rc.ch2)) / 6600.0 * STICK_TO_YAW_ANGLE_INC_FACT, AngleMode_ABS); //非线性

}

float pitch_set=0;
float yaw_set=0;
static void Ref_UpdataFromMouse(void)
{
	#ifdef ANGLE_Test
			Gimbal_PitchAngleSet(pitch_set, AngleMode_ABS);
			Gimbal_YawAngleSet(yaw_set, AngleMode_ABS);
	#else
		if(MiniPC_Data.Flag_Get ==1)
	{
			Gimbal_PitchAngleSet(MiniPC_Data.Yaw_Now, AngleMode_ABS);
			Gimbal_YawAngleSet(MiniPC_Data.Pitch_Now, AngleMode_ABS);

	}
	else
	{
		if (RC_Update_Flag == 1)
		{
			Gimbal_PitchAngleSet(GimbalRef.Pitch - RC_CtrlData.mouse.y * MOUSE_TO_PITCH_ANGLE_INC_FACT, AngleMode_ABS);
			Gimbal_YawAngleSet(GimbalRef.Yaw + RC_CtrlData.mouse.x * MOUSE_TO_YAW_ANGLE_INC_FACT, AngleMode_ABS);
			RC_Update_Flag = 0;
		}
	}

	#endif

}

float pp_pitch=25;
float pi_pitch=0;
float pd_pitch=0;
float sp_pitch=2;
float si_pitch=0.0005;
float sd_pitch=0.02;

/**
  * @brief  使用上位机给的PID值
  * @param  None
  * @retval None
  */
static void PID_Calibration(void)
{
	GMYPositionPID.kp = AppParamRealUsed.YawPositionPID.kp_offset;
	GMYPositionPID.ki = AppParamRealUsed.YawPositionPID.ki_offset / 1000.0;
	GMYPositionPID.kd = AppParamRealUsed.YawPositionPID.kd_offset;

	GMYSpeedPID.kp = AppParamRealUsed.YawSpeedPID.kp_offset;
	GMYSpeedPID.ki = AppParamRealUsed.YawSpeedPID.ki_offset / 1000.0;
	GMYSpeedPID.kd = AppParamRealUsed.YawSpeedPID.kd_offset;

	GMPPositionPID.kp =AppParamRealUsed.PitchPositionPID.kp_offset;
	GMPPositionPID.ki = AppParamRealUsed.PitchPositionPID.ki_offset / 1000.0;
	GMPPositionPID.kd = AppParamRealUsed.PitchPositionPID.kd_offset;

	GMPSpeedPID.kp = AppParamRealUsed.PitchSpeedPID.kp_offset;
	GMPSpeedPID.ki = AppParamRealUsed.PitchSpeedPID.ki_offset / 1000.0;
	GMPSpeedPID.kd = AppParamRealUsed.PitchSpeedPID.kd_offset;

//	GMPPositionPID.kp =pp_pitch;
//	GMPPositionPID.ki = pi_pitch;
//	GMPPositionPID.kd = pd_pitch;

//	GMPSpeedPID.kp = sp_pitch;
//	GMPSpeedPID.ki = si_pitch;
//	GMPSpeedPID.kd = sd_pitch;

}
/**
  * @note   modified
  * @brief  云台PID计算
  * @param  void
  * @retval void
  * @note   
  */
void GMCalLoop(void)
{
		if (InputMode == STOP)
	{                                                           
		GMYSpeedPID.output = 0;
		GMPSpeedPID.output = 0;
	}
	else
	{

//		PID_Task(&GMYPositionPID, GimbalRef.Yaw, yaw_angle);
//		PID_Task(&GMYSpeedPID, GMYPositionPID.output, MPU6050_Real_Data.Gyro_Yaw);
		
		//PID_Task(&GMPPositionPID, GimbalRef.Pitch, -GMPitchEncoder.ecd_angle/27.0f);//原版
		PID_Task(&GMPPositionPID, GimbalRef.Pitch, GMPitchEncoder.ecd_angle);//尝试
		PID_Task(&GMPSpeedPID, GMPPositionPID.output, -MPU6050_Real_Data.Gyro_Pitch);
	}
}
/**
  * @note   modified
  * @brief  电流值通过CAN发送给电机
  * @param  void
  * @retval void
  * @note   
  */
int16_t test_yangle=0;
void SetGimbalMotorOutput(void)
{
	test_yangle++;
	if ((GetWorkState() == STOP_STATE) || GetWorkState() == CALI_STATE)
	{
		Set_Gimbal_Current(&hcan1,0,0);
		if(test_yangle%4==0)//DM90帧率低
		{
//		DM_SendAngle(GimbalRef.Yaw *100);
		//	DM_SendPower(0)
		test_yangle=0	;
		}
		
		
	}
	else
	{
		
		Set_Gimbal_Current(&hcan1, 0,-(int16_t)GMPSpeedPID.output); //
//		DM_SendPower((int16_t)GMYSpeedPID.output*RM66TODM90);
//		DM_SendPower((int16_t)RC_CtrlData.rc.ch3);
		if(test_yangle%4==0)//DM90帧率低
		{
//		DM_SendAngle(GimbalRef.Yaw *100);
		//	DM_SendAngle((int16_t)RC_CtrlData.rc.ch3/3*100);
		test_yangle=0	;
		}
	}
		
}
uint8_t limit_flag = 0;
uint8_t limit_counter = 0;
float limitoffset = 0;
uint32_t open_time = 0;
int16_t test1=0;

void GimbalGetRef(void)
{
	Gimbal_MoveMode_t MoveMode_Now = GetGimbal_MoveMode();
	if (MoveMode_Now == Gimbal_Stop)
	{
		
	}
	else if (MoveMode_Now == Gimbal_Prepare)
	{
		Ref_Gimbal_Prepare();
	}
	else if (MoveMode_Now == Gimbal_RC_Mode)
	{
		Ref_UpdataFromRCStick();
	}
	else if (MoveMode_Now == Gimbal_Mouse_Mode)
	{
		Ref_UpdataFromMouse();
	}
	else if (MoveMode_Now == Gimbal_Auto)
	{
		
	}
	
	//-------------------------------云台限位---------------------------------------------------------------
	if (GetGimbal_MoveMode() != Gimbal_Prepare)
	{
		if (GetGM_CM_Mode() == GM_CM_Lock) //云台解锁时 云台不限位 地盘超过角度后跟随
		{
			VAL_LIMIT(GimbalRef.Yaw,
					  yaw_angle + GMYawEncoder.ecd_angle - YAW_MAX,
					  yaw_angle + GMYawEncoder.ecd_angle + YAW_MAX);
		}
	}
	
	if (GetGimbal_MoveMode() != Gimbal_Prepare)
	{
		VAL_LIMIT(GimbalRef.Pitch,
				  -23,
				  35);
	}
		/*3510
	//pitch轴限位--->相对的转动角度 - 陀螺仪初始角度
	if(limit_flag)
	{	
			VAL_LIMIT(GimbalRef.Pitch,
					-25 - limitoffset,
					55 - limitoffset);
	}
	
	//陀螺仪数据上电为0，需等一段时间再使用
	open_time = Get_Time_Micros();
	if(open_time > 900000 && limit_counter < 1)
	{
		if(fabs(pitch_angle_out) < 0.1)
		{
			limitoffset = GMPitchEncoder.ecd_angle/27.0f;
			limit_counter++;
			limit_flag = 1;
		} 
	}
	*/
}

void GMControlLoop(void)
{
	PID_Calibration();
	GimbalGetRef(); //获取参考值
	GMCalLoop();	//根据模式对应不同
	SetGimbalMotorOutput();
}
