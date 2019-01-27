#include "Driver_FriMotor.h"
#include "main.h"
#include "tim.h"
#include "pid.h"
#include "StatusMachine.h"
#include "CanBusTask.h"
#include "gpio.h"
#include "SuperviseTask.h"

PID_Regulator_t ShootMotorSpeedPID = SHOOT_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t ShootMotorPositionPID = SHOOT_MOTOR_SPEED_PID_DEFAULT;

Turntale_ref_t Turntable_ref;

extern FrictionWheelState_e friction_wheel_state;

void LASER_Task(void)
{
	//无遥控器信号 激光2Hz闪烁
	if(DBUSFrameRate<10)
	{
		static uint32_t count = 0;
		count++;
		if(count > 500)
		{
			count = 0;
			LASER_TOGGLE();
		}
	}
	
	else
	{
		if(GetFrictionState() == FRICTION_WHEEL_ON)//摩擦轮开转 用激光来指示弹舱状态
		{
				LASER_ON();
			
		}
		else if(GetFrictionState() ==FRICTION_WHEEL_TURNNING)
		{
			LASER_TOGGLE();
		}
		else if(GetFrictionState() == FRICTION_WHEEL_OFF)
		{
			LASER_OFF();
		}
	}
}

/****************************************摩擦轮*****************************************/
//摩擦轮启动占空比为4%
static uint32_t Time_Mark;
void FriMotor_Control(void)
{                  
	switch(friction_wheel_state)  
	{
		case FRICTION_WHEEL_OFF:
		{	              
			SetFrictionWheelSpeed(800);
			Time_Mark = Get_Time_Micros();  
	
		}break;
		
		case FRICTION_WHEEL_TURNNING:
		{	

			SetFrictionWheelSpeed(BULLET_SPEED);               
			//起转超过1.5s 进入ON模式
			if(Get_Time_Micros() - Time_Mark > 1000000)
			{
				friction_wheel_state = FRICTION_WHEEL_ON;//强行将状态变成ON 
				                                                                                                    
			}
		}break;	
		case FRICTION_WHEEL_ON:
		{

			SetFrictionWheelSpeed(BULLET_SPEED);			
			
		}break;
		                                                                                                                          
		default:break;
	}
}

/****************************************拨盘****************************************/
uint8_t  BulletNumber = 0;
uint32_t Block_TimeStamp=0;
uint8_t BulletBlock_State=0;
	static uint32_t Stuck_Count = 0;
uint32_t ii = 0;
void BulletStuck_Check(void)
{
	if(abs(ShootMotorSpeedPID.ref) >1000 && abs(ShootMotorSpeedPID.fdb) <400)
	{
		Stuck_Count++;
		ii++;
	}
	else
	{
		Stuck_Count = 0;
	}
	
	if(Stuck_Count> 200)//这个数等于 卡弹判断阈值(ms)/函数调用周期(ms)
	{
		Block_TimeStamp = Get_Time_Micros();
		BulletBlock_State = 1;
//		Laser_Warnning(100);
	}
	else
	{
		BulletBlock_State = 0;
	}
}	
	
uint8_t pass_flag;
void Pass_Check()
{
	static uint8_t Passdoor = 1;                              
	if(Psensor_Check() && Passdoor)
	{   
		pass_flag = 1;
		Passdoor = 0;
	}
	else if(!Psensor_Check())
		Passdoor = 1;
	else
		pass_flag = 0;
}

float ShootNumber;
void ShootCounter(void)
{
	Pass_Check();
	if(pass_flag)
	{
		ShootNumber += 1;
	}
}   

float Turntable_Angle = 0;

//发射单颗子弹命令 每调用一次拨盘转90°
uint8_t shootdouble=0;
uint8_t shootnum=0;
void Shoot_Single(uint8_t num)
{
	
		if(Turntable_ref.Angle - TurntableEncoder.ecd_angle/REDUCTION <135.0f)
		{
//			Turntable_ref.Angle = PreShoot_Angle;//校准拨盘角度
			Turntable_ref.Angle += 90*1;		//转动一个子弹的角度
		}
	shootnum=num;
		if(num>1)
		
	 shootdouble=1;
}
void Turntable_Control(void)
{
	ShootMotorSpeedPID.kp = 9;
	ShootMotorSpeedPID.ki = 0.1;
	ShootMotorSpeedPID.kd = 3;
	ShootMotorPositionPID.kp = 200;
	ShootMotorPositionPID.ki = 0;//AppParamRealUsed.TurntablePositionPID.ki_offset/1000.0;
	ShootMotorPositionPID.kd = 0;//AppParamRealUsed.TurntablePositionPID.kd_offset;
	Turntable_ref.Speed = 7000;
//	BulletTurntableSpeedPID.kp = 5500;
//	BulletTurntableSpeedPID.ki = 0;         
//	BulletTurntableSpeedPID.kd = 0;
	
//	uint8_t Attack_Change_Flag = Attack_Switch_Check();
	BulletStuck_Check();
//	BulletStuck_Check_up();
//	if(Attack_Change_Flag == 1 || Get_Time_Micros() <4000000)
//	{
		if(Get_Time_Micros() <4000000)
	{
		Turntable_ref.Angle = TurntableEncoder.ecd_angle/REDUCTION;
	}
//	}
	static InputMode_e last_Input;
	static InputMode_e now_Input;
	if(GetFrictionState() == FRICTION_WHEEL_ON)
	{	
		now_Input = GetInputMode();
		if(last_Input != now_Input)
		{                                                             
	 		last_Input = now_Input;                                    
			Turntable_ref.Angle = TurntableEncoder.ecd_angle/36.0f;
			//BulletTurntable_ref.Angle = BulletEncoder.ecd_angle/19.0f;
		}
		if(GetInputMode() == REMOTE_INPUT)	
		{
			if(Get_Time_Micros() - Block_TimeStamp < 200000) //正在回拨时间内
			{
				PID_Task(&ShootMotorSpeedPID, -Turntable_ref.Speed, Turntable_Measure.speed_rpm);		
			}
			else
			{
				PID_Task(&ShootMotorPositionPID, Turntable_ref.Angle, TurntableEncoder.ecd_angle/REDUCTION);
			  PID_Task(&ShootMotorSpeedPID, ShootMotorPositionPID.output, Turntable_Measure.speed_rpm);
			}
			
		} 
		if(GetInputMode() == KEY_MOUSE_INPUT)	
		{
			
		  	ShootCounter();//光电门计数
			
			if(Get_Time_Micros() - Block_TimeStamp < 200000) //正在回拨时间内
			{
				PID_Task(&ShootMotorSpeedPID, -Turntable_ref.Speed, Turntable_Measure.speed_rpm);		
			}
			
			else
			{
				if(ShootNumber < BulletNumber)
				{	
					PID_Task(&ShootMotorSpeedPID, Turntable_ref.Speed, Turntable_Measure.speed_rpm);
				}
				else
				{
					PID_Task(&ShootMotorSpeedPID, 0, Turntable_Measure.speed_rpm);
				}
			}	
						
//			if(Remote_CheckJumpKey(KEY_E))
//			{
//				PID_Task(&BulletTurntableSpeedPID, BulletTurntableSpeed, Bullet_Measure.speed_rpm/10.0); 
//			}
//			
//			else
//			{
//				if(Get_Time_Micros() - BulletBlock_TimeStamp < 450000) //正在回拨时间内
//				{
//					PID_Task(&BulletTurntableSpeedPID, -BulletTurntableSpeed, Bullet_Measure.speed_rpm/10.0);		
//				}
//				else
//				{   
//					PID_Task(&BulletTurntableSpeedPID, BulletTurntableSpeed, Bullet_Measure.speed_rpm/10.0);
//				}
//			}
			}
		 
		else  
		{
			if(GetShootState() == SHOOTING)	//普通模式模式 单速度环
			{	
//				if(Get_Time_Micros() - BulletBlock_TimeStamp < 450000) //正在回拨时间内
//				{
//					PID_Task(&BulletTurntableSpeedPID, -BulletTurntableSpeed, Bullet_Measure.speed_rpm/10.0);		
//				}
//				else
//				{
//					PID_Task(&BulletTurntableSpeedPID, BulletTurntableSpeed, Bullet_Measure.speed_rpm/10.0);
//				}
//			}
//			else
//			{
			//	PID_Task(&ShootMotorSpeedPID, 0, Turntable_Measure.speed_rpm);
//				PID_Task(&BulletTurntableSpeedPID, 0, 0);
//			}                                       
		}
	}
}
	else
	{	
		ShootMotorSpeedPID.Reset(&ShootMotorSpeedPID);
		PID_Task(&ShootMotorSpeedPID, 0, 0);
		
//		BulletTurntableSpeedPID.Reset(&BulletTurntableSpeedPID);
//		PID_Task(&BulletTurntableSpeedPID, 0, 0); 
	}
	
	
}