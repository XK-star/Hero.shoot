#ifndef  __STATUSMACHINE_H
#define  __STATUSMACHINE_H

typedef enum
{
	KEY_MOUSE_INPUT,
	STOP,
	REMOTE_INPUT,
}InputMode_e;

typedef enum
{												//															
  PREPARE_STATE,     		//上电后初始化状态 4s钟左右			
  NORMAL_AUTO_STATE,			//全自动模式
  NORMAL_RC_STATE,			//遥控器模式 //attackmode
	STOP_STATE,        	//停止运动状态   
	CALI_STATE,    			//校准状态
}WorkState_e;

typedef enum//									Notmal									
{															
	Attack_Normal,//开始麦轮
	Attack_Dragon,		//大符模式
	Attack_CatWalk,   //猫步
	Attack_Slow,     //补给站
	Attack_45Degree,  //45
	Attack_None,
}AttackMode_e;

typedef enum                 //lock
{
	GM_CM_Lock,
	GM_CM_Notlock,
}GM_CM_Mode_e;		//云台锁定底盘模式

typedef enum                 //perpare
{
	Gimbal_Prepare,
	Gimbal_Stop,
	Gimbal_RC_Mode,
	Gimbal_Mouse_Mode,
	Gimbal_Auto,
	Gimbal_Cali,
}Gimbal_MoveMode_t;

typedef enum
{
  Chassis_Prepare,
	Chassis_Stop,
	Chassis_RC_Mode,
	Chassis_Mouse_Mode,
	Chassis_Auto,            //终极全自动模式
}Chassis_MoveMode_t;

typedef enum
{
	FRICTION_WHEEL_OFF = 0,
	FRICTION_WHEEL_TURNNING = 1,
	FRICTION_WHEEL_ON = 2,
}FrictionWheelState_e;

typedef enum
{
	NOSHOOTING = 0,
	SHOOTING = 1,
	
}Shoot_State_e;

void StatusMachine_Update(void);

InputMode_e GetInputMode(void );
WorkState_e GetWorkState(void);
AttackMode_e Get_AttackMode(void);
GM_CM_Mode_e GetGM_CM_Mode(void);
Gimbal_MoveMode_t GetGimbal_MoveMode(void);
Chassis_MoveMode_t GetChassis_MoveMode(void);
FrictionWheelState_e GetFrictionState(void);
Shoot_State_e GetShootState(void);


void SetWorkState(WorkState_e workState);
	void StatusMachine_Init(void);


extern InputMode_e         InputMode;
extern WorkState_e         WorkState;
extern WorkState_e     lastWorkstate;
extern AttackMode_e       AttackMode;
extern GM_CM_Mode_e       GM_CM_Mode;
extern Gimbal_MoveMode_t  Gimbal_MoveMode;
extern Chassis_MoveMode_t  Chassis_MoveMode;
extern Shoot_State_e  ShootState;

#endif
