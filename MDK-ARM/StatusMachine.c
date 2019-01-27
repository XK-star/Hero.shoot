#include "StatusMachine.h"
#include "Remote.h"
#include "ControlTask.h"
#include "Driver_FriMotor.h"
#include "tim.h"
#include "gpio.h"
#include "Remote.h"
#include "SuperviseTask.h"


InputMode_e                InputMode;
WorkState_e                WorkState;
WorkState_e            lastWorkstate;
AttackMode_e              AttackMode;
GM_CM_Mode_e              GM_CM_Mode;
Gimbal_MoveMode_t    Gimbal_MoveMode;
Chassis_MoveMode_t  Chassis_MoveMode;
Shoot_State_e            ShootState;
FrictionWheelState_e  friction_wheel_state;

RemoteSwitch_t switch1;   //遥控器左侧拨杆

extern uint8_t BulletNumber;

/**************************************输入端*****************************************/
void InputMode_Select(void)
{
	if(DBUSFrameRate<10)
	{
		InputMode = STOP;
	}
	else{
		if(RC_CtrlData.rc.s2 == REMOTE_SWITCH_VALUE_UP)
				{
					InputMode = REMOTE_INPUT;
				}
				else if(RC_CtrlData.rc.s2 == REMOTE_SWITCH_VALUE_CENTRAL)
				{
					InputMode = KEY_MOUSE_INPUT;
				}
				else if(RC_CtrlData.rc.s2 == REMOTE_SWITCH_VALUE_DOWN)
				{
					InputMode = STOP;
				}
	}
				
		
}

InputMode_e GetInputMode(void )
{
	return InputMode;
}
/**************************************工作模式****************************************/

static void WorkStateSwitchProcess(void)//如果从其他模式切换到prapare模式，要将一系列参数初始化
{
	if((lastWorkstate!= WorkState) && (WorkState == PREPARE_STATE))  
	{
		ControlLoopTaskInit();
		RemoteTaskInit();
	}
}
void WorkstateFSM(void)//state只是一个类似的监督机制，attack_mode 是真正的控制最上端。
{
	lastWorkstate  = WorkState;
	
	if(InputMode == STOP)
		WorkState=STOP_STATE;
	else if(InputMode == REMOTE_INPUT)
		WorkState=  NORMAL_RC_STATE;
 
	
	switch(WorkState)
	{
		case PREPARE_STATE:
		{		
			if(time_task_ms > PREPARE_TIME_TICK_MS)
			{
					WorkState = NORMAL_RC_STATE;
			}
		}break;
		case NORMAL_RC_STATE:
		{
			if(GetInputMode()==STOP)
				WorkState=STOP_STATE;
		}break ;
		case STOP_STATE :
		{
			if(GetInputMode()!=STOP)
				WorkState=PREPARE_STATE;
		}break ;
	}
		WorkStateSwitchProcess();//模式改变初始化
}
WorkState_e GetWorkState(void)
{
	return WorkState;
}
void SetWorkState(WorkState_e workState)
{
	WorkState=workState;
}
/**************************************攻击方式****************************************/

void Attack_Mode_Select(void)
{
	if(GetInputMode() == KEY_MOUSE_INPUT)
	{
				
		if(RC_CtrlData.mouse.press_r == 1)
		{
			GM_CM_Mode = GM_CM_Notlock; //按下右键 解锁云台
		}								
		else							//松开右键 锁定云台
		{
			GM_CM_Mode = GM_CM_Lock;	
		}
		
	}
}

AttackMode_e Get_AttackMode(void)
{
	return AttackMode;
}

GM_CM_Mode_e GetGM_CM_Mode(void)
{
	return GM_CM_Mode;
}
/**************************************云台*****************************************/

void GimbalModeSelect(void)
{
	if(!Check_Button())
	{
			switch(WorkState )
			{
				case PREPARE_STATE:   
				{
					Gimbal_MoveMode = Gimbal_Prepare;//云台归中
				}	break;
				case NORMAL_RC_STATE:
				{
					if(GetInputMode() == REMOTE_INPUT)
					{
						Gimbal_MoveMode = Gimbal_RC_Mode;
					}
					else if(GetInputMode() == KEY_MOUSE_INPUT)
					{
							Gimbal_MoveMode = Gimbal_Mouse_Mode;	
					}
				}break ;
				case STOP_STATE :
				{
					Gimbal_MoveMode=Gimbal_Stop;
				}break ;
				case CALI_STATE:    
				{                                 
					Gimbal_MoveMode = Gimbal_Stop;
				}break;	
				
				default:break;
			}
	 }
//	else 
//	{
//		HAL_Delay (100);
//		if(Check_Button ())
//	     Gimbal_MoveMode = Gimbal_Cali;
//	}
}
Gimbal_MoveMode_t GetGimbal_MoveMode(void)
{	
	return Gimbal_MoveMode;
}

/**************************************底盘*****************************************/

void ChassisModeSelect(void)
{
	switch(WorkState )
	{
		case PREPARE_STATE:   
		{
			Chassis_MoveMode = Chassis_Prepare;
		}	break;
	  case NORMAL_RC_STATE:
		{
			if(GetInputMode() == REMOTE_INPUT)
			{
				Chassis_MoveMode = Chassis_RC_Mode;
			}
			else if(GetInputMode() == KEY_MOUSE_INPUT)
			{
					Chassis_MoveMode = Chassis_Mouse_Mode;	
			}
		}break ;
		case STOP_STATE :
		{
		  Chassis_MoveMode=Chassis_Stop;
		}break ;
		case CALI_STATE:    
		{                                 
			Chassis_MoveMode = Chassis_Stop;
		}break;	
		
		default:break;
	}
}
Chassis_MoveMode_t GetChassis_MoveMode(void)
{
	return Chassis_MoveMode;
}

/**************************************摩擦轮****************************************/

FrictionWheelState_e GetFrictionState(void)
{
	return friction_wheel_state;
}

void SetFrictionState(FrictionWheelState_e v)
{
	if(v == FRICTION_WHEEL_ON)
	{
		//尚未完全起转
		if(friction_wheel_state == FRICTION_WHEEL_TURNNING || friction_wheel_state == FRICTION_WHEEL_OFF)
		{
			friction_wheel_state = FRICTION_WHEEL_TURNNING;
		}
	}
	else
	{
		friction_wheel_state = v;
	}
}
void FriMotor_Select(void)
{	
	switch(GetWorkState())
	{
		case PREPARE_STATE:
		{
			SetFrictionState(FRICTION_WHEEL_OFF);
		}break;
		case NORMAL_RC_STATE:     
		{			
       static uint32_t Change_TimeStamp = 0;	
			
			if(InputMode == REMOTE_INPUT)
			{
				if(Get_Time_Micros() - Change_TimeStamp > 100000)
				{
					if(GetFrictionState() == FRICTION_WHEEL_OFF)
					{
						if(switch1.switch_value1== REMOTE_SWITCH_CHANGE_3TO1)
						{
							SetFrictionState(FRICTION_WHEEL_ON);
							Change_TimeStamp = Get_Time_Micros();
						}
					}
					else
					{
						if(switch1.switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)
						{
							SetFrictionState(FRICTION_WHEEL_OFF);
							Change_TimeStamp = Get_Time_Micros();
						}
					}
				}
			}			
			else if(InputMode == KEY_MOUSE_INPUT)
			{
				//R 打开摩擦轮
				if(Remote_CheckJumpKey(KEY_R) == 1 && (Get_Time_Micros() - Change_TimeStamp > 1000000))
				{
					Change_TimeStamp = Get_Time_Micros();
					if(GetFrictionState() == FRICTION_WHEEL_ON)
					{
						SetFrictionState(FRICTION_WHEEL_OFF);
					}
					else
					{
						SetFrictionState(FRICTION_WHEEL_ON);
					}
				}
				
			}
		}
	}
}

/**************************************拨盘*****************************************/
Shoot_State_e GetShootState(void)
{
	return ShootState;
}

void SetShootState(Shoot_State_e v)
{
	ShootState = v;
}

uint16_t wait_time=200;
void Turntable_Select(void)
{
	switch(WorkState )
	{
		
		case PREPARE_STATE:   
		{
			ShootState = NOSHOOTING;
		}	break;
	  case NORMAL_RC_STATE:
		{
			if(shootdouble)//双发
							{
								wait_time --;
								
								if(wait_time ==0)
								{
									shootnum--;
								Shoot_Single(1);
									wait_time =230;
									if(shootnum==1)
									{
										shootdouble=0;
									}
									
								}
							}				
			if(GetInputMode() == REMOTE_INPUT)
			{
				switch(RC_CtrlData.rc.s1)
				{		
					case REMOTE_SWITCH_VALUE_DOWN:
					{
						static uint32_t rctime_stamp = 0;

						if(GetFrictionState() == FRICTION_WHEEL_ON)
						{
							SetShootState(SHOOTING);                            
						}
						else
						{
							SetShootState(NOSHOOTING);
						}  
				 if(Get_Time_Micros() - rctime_stamp  > 100000)
					{
						if(GetFrictionState() == FRICTION_WHEEL_ON)
						{
							
							if(switch1.switch_value1== REMOTE_SWITCH_CHANGE_3TO2&&RC_CtrlData.rc.ch0>600)
							{
								Shoot_Single(10);
								rctime_stamp  = Get_Time_Micros();
							}
							else if(switch1.switch_value1== REMOTE_SWITCH_CHANGE_3TO2)
							{
								Shoot_Single(1);
								rctime_stamp  = Get_Time_Micros();
							}
							
						}
						
					}
         	
					}break;  
					//case REMOTE_SWITCH_VALUE_CENTRAL:
//					{static uint32_t rctime_stamp = 0;
//						if(Get_Time_Micros() - rctime_stamp  > 100000)
//					{
//						
//          if(RC_CtrlData.rc.ch0>600)
//							{
//								Shoot_Single(10);
//								rctime_stamp  = Get_Time_Micros();
//							}
//							if(RC_CtrlData.rc.ch1>600)
//							{
//								Shoot_Single(2);
//								rctime_stamp  = Get_Time_Micros();
//							}			
//						}							
//						}							
				}
		  }
			else if(GetInputMode() == KEY_MOUSE_INPUT)
			{
//								if (GetFrictionState() == FRICTION_WHEEL_ON)		//普通模式 单环速度控制 
//				{
//					static uint32_t time_stamp = 0;
//					if(RC_CtrlData.mouse.press_l == 1 && (Get_Time_Micros() - time_stamp > 300000))
//					{
//						time_stamp = Get_Time_Micros();
//						BulletNumber ++;
//					}
//				}
				static uint32_t rctime_stamp = 0;
				if(Get_Time_Micros() - rctime_stamp  > 100000)
					{
						
							if(switch1.switch_value1== REMOTE_SWITCH_CHANGE_3TO2)
							{
								Shoot_Single(1);
								rctime_stamp  = Get_Time_Micros();
							}
							
					
						
					}
			}
		}break ;
    case NORMAL_AUTO_STATE:     
		{
			//交由云台程序控制
		}break;		
		
		default:
		{
			SetShootState(NOSHOOTING);
		}break;
	}
	
}

	void StatusMachine_Init(void)
{
	WorkState = PREPARE_STATE;
	AttackMode = Attack_Normal;
	InputMode = STOP;
	Gimbal_MoveMode = Gimbal_Stop;
	Chassis_MoveMode = Chassis_Stop;
	friction_wheel_state = FRICTION_WHEEL_OFF;
	ShootState = NOSHOOTING;
	
	
}
void StatusMachine_Update(void)
{
	  InputMode_Select();
  	GetRemoteSwitchAction(&switch1, RC_CtrlData.rc.s1);	
  	WorkstateFSM();
  	Attack_Mode_Select();
  	GimbalModeSelect();
  	ChassisModeSelect();
  	FriMotor_Select();
  	Turntable_Select();
	
}
