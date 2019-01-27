#include "Remote.h"
#include "main.h"
#include "Driver_Chassis.h"
#include "Ramp.h"
#include "Driver_Gimbal.h"
#include "Commondefine.h"

extern RC_Ctl_t RC_CtrlData;
extern RampGen_t LRSpeedRamp;   //mouse左右移动斜坡
extern RampGen_t FBSpeedRamp;
extern uint8_t RC_Update_Flag;

RC_Ctl_t RC_CtrlData;   

uint8_t RC_Update_Flag = 0;

void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t switch_cnt = 0;

	/* 最新状态值 */
	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	/* 取最新值和上一次值 */
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);


	/* 最老的状态值的索引 */
	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	/* 合并三个值 */
	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	/* 长按判断 */
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}

	if(switch_cnt >= 40)
	{
		sw->switch_long_value = sw->switch_value_buf[sw->buf_index]; 	
	}

	//索引循环
	sw->buf_last_index = sw->buf_index;
	sw->buf_index++;		
	if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		sw->buf_index = 0;	
	}			
}

uint8_t Remote_CheckJumpKey(uint16_t Key)
{
    if(RC_CtrlData.key.v & Key)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uint8_t get_RC=0;
void RemoteDataPrcess(uint8_t *pData)
{
    if(pData == NULL)
    {
        return;
    }
    RC_Update_Flag = 1;	
	
    RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
    RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                         ((int16_t)pData[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	
	RC_CtrlData.rc.ch0 -= (int16_t)REMOTE_CONTROLLER_STICK_OFFSET;
	RC_CtrlData.rc.ch1 -= (int16_t)REMOTE_CONTROLLER_STICK_OFFSET;
	RC_CtrlData.rc.ch2 -= (int16_t)REMOTE_CONTROLLER_STICK_OFFSET;
	RC_CtrlData.rc.ch3 -= (int16_t)REMOTE_CONTROLLER_STICK_OFFSET;
    
    RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

    RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

    RC_CtrlData.mouse.press_l = pData[12];
    RC_CtrlData.mouse.press_r = pData[13];
 
    RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);

}

//遥控器数据初始化，斜坡函数等的初始化
void RemoteTaskInit()
{
	//斜坡初始化
	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
	FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
	LRSpeedRamp.ResetCounter(&LRSpeedRamp);
	FBSpeedRamp.ResetCounter(&FBSpeedRamp);
	//底盘云台给定值初始化
	GimbalRef.Pitch = 0.0f;
	GimbalRef.Yaw = 0.0f;
	ChassisData.ChassisSpeedRef.Y = 0.0f;
	ChassisData.ChassisSpeedRef.X = 0.0f;
	ChassisData.ChassisSpeedRef.Omega = 0.0f;

}
