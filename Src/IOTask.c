#include "IOTask.h"
#include "StatusMachine.h"
#include "gpio.h"
#include "flash.h"
#include "CanBusTask.h"
#include "test_imu.h"


AppParam_t gAppParamStruct;					//配置信息,这里保存着最新的校准值，并且与Flash中的内容同步
AppParam_t AppParamRealUsed;        //实际使用的
AppParam_t PIDCaliData;  //保存pitch轴position校准值

static GimbalCaliStruct_t  GimbalCaliData;   //保存云台编码器偏差值
static GyroCaliStruct_t GyroCaliData;        //保存校准陀螺仪偏差值

uint8_t app_param_calied_flag = 0;
uint8_t Gimbal_Cali_Success = 0;


//用于保存数据到flash中
static uint8_t AppParamSave(void)
{
    uint8_t retval = 1;   
    retval = BSP_FLASH_Write(PARAM_SAVED_START_ADDRESS, (uint8_t *)&gAppParamStruct, sizeof(AppParam_t));    
    if(retval == 0)
    {
//		Set_Error_Flag(LOST_ERROR_FLASH);
//    }
//	else
//	{
//		Reset_Error_Flag(LOST_ERROR_FLASH);
	}
    return retval;   
}

//用于从flash中读取校准数据
void AppParamInit(void)
{
    AppParam_t tmp_param;
    
    memcpy(&tmp_param, (void *)PARAM_SAVED_START_ADDRESS, sizeof(AppParam_t));		//读取flash中的数据
    
	{
		app_param_calied_flag =1;
        memcpy(&gAppParamStruct, &tmp_param, sizeof(AppParam_t));
    }
    if(gAppParamStruct.GimbalCaliData.GimbalCaliFlag != PARAM_CALI_DONE)
    {
        gAppParamStruct.GimbalCaliData.GimbalCaliFlag = PARAM_CALI_NONE;
        gAppParamStruct.GimbalCaliData.GimbalPitchOffset = 0;
        gAppParamStruct.GimbalCaliData.GimbalYawOffset = 0;
    }
		
    //目前不用
//    if(gAppParamStruct.GyroCaliData.GyroCaliFlag != PARAM_CALI_DONE)
//    {
//        gAppParamStruct.GyroCaliData.GyroCaliFlag = PARAM_CALI_NONE;
//        gAppParamStruct.GyroCaliData.GyroXOffset = 0;
//        gAppParamStruct.GyroCaliData.GyroYOffset = 0;
//        gAppParamStruct.GyroCaliData.GyroZOffset = 0;
//    }
//    
//    if(gAppParamStruct.AccCaliData.AccCaliFlag != PARAM_CALI_DONE)
//    {
//        gAppParamStruct.AccCaliData.AccCaliFlag = PARAM_CALI_NONE;
//        gAppParamStruct.AccCaliData.AccXOffset = 0;
//        gAppParamStruct.AccCaliData.AccYOffset = 0;
//        gAppParamStruct.AccCaliData.AccZOffset = 0;
//        gAppParamStruct.AccCaliData.AccXScale = 1.0;
//        gAppParamStruct.AccCaliData.AccYScale = 1.0;
//        gAppParamStruct.AccCaliData.AccZScale = 1.0;
//    }
//    
//    if(gAppParamStruct.MagCaliData.MagCaliFlag != PARAM_CALI_DONE)
//    {
//        gAppParamStruct.MagCaliData.MagCaliFlag = PARAM_CALI_NONE;
//        gAppParamStruct.MagCaliData.MagXOffset = 0;
//        gAppParamStruct.MagCaliData.MagYOffset = 0;
//        gAppParamStruct.MagCaliData.MagZOffset = 0;
//        gAppParamStruct.MagCaliData.MagXScale = 1.0;
//        gAppParamStruct.MagCaliData.MagYScale = 1.0;
//        gAppParamStruct.MagCaliData.MagZScale = 1.0;
//    }
}

void SetGimbalCaliData(GimbalCaliStruct_t *cali_data)
{
	if(cali_data != NULL)
    {
		memcpy(&gAppParamStruct.GimbalCaliData, cali_data, sizeof(*cali_data));
		AppParamSave();
	}
}	



void GetGimbalCaliData(GimbalCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParamStruct.GimbalCaliData, sizeof(GimbalCaliStruct_t));
    }
}

uint8_t IsGimbalCalied(void)
{
    return (gAppParamStruct.GimbalCaliData.GimbalCaliFlag == PARAM_CALI_DONE);
}

uint32_t CaliCmdFlagGrp = 0;     

void SetCaliCmdFlag(uint32_t flag)  //设置校准标志位
{
	CaliCmdFlagGrp |= flag;
}

void ResetCaliCmdFlag(uint32_t flag)
{
	CaliCmdFlagGrp &= ~flag;
}

uint8_t IsCaliCmdFlagSet(uint32_t flag)//判断校准标志位
{
	if(flag & CaliCmdFlagGrp)
	{
		return 1;
	}else
	{
		return 0;	
	}
}

void SetPIDCaliData(AppParam_t *cali_data)
{
	if(cali_data != NULL)
    {	
		memcpy(&gAppParamStruct.PitchPositionPID, &cali_data->PitchPositionPID, sizeof(cali_data->PitchPositionPID));
		memcpy(&(gAppParamStruct.PitchSpeedPID), &(cali_data->PitchSpeedPID), sizeof((cali_data->PitchSpeedPID)));
		memcpy(&(gAppParamStruct.YawPositionPID), &(cali_data->YawPositionPID), sizeof((cali_data->YawPositionPID)));
		memcpy(&(gAppParamStruct.YawSpeedPID), &(cali_data->YawSpeedPID), sizeof((cali_data->YawSpeedPID)));
		memcpy(&(gAppParamStruct.RotateSpeedPID), &(cali_data->RotateSpeedPID), sizeof((cali_data->RotateSpeedPID)));
		memcpy(&(gAppParamStruct.ChassisSpeedPID), &(cali_data->ChassisSpeedPID), sizeof((cali_data->ChassisSpeedPID)));
		memcpy(&(cali_data->GimbalCaliData), &gAppParamStruct.GimbalCaliData, sizeof(gAppParamStruct.GimbalCaliData));//防止calidata里面的空值影响
		//memcpy(&(cali_data->CamaraOffset), &gAppParamStruct.CamaraOffset, sizeof(gAppParamStruct.CamaraOffset));
		memcpy(&(cali_data->GyroCaliData), &gAppParamStruct.GyroCaliData, sizeof(gAppParamStruct.GyroCaliData));
		memcpy(&gAppParamStruct, cali_data, sizeof(AppParam_t));
			
		 AppParamSave();	
	 }
}
CALI_STATE_e PIDCaliProcess(AppParam_t *cali_data)
{
	if(cali_data!=NULL)
	{
		memcpy(&PIDCaliData, cali_data, sizeof(*cali_data));
		return CALI_STATE_DONE;
	}	
    return CALI_STATE_DONE;
}

CALI_STATE_e  GimbalCaliProcess()     //返回校准状态   ERROR DONE
{
	static uint32_t loopCount = 0;
	static uint32_t loopTime = 10;
	static int32_t pitchSum = 0;
	static int32_t yawSum = 0;
	
//	if(Is_Lost_Error_Set(LOST_ERROR_GIMBAL_YAW) || Is_Lost_Error_Set(LOST_ERROR_GIMBAL_PITCH))
//	{
//		return CALI_STATE_ERR;
//	}
//	else 
		if(loopCount++<loopTime)   //in cali state
	{
		pitchSum += GMPitchEncoder.raw_value;
		yawSum += GMYawEncoder.raw_value;
		return CALI_STATE_IN;
	}
	else
	{		
		GimbalCaliData.GimbalPitchOffset = pitchSum/loopTime;   //读取pitch轴陀螺仪作为偏差
	  GimbalCaliData.GimbalYawOffset = yawSum/loopTime;		//读取yaw轴陀螺仪作为偏差
		GimbalCaliData.GimbalCaliFlag = PARAM_CALI_DONE;
		pitchSum = 0;
		yawSum = 0;
		loopCount = 0;
		return CALI_STATE_DONE;
	}	
}

void SetGyroCaliData(GyroCaliStruct_t *cali_data)
{
	if(cali_data != NULL)
    {
		memcpy(&gAppParamStruct.GyroCaliData, cali_data, sizeof(*cali_data));
		AppParamSave();
	}
}  

//给此板子校准陀螺仪
CALI_STATE_e  GyroCaliProcess()     
{
//	int16_t temp[6] = {0};
	static uint16_t loopCount = 0;
	static uint16_t loopTime = 100;			//累计20次算平均值
	static int32_t gyroXSum = 0;
	static int32_t gyroYSum = 0;
	static int32_t gyroZSum = 0;
	//将gyro值清零,如此得到的才是原始值
//	GyroSavedCaliData.GyroXOffset = 0;
//	GyroSavedCaliData.GyroYOffset = 0;
//	GyroSavedCaliData.GyroZOffset = 0;	
	AppParamRealUsed.GyroCaliData.GyroXOffset = 0;
	AppParamRealUsed.GyroCaliData.GyroYOffset = 0;
	AppParamRealUsed.GyroCaliData.GyroZOffset = 0;
	
//	if(Is_Lost_Error_Set(LOST_ERROR_IMU))    //
//	{
//		return CALI_STATE_ERR;
//	}
	 if(loopCount++<loopTime)   
	{
	//	MPU6050_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
		gyroXSum += imu_data.gx;
		gyroYSum += imu_data.gy;
		gyroZSum += imu_data.gz;
		return CALI_STATE_IN;
	}
	else
	{					
		GyroCaliData.GyroXOffset = gyroXSum/loopTime;   
	  GyroCaliData.GyroYOffset = gyroYSum/loopTime;
		GyroCaliData.GyroZOffset = gyroZSum/loopTime;	
		GyroCaliData.GyroCaliFlag = PARAM_CALI_DONE;
		gyroXSum = 0;
		gyroYSum = 0;
		gyroZSum = 0;
		loopCount = 0;
		return CALI_STATE_DONE;
	}
}

void Sensor_Offset_Param_Init(AppParam_t *appParam)
{
	memcpy(&AppParamRealUsed, appParam, sizeof(AppParam_t));//复制到realused

	GMPitchEncoder.ecd_bias = AppParamRealUsed.GimbalCaliData.GimbalPitchOffset;
	GMYawEncoder.ecd_bias = AppParamRealUsed.GimbalCaliData.GimbalYawOffset;	
}


uint16_t Key_count = 0;
void CaliLoop()
{
	 CALI_STATE_e cali_result;    
	
	//大主控板陀螺仪校准
		if(IsCaliCmdFlagSet(CALI_FLAG_GYRO))  		
		{
			cali_result = GyroCaliProcess();  
			if(cali_result == CALI_STATE_ERR)
			{
				
			}
			else if(cali_result == CALI_STATE_IN)
			{
				
			}
			else if(cali_result == CALI_STATE_DONE)
			{
				SetGyroCaliData(&GyroCaliData);  			//把得到的值更新到 gAppParamStruct 里面 并将gAppParamStruct烧进flash里
				Sensor_Offset_Param_Init(&gAppParamStruct); //将apparamStruct的数据更新到GyroSavedCaliData
				ResetCaliCmdFlag(CALI_FLAG_GYRO);	
        Gimbal_Cali_Success = 1;				
			}
		}
	if(IsCaliCmdFlagSet(CALI_FLAG_GIMBAL))//云台校准
		{
			cali_result = GimbalCaliProcess();  
			if(cali_result == CALI_STATE_ERR)
			{
				ResetCaliCmdFlag(CALI_FLAG_GIMBAL);
			}
			else if(cali_result == CALI_STATE_IN)
			{
				
			}
			else if(cali_result == CALI_STATE_DONE)
			{
				SetGimbalCaliData(&GimbalCaliData);       //实际使用struct   
				Sensor_Offset_Param_Init(&gAppParamStruct);  
				ResetCaliCmdFlag(CALI_FLAG_GIMBAL);
				Gimbal_Cali_Success = 1;
			}	
		}
		else if(IsCaliCmdFlagSet(CALI_FLAG_PID))
		{
			SetPIDCaliData(&PIDCaliData);                 
			Sensor_Offset_Param_Init(&gAppParamStruct);   
			ResetCaliCmdFlag(CALI_FLAG_PID);

		}
		if(Check_Button() == 1)
		{
			Key_count++;
			//长按超过3s
			if(Key_count> 3000)
			{
				Key_count = 0;
				SetWorkState(STOP_STATE);		//停机 防止发疯
				SetCaliCmdFlag(CALI_FLAG_GYRO);//进入云台校准
			}
		}
		else
		{
			Key_count = 0;
		}
			
}

