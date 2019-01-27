#ifndef IOTASK_H
#define IOTASK_H
#include "main.h" 
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"


#define PARAM_SAVED_START_ADDRESS 								ADDR_FLASH_SECTOR_11


#define PARAM_SAVED_FLAG                            0x5A   
#define PARAM_CALI_DONE                             0x5A 		
#define PARAM_CALI_NONE                             0x00

#define CALI_FLAG_GYRO                    ((uint32_t)1<<2)	//4
#define CALI_START_FLAG_ACC                   ((uint32_t)1<<3)
#define CALI_START_FLAG_MAG                   ((uint32_t)1<<4)
#define CALI_END_FLAG_MAG                     ((uint32_t)1<<5)
#define CALI_FLAG_GIMBAL                  	  ((uint32_t)1<<7)//80
#define CALI_FLAG_PID         				  ((uint32_t)1<<8)//256
#define CALI_FLAG_PITCH_SPEED_PID             ((uint32_t)1<<9)
#define CALI_FLAG_YAW_POSITION_PID            ((uint32_t)1<<10)
#define CALI_FLAG_YAW_SPEED_PID               ((uint32_t)1<<11)
#define CALI_FLAG_CAMERA         			  ((uint32_t)1<<12)
#define CALI_FLAG_LITTLEGYRO         		  ((uint32_t)1<<13)




typedef enum
{
    CALI_STATE_ERR,
    CALI_STATE_IN,
    CALI_STATE_DONE,
}CALI_STATE_e;

typedef __packed struct
{
    int16_t     GimbalYawOffset;
    int16_t     GimbalPitchOffset;
    uint8_t     GimbalCaliFlag;
}GimbalCaliStruct_t;

typedef __packed struct
{
    int16_t     GyroXOffset;
    int16_t     GyroYOffset;
    int16_t     GyroZOffset;
    uint8_t     GyroCaliFlag;
}GyroCaliStruct_t;

typedef __packed struct
{
    int16_t     AccXOffset;
    int16_t     AccYOffset;
    int16_t     AccZOffset; 
    float       AccXScale;
    float       AccYScale;
    float       AccZScale;
    uint8_t     AccCaliFlag;
}AccCaliStruct_t;

typedef __packed struct
{
    int16_t     MagXOffset;
    int16_t     MagYOffset;
    int16_t     MagZOffset;
    float       MagXScale;
    float       MagYScale;
    float       MagZScale;    
    uint8_t     MagCaliFlag;
}MagCaliStruct_t;

typedef __packed struct
{
	int8_t pid_type;		// position PID
	int8_t motor_type;   //motor type ie: pitch yaw 201 202 203 204	
	int16_t kp_offset;
	int16_t ki_offset;
	int16_t kd_offset;
}PIDParamStruct_t;

typedef __packed struct 
{
    uint8_t     ParamSavedFlag;    				//header 
    uint32_t    FirmwareVersion;    			//version
    GimbalCaliStruct_t GimbalCaliData;    //gimbal pitch yaw encoder offset
    GyroCaliStruct_t   GyroCaliData;      //gyro offset data
    AccCaliStruct_t    AccCaliData;    		//ACC offset data
    MagCaliStruct_t    MagCaliData;				//Mag offset data
	PIDParamStruct_t   PitchPositionPID;
	PIDParamStruct_t   PitchSpeedPID;
	PIDParamStruct_t   YawPositionPID;
	PIDParamStruct_t   YawSpeedPID;
	PIDParamStruct_t   RotateSpeedPID;
	PIDParamStruct_t   ChassisSpeedPID;
	PIDParamStruct_t	TurntableSpeedPID;
	PIDParamStruct_t	TurntablePositionPID;	
	PIDParamStruct_t	FrictionWheelMaxSpeed;
	int16_t		TurntableSpeed_target;
	//CameraOffset_t		CamaraOffset;	
}AppParam_t;
//上传数据的类型

extern AppParam_t gAppParamStruct;					//配置信息,这里保存着最新的校准值，并且与Flash中的内容同步
extern AppParam_t AppParamRealUsed;        //实际使用的

//校正云台/陀螺仪/加速计
void SetCaliCmdFlag(uint32_t flag);

CALI_STATE_e PIDCaliProcess(AppParam_t *cali_data);			//上位机数据保存到中间变量


void CaliLoop(void);
void AppParamInit(void);
void Sensor_Offset_Param_Init(AppParam_t *appParam);


#endif