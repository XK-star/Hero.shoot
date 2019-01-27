#ifndef  __DRIVER_GIMBAL_H
#define  __DRIVER_GIMBAL_H

#define STICK_TO_PITCH_ANGLE_INC_FACT       0.004f		//拨杆到云台灵敏度
#define STICK_TO_YAW_ANGLE_INC_FACT         0.004f
#define MOUSE_TO_PITCH_ANGLE_INC_FACT 		0.1f		//鼠标到云台灵敏度
#define MOUSE_TO_YAW_ANGLE_INC_FACT 		0.1f

#define PITCH_MAX 17.0f
#define YAW_MAX 50 	

#define RM66TODM90 1

//角度类型枚举
typedef enum
{
  AngleMode_REL,          //相对角度
  AngleMode_ABS,           //绝对角度
	AngleMode_ECD            //编码器角度
}AngleMode_Enum;					//没有定义参数

typedef struct
{
    float Pitch;
    float Yaw;
}Gimbal_Ref_t;

typedef __packed struct
{
	float yaw;
	float pitch;
}CameraOffset_t;

void GMControlLoop(void);
static void Ref_UpdataFromRCStick(void);

extern Gimbal_Ref_t GimbalRef;

#endif
