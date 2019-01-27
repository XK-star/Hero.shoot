#ifndef  __CANBUSTASK_H
#define  __CANBUSTASK_H

#include "main.h"
#include "can.h"
#include "cmsis_os.h"

//can1
#define CM1_ID           0x201
#define CM2_ID           0x202 
#define CM3_ID           0x203
#define CM4_ID           0x204
#define GMLittlePitch_ID          0x205
#define GMPitch_ID      0x206

#define TM_ID           0x207




#define RATE_BUF_SIZE 6
typedef struct{
	int16_t	 	speed_rpm;          //
	int16_t  	real_current;       //
	int16_t  	Temperature;        //
	int16_t 	angle;              //
}Measure;

typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                       //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t temp_count;                   //计数用
	uint8_t buf_count;								//滤波更新buf用
	int32_t ecd_bias;											//初始编码器值	
	int32_t ecd_raw_rate;									//通过编码器计算得到的速度原始值
	int32_t rate_buf[RATE_BUF_SIZE];	
	int32_t round_cnt;										//圈数
	int32_t filter_rate;											//速度
	float ecd_angle;											//角度
}Encoder;


extern Measure CM1_Measure;
extern Measure CM2_Measure;
extern Measure CM3_Measure;
extern Measure CM4_Measure;
extern Measure Turntable_Measure ;

extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder TurntableEncoder;

void CanReceiveMsgProcess(CanRxMsgTypeDef * msg);			//接收到CAN信号 信息处理
void CanReceiveMsgProcess_2(CanRxMsgTypeDef *msg);
void Set_Gimbal_Current(CAN_HandleTypeDef *CANx,int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq);
void Set_Up_Current(CAN_HandleTypeDef *CANx, int16_t gimbal_pitch_iq);
void Set_CM_Speed(CAN_HandleTypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);


//9015
void DM_EncoderProcess(Encoder *v);

#endif

