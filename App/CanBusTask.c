#include "CanBusTask.h"
#include "main.h"
#include "pid.h"

extern PID_Regulator_t ShootMotorSpeedPID ;

Measure CM1_Measure = {0, 0, 0, 0};
Measure CM2_Measure = {0, 0, 0, 0};
Measure CM3_Measure = {0, 0, 0, 0};
Measure CM4_Measure = {0, 0, 0, 0};
Measure Turntable_Measure ={0, 0, 0, 0};

volatile Encoder GMYawEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile Encoder GMPitchEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};
volatile Encoder TurntableEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};


/*处理接收数据*/
/**
  * @brief  处理编码值,将其转换为连续的角度
  * @param  msg电机由CAN回传的信息 v编码器结构体
  * @retval void
  */
void EncoderProcess(volatile Encoder *v, CanRxMsgTypeDef *msg)
{
	int i = 0;
	int32_t temp_sum = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (msg->Data[0] << 8) | msg->Data[1];
	v->diff = v->raw_value - v->last_raw_value;
	if (v->diff < -6400) //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if (v->diff > 6400)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff - 8192;
	}
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias) * 360.0f / 8192 + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if (v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}	
	//计算速度平均值
	for (i = 0; i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum / RATE_BUF_SIZE);
}

void get_measure(Measure *mea, CanRxMsgTypeDef *msg)
{
	mea->angle = (uint16_t)(msg->Data[0] << 8 | msg->Data[1]);
	mea->speed_rpm = (int16_t)(msg->Data[2] << 8 | msg->Data[3]);
	mea->real_current = (int16_t)((msg->Data[4] << 8) | (msg->Data[5]));
	mea->Temperature = msg->Data[6];
}

void CanReceiveMsgProcess(CanRxMsgTypeDef *msg)
{
	
	switch(msg->StdId )
	{
		case CM1_ID:
		{
			get_measure(&CM1_Measure, msg);
		}break;
		case CM2_ID :
		{
			get_measure(&CM2_Measure, msg);
		}break;
		case CM3_ID:
		{
			get_measure(&CM3_Measure, msg);
		}break;
		case CM4_ID :
		{
			get_measure(&CM4_Measure, msg);
		}break;
		case GMLittlePitch_ID:
		{
			EncoderProcess(&GMPitchEncoder, msg);
		}break;
		case GMPitch_ID :
		{
			EncoderProcess(&GMPitchEncoder, msg);
		}break;
	 case TM_ID:
		{
				get_measure(&Turntable_Measure, msg);
				EncoderProcess(&TurntableEncoder, msg);

		}break;
  }
}


void CanReceiveMsgProcess_2(CanRxMsgTypeDef *msg)
{
	
	switch(msg->StdId )
	{
		case GMPitch_ID :
		{
			EncoderProcess(&GMPitchEncoder, msg);
		}break;

  }
}

/*底盘*/
void Set_CM_Speed(CAN_HandleTypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
	hcan1.pTxMsg->StdId = 0x200;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;

	hcan1.pTxMsg->Data[0] = (uint8_t)(cm1_iq >> 8);
	hcan1.pTxMsg->Data[1] = (uint8_t)cm1_iq;
	hcan1.pTxMsg->Data[2] = (uint8_t)(cm2_iq >> 8);
	hcan1.pTxMsg->Data[3] = (uint8_t)cm2_iq;
	hcan1.pTxMsg->Data[4] = (uint8_t)(cm3_iq >> 8);
	hcan1.pTxMsg->Data[5] = (uint8_t)cm3_iq;
	hcan1.pTxMsg->Data[6] = (uint8_t)(cm4_iq >> 8);
	hcan1.pTxMsg->Data[7] = (uint8_t)cm4_iq;
	
	HAL_CAN_Transmit_IT(&hcan1);
	
}
/*云台*/
//can1   pitch 6623   2006
uint16_t Set_Gimbal_Count;
void Set_Gimbal_Current(CAN_HandleTypeDef *CANx,int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq)
{
	Set_Gimbal_Count++;
	hcan1.pTxMsg->StdId = 0x1FF;
	hcan1.pTxMsg->IDE = CAN_ID_STD;
	hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	hcan1.pTxMsg->DLC = 0x08;

	hcan1.pTxMsg->Data[0] = (unsigned char)(gimbal_yaw_iq >> 8);
	hcan1.pTxMsg->Data[1] = (unsigned char)gimbal_yaw_iq;
	hcan1.pTxMsg->Data[2] = (unsigned char)(gimbal_pitch_iq >> 8);
	hcan1.pTxMsg->Data[3] = (unsigned char)gimbal_pitch_iq;
	hcan1.pTxMsg->Data[4] = (int16_t)(ShootMotorSpeedPID.output) >> 8;
	hcan1.pTxMsg->Data[5] = (unsigned char)(ShootMotorSpeedPID.output);
	hcan1.pTxMsg->Data[6] = 0x00;
	hcan1.pTxMsg->Data[7] = 0x00;
	
	HAL_CAN_Transmit_IT(&hcan1);
}
//CAN2 -->弹仓3508 机械臂 
void Set_Up_Current(CAN_HandleTypeDef *CANx, int16_t gimbal_pitch_iq)
{
	HAL_StatusTypeDef status;
	hcan2.pTxMsg->StdId = 0x200;                                           
	hcan2.pTxMsg->IDE = CAN_ID_STD;
	hcan2.pTxMsg->RTR = CAN_RTR_DATA;
	hcan2.pTxMsg->DLC = 0x08;

	hcan2.pTxMsg->Data[0] = 0x00;//(int16_t)(BulletTurntableSpeedPID.output) >> 8;
	hcan2.pTxMsg->Data[1] = 0x00;//(unsigned char)(BulletTurntableSpeedPID.output);
	hcan2.pTxMsg->Data[2] = 0x00;//(int16_t)(Wrist_SpeedPID.output) >> 8;
	hcan2.pTxMsg->Data[3] = 0x00;//(unsigned char)(Wrist_SpeedPID.output);
	hcan2.pTxMsg->Data[4] = (unsigned char)(gimbal_pitch_iq >> 8);
	hcan2.pTxMsg->Data[5] = (unsigned char)(gimbal_pitch_iq);
	hcan2.pTxMsg->Data[6] = 0x00;
	hcan2.pTxMsg->Data[7] = 0x00;
  
	//status=HAL_CAN_Transmit(&hcan2,1);
	HAL_CAN_Transmit_IT(&hcan2);
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FOV0 | CAN_IT_FMP0 );
	
}