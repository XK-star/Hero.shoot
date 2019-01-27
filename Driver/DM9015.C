#include "DM9015.h"
#include "data.h"
#include "stdio.h"
#include "CanbusTask.h"

CIRCLE_BUFF_t Que_DMfold = {0, 0, {0}};
CIRCLE_BUFF_t Que_DM_Tx  = {0, 0, {0}};

//volatile Encoder GMYawEncoder = {0, 0, 0, 0, 0, 0, 0, 0, 0};
static int8_t frame_temp_DM[30];
	 int8_t  DM_DMA_BUFF[200] = {0}; //DMA取值缓冲区


/**接收驱动回复(8byte) 格式 1   帧头       0x3E
                            2   命令字节   0xA0 0xA2 0xA3 0xA4
                            3   ID         0x01~0X04
                            4   数据长度   0x02
                            5   校验位     1-3位校验和
                           6.7  编码器数据 0~4095
                            8   校验位     5-6位校验和               **/

void DMfold_Res_Task()
{
	int16_t sum=0;
	static int16_t Ecoder;
		while(bufferlen(&Que_DMfold) != 0)
	{
		//寻找帧头AD
		if(bufferPop(&Que_DMfold,frame_temp_DM) == 0x3E)  
		{
			buffer_multiPop(&Que_DMfold, &frame_temp_DM[1],6);
			if(frame_temp_DM[2]==0x01)
			{
			sum=frame_temp_DM[5]+frame_temp_DM[6];
			if(bufferPop(&Que_DMfold,&frame_temp_DM[7]) ==sum&0xff )	
			{
				GMYawEncoder .last_raw_value =GMYawEncoder.raw_value;
				GMYawEncoder.raw_value=((int16_t)(frame_temp_DM[6] << 8 | frame_temp_DM[5]));
				DM_EncoderProcess(&GMYawEncoder);
			}
	  	}
		}
	}		
}

void DM_EncoderProcess(Encoder *v)//没写好
{
	int i = 0;
	int32_t temp_sum = 0;
	if(v->raw_value<0)
	{
		v->raw_value+=4095;
	}
	v->diff = v->raw_value - v->last_raw_value;
	if (v->diff < -3200)//两次的编码器角度偏差太大
	{
			v->round_cnt++;
			v->ecd_raw_rate = v->diff + 4095;
		}
	else if (v->diff > 3200)
	{
			v->round_cnt--;
			v->ecd_raw_rate = v->diff - 4095;
		}
	else
	{
			v->ecd_raw_rate = v->diff;
		}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 4095;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias) * 360.0f / 4095 + v->round_cnt * 360;
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


/*************开环控制：发送输出到电机的堵转扭矩*****************/ 
/******************int16_t POWER -850~+850***********************/
uint16_t SendP_count=0;

void DM_SendPower(int16_t power)
{
	SendP_count++;
	int8_t Data[50];
  int16_t sum=0;	

	Data[0] = 0x3E;//头字节
	Data[1] = 0xA0;//命令字节 扭转比例
	Data[2] = 0x01;//ID
	Data[3] = 0x02;//数据长度
	Data[4] = 0xE1;//1-4字节校验和
	
	Data[5] = (int8_t)(power);
	Data[6] = (int8_t)(power>>8);//数据
	
	for(uint8_t i=5;i<7;i++)		//求和
	{
		sum += Data[i];
	}
	Data[7] = sum&0xFF;//写入校验位

	for(int i = 0;i<8;i++)
	{
		bufferPush(&Que_DM_Tx, Data[i]);//数据写入队列
	}
	
	//


	uint8_t Bufferl_len = bufferlen(&Que_DM_Tx);//队列数据长度
	buffer_multiPop(&Que_DM_Tx, DM_DMA_BUFF, Bufferl_len);//从队列中取出所有数据
	HAL_UART_Transmit(&huart2, DM_DMA_BUFF, Bufferl_len,10);//启用DMA发送  串口待定huartx

}

/**************速度闭环控制：发送转动角速度************************************/ 
/**************int32_t Angular Speed 实际速度比0.01dps/LSB*****************/
void DM_SendAngularSpeed(int32_t angularspeed)
{
	int8_t Data[50];
  int16_t sum=0;	

	Data[0] = 0x3E;//头字节
	Data[1] = 0xA2;//命令字节 角速度
	Data[2] = 0x01;//ID
	Data[3] = 0x04;//数据长度
	Data[4] = 0xE5;//1-4字节校验和
	
	Data[5] = (int8_t)(angularspeed);
	Data[6] = (int8_t)(angularspeed>>8);
	Data[7] = (int8_t)(angularspeed>>16);
	Data[8] = (int8_t)(angularspeed>>24);//数据

	for(uint8_t i=5;i<9;i++)		//求和
	{
		sum += Data[i];
	}
	Data[9] = sum&0xFF;//写入校验位

	for(int i = 0;i<10;i++)
	{
		bufferPush(&Que_DM_Tx, Data[i]);//数据写入队列
	}
	
	//

	uint8_t Bufferl_len = bufferlen(&Que_DM_Tx);//队列数据长度
	buffer_multiPop(&Que_DM_Tx, DM_DMA_BUFF, Bufferl_len);//从队列中取出所有数据
	HAL_UART_Transmit(&huart2, DM_DMA_BUFF, Bufferl_len,10);//启用DMA发送  串口待定huartx

}

/***************位置闭环控制1：发送转动角度************************************/ 
/**************int32_t Angle 实际角度比0.01 degree/LSB*****************/
void DM_SendAngle(int64_t angle)
{
	int8_t Data[50];
  int16_t sum=0;	

	Data[0] = 0x3E;//头字节
	Data[1] = 0xA3;//命令字节 角度
	Data[2] = 0x01;//ID
	Data[3] = 0x08;//数据长度
	Data[4] = 0xEA;//1-4字节校验和
	
	Data[5] = (int8_t)(angle);
	Data[6] = (int8_t)(angle>>8);
	Data[7] = (int8_t)(angle>>16);
	Data[8] = (int8_t)(angle>>24);
	Data[9] = (int8_t)(angle>>32);
	Data[10] = (int8_t)(angle>>40);
	Data[11] = (int8_t)(angle>>48);
	Data[12] = (int8_t)(angle>>56);//数据
	
	for(uint8_t i=5;i<13;i++)		//求和
	{
		sum += Data[i];
	}
	Data[13] = sum&0xFF;//写入校验位

	for(int i = 0;i<14;i++)
	{
		bufferPush(&Que_DM_Tx, Data[i]);//数据写入队列
	}
	
	//


	uint8_t Bufferl_len = bufferlen(&Que_DM_Tx);//队列数据长度
	buffer_multiPop(&Que_DM_Tx, DM_DMA_BUFF, Bufferl_len);//从队列中取出所有数据
	HAL_UART_Transmit(&huart2, DM_DMA_BUFF, Bufferl_len,10);//启用DMA发送  串口待定huartx

}	

/*****************位置闭环控制2：发送角度和转动角速度*****************/
/**************int32_t Angle 实际角度比0.01 degree/LSB*****************/
/**************int32_t Angular Speed 实际速度比0.01dps/LSB*****************/

void DM_SendPosition(int64_t angle,int32_t angularspeed)
{
	int8_t Data[50];
  int16_t sum=0;	

	Data[0] = 0x3E;//头字节
	Data[1] = 0xA4;//命令字节 角速度和角度
	Data[2] = 0x01;//ID
	Data[3] = 0x0C;//数据长度
	Data[4] = 0xEF;//1-4字节校验和
	
	Data[5] = (int8_t)(angle);;
	Data[6] = (int8_t)(angle>>8);
	Data[7] = (int8_t)(angle>>16);
	Data[8] = (int8_t)(angle>>24);
	Data[9] = (int8_t)(angle>>32);
	Data[10] = (int8_t)(angle>>40);
	Data[11] = (int8_t)(angle>>48);
	Data[12] = (int8_t)(angle>>56);
	Data[13] = (int8_t)(angularspeed);
	Data[14] = (int8_t)(angularspeed>>8);
	Data[15] = (int8_t)(angularspeed>>16);
	Data[16] = (int8_t)(angularspeed>>24);//数据

	for(uint8_t i=5;i<17;i++)		//求和
	{
		sum += Data[i];
	}
	Data[17] = sum&0xFF;//写入校验位

	for(int i = 0;i<18;i++)
	{
		bufferPush(&Que_DM_Tx, Data[i]);//数据写入队列
	}
	
	//


	uint8_t Bufferl_len = bufferlen(&Que_DM_Tx);//队列数据长度
	buffer_multiPop(&Que_DM_Tx, DM_DMA_BUFF, Bufferl_len);//从队列中取出所有数据
	HAL_UART_Transmit(&huart2, DM_DMA_BUFF, Bufferl_len,10);//启用DMA发送  串口待定huartx

}