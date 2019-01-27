#include "main.h"
#include "data.h"



CIRCLE_BUFF_t Que_JudgeFrame = {0, 0, {0}};
CIRCLE_BUFF_t Que_USART2_Tx  = {0, 0, {0}};
CAN_CIRCLE_BUFF_t Que_CAN1_Tx = {0};
CAN_CIRCLE_BUFF_t Que_CAN2_Tx = {0};
/**
* @brief 从队列中取出一个值
* @param CIRCLE_BUFF_t* buffer  队列指针
* @param unsigned char* _buf    接收数据的地址
* @param uint8_t num			要取出的个数
* @retval int 					失败	-1
								成功	所读取到的值
*/ 
int bufferPop(CIRCLE_BUFF_t* buffer, unsigned char* _buf)
{
    if(buffer->head_pos==buffer->tail_pos)        //如果头尾接触表示缓冲区为空
	{
        *_buf=0xFF;
		return -1;
	}
	else
    {
        *_buf = buffer->circle_buffer[buffer->head_pos];    //如果缓冲区非空则取头节点值并偏移头节点
        if(++buffer->head_pos >= BUFFER_MAX)
            buffer->head_pos=0;
		return *_buf;
    }
}

/**
* @brief 从队列中取出多个值
* @param CIRCLE_BUFF_t* buffer  队列指针
* @param unsigned char* _buf    接收数据的地址
* @param uint8_t num			要取出的个数
* @retval unsigned char 		最后取出的一个数
*/ 
unsigned char buffer_multiPop(CIRCLE_BUFF_t* buffer, unsigned char* rx_buf, uint16_t num)
{
    for(int i = 0; i<num; i++)
	{
		if(bufferPop(buffer, &rx_buf[i]) == -1)
			break;
	}
	return rx_buf[num-1];
}

/**
* @brief 写入数据到队列中
* @param CIRCLE_BUFF_t* buffer  队列指针
* @param const unsigned char _buf    要写入的数据的地址
* @retval void
*/ 
void bufferPush(CIRCLE_BUFF_t* buffer, const unsigned char _buf)
{   
    buffer->circle_buffer[buffer->tail_pos]=_buf; //从尾部追加
    if(++buffer->tail_pos>=BUFFER_MAX)           //尾节点偏移
	{
        buffer->tail_pos=0;                      //大于数组最大长度 制零 形成环形队列
    }
	if(buffer->tail_pos==buffer->head_pos)    //如果尾部节点追到头部节点 则修改头节点偏移位置丢弃早期数据
    {
		if(++buffer->head_pos>=BUFFER_MAX)
        {
			buffer->head_pos=0;
		}
	}
}

/**
* @brief 读取队列数据长度
* @param CIRCLE_BUFF_t* buffer  队列指针
* @retval uint8_t				长度	
*/ 
uint16_t bufferlen(CIRCLE_BUFF_t* buffer)
{
	int len = buffer->tail_pos-buffer->head_pos;
	if(len >=0)
		return len;
	else
		return len + BUFFER_MAX;
}



//------------------------------------CAN类型队列-------------------------------------------
/**
* @brief 从队列中取出一个值
* @param CIRCLE_BUFF_t* buffer  队列指针
* @param unsigned char* _buf    接收数据的地址
* @param uint8_t num			要取出的个数
* @retval int 					失败	-1
								成功	所读取到的值
*/ 
int CAN_bufferPop(CAN_CIRCLE_BUFF_t* buffer, CanTxMsgTypeDef* _buf)
{
    if(buffer->head_pos==buffer->tail_pos)        //如果头尾接触表示缓冲区为空
	{
		CanTxMsgTypeDef que_temp = {0, 0, 0};
        *_buf= que_temp;
		return -1;
	}
	else
    {
        *_buf = buffer->circle_buffer[buffer->head_pos];    //如果缓冲区非空则取头节点值并偏移头节点
        if(++buffer->head_pos >= CAN_BUFFER_MAX)
            buffer->head_pos=0;
		return 0;
    }
}

/**
* @brief 从队列中取出多个值
* @param CAN_CIRCLE_BUFF_t* buffer  队列指针
* @param unsigned char* _buf    接收数据的地址
* @param uint8_t num			要取出的个数
* @retval unsigned char 		最后取出的一个数
*/ 
int CAN_buffer_multiPop(CAN_CIRCLE_BUFF_t* buffer, CanTxMsgTypeDef* rx_buf, uint16_t num)
{
    for(int i = 0; i<num; i++)
	{
		if(CAN_bufferPop(buffer, &rx_buf[i]) == -1)
		{
			return -1 ;
		}
	}
	return 0;
}

/**
* @brief 写入数据到队列中
* @param CAN_CIRCLE_BUFF_t* buffer  队列指针
* @param const unsigned char _buf    要写入的数据的地址
* @retval void
*/ 
void CAN_bufferPush(CAN_CIRCLE_BUFF_t* buffer, CanTxMsgTypeDef _buf)
{   
    buffer->circle_buffer[buffer->tail_pos]=_buf; //从尾部追加
    if(++buffer->tail_pos>=CAN_BUFFER_MAX)           //尾节点偏移
	{
        buffer->tail_pos=0;                      //大于数组最大长度 制零 形成环形队列
    }
	if(buffer->tail_pos==buffer->head_pos)    //如果尾部节点追到头部节点 则修改头节点偏移位置丢弃早期数据
    {
		if(++buffer->head_pos>=CAN_BUFFER_MAX)
        {
			buffer->head_pos=0;
		}
	}
}

/**
* @brief 读取队列数据长度
* @param CAN_CIRCLE_BUFF_t* buffer  队列指针
* @retval uint8_t				长度	
*/ 
uint16_t CAN_bufferlen(CAN_CIRCLE_BUFF_t* buffer)
{
	int len = buffer->tail_pos-buffer->head_pos;
	if(len >=0)
		return len;
	else
		return len + CAN_BUFFER_MAX;
}

