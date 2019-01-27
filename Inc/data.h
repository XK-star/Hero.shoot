#ifndef __DATA_H
#define __DATA_H

#define BUFFER_MAX  500      //缓冲区大小
#include "main.h"
#include "stm32f4xx_hal.h"
typedef struct _circle_buffer{
    uint16_t head_pos;             //缓冲区头部位置
    uint16_t tail_pos;             //缓冲区尾部位置   
    unsigned char circle_buffer[BUFFER_MAX];    //缓冲区数组 
}CIRCLE_BUFF_t;

void bufferPush(CIRCLE_BUFF_t* buffer, const unsigned char _buf);
int bufferPop(CIRCLE_BUFF_t* buffer, unsigned char* _buf);
uint8_t buffer_multiPop(CIRCLE_BUFF_t* buffer, unsigned char* rx_buf, uint16_t num);
uint16_t bufferlen(CIRCLE_BUFF_t* buffer);


extern CIRCLE_BUFF_t Que_JudgeFrame;
extern CIRCLE_BUFF_t Que_USART2_Tx;

//------------------------CAN结构体类型队列-------------------------
#define CAN_BUFFER_MAX  20      //缓冲区大小

typedef struct CAN_circle_buffer{
    uint16_t head_pos;             //缓冲区头部位置
    uint16_t tail_pos;             //缓冲区尾部位置   
    CanTxMsgTypeDef circle_buffer[CAN_BUFFER_MAX];    //缓冲区数组 
}CAN_CIRCLE_BUFF_t;

void CAN_bufferPush(CAN_CIRCLE_BUFF_t* buffer, CanTxMsgTypeDef _buf);
int CAN_bufferPop(CAN_CIRCLE_BUFF_t* buffer, CanTxMsgTypeDef* _buf);
int CAN_buffer_multiPop(CAN_CIRCLE_BUFF_t* buffer, CanTxMsgTypeDef* rx_buf, uint16_t num);
uint16_t CAN_bufferlen(CAN_CIRCLE_BUFF_t* buffer);
extern CAN_CIRCLE_BUFF_t Que_CAN1_Tx;
extern CAN_CIRCLE_BUFF_t Que_CAN2_Tx;
extern CIRCLE_BUFF_t Que_Mainifold;
extern CIRCLE_BUFF_t Que_Protocol;
extern CIRCLE_BUFF_t Que_DMfold;
#endif



