#ifndef _SUPERVISETASK_H
#define _SUPERVISETASK_H
#include "stdint.h"

extern uint16_t DBUSFrameRate;
extern uint16_t DBUSFrameCounter ;

extern uint16_t ChassisFrameRate[4] ;
extern uint16_t ChassisFrameCounter[4] ;

extern uint16_t PitchFrameRate;
extern uint16_t PitchFrameCounter ;
	
extern uint16_t YawFrameRate ;
extern uint16_t YawFrameCounter ;

extern uint16_t TurntableFrameRate ;
extern uint16_t TurntableFrameCounter ;

extern uint16_t IMUFrameRate ;
extern uint16_t IMUFrameCounter ;

extern uint16_t UART2FrameRate ;
extern uint16_t UART2FrameCounter ;

extern uint16_t CAN_Send1_FrameRate ;
extern uint16_t CAN_Send1_FrameCounter ;

extern uint16_t CAN_Send2_FrameRate ;
extern uint16_t CAN_Send2_FrameCounter ;

extern uint16_t CAN_Res_FrameRate ;
extern uint16_t CAN_Res_FrameCounter ;

extern uint16_t JudgeFrameRate ;
extern uint16_t JudgeFrameCounter ;

extern uint16_t MiniPCFrameRate ;
extern uint16_t MiniPCFrameCounter ;

extern uint16_t IMU_Out_FrameRate;
extern uint16_t IMU_Out_FrameCounter ;

void Task_Monitor(void);

#endif
