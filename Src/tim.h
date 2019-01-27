#ifndef _TIM_H
#define _TIM_H
#include "stdint.h"
////#include "stm32f4xx_hal_tim.h"

////extern TIM_HandleTypeDef htim2;
////extern TIM_HandleTypeDef htim3;
////extern TIM_HandleTypeDef htim5;
////extern TIM_HandleTypeDef htim6;
////extern TIM_HandleTypeDef htim12;


void MX_TIM2_Init(void);
void MX_TIM3_Init(void);
void MX_TIM6_Init(void);
void MX_TIM5_Init(void);
void MX_TIM12_Init(void);
void TIM6_Start(void);
uint32_t Get_Time_Micros(void);

#endif
