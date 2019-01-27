#include "Nvic.h"
#include "main.h"
#include "stm32f4xx_hal.h"

void BSP_NVIC_Init(void)
{
//	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 1);
//	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
//	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 1);
//	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
//	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 1);
//	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
//	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 1);
//	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
//	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 1);
//	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);


	HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);//串口1
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);//串口2
	HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART3_IRQn);//串口3
	HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(USART6_IRQn);//串口6
	
		
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);//MPU外部中断

  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
	HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);//CAN接收中断
	
	HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
	HAL_NVIC_SetPriority(CAN2_TX_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(CAN2_TX_IRQn);//CAN接收中断

	
		
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);//定时器

	
}
