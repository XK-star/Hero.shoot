/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"
#include "main.h"
#include "Remote.h"
#include "Commondefine.h"
#include "can.h"
#include "CanBusTask.h"
#include "imu_out.h"
#include "data.h"
#include "Driver_Minifold.h"
#include "Driver_Protocol.h"
#include "DM9015.h"
#include "SuperviseTask.h"

//#define MiNi_PC_Test//视觉串口2收发//9015测试

//#include "tim.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
#define FRAME_COUNT   1000

#define COUNT_1000HZ   1        // Number of 1000 Hz frames for 500 Hz Loop
#define COUNT_500HZ    2      // Number of 1000 Hz frames for  500 Hz Loop
#define COUNT_100HZ   10        // Number of 1000 Hz frames for 100 Hz Loop
#define COUNT_50HZ    20        // Number of 1000 Hz frames for  50 Hz Loop
#define COUNT_10HZ    100       // Number of 1000 Hz frames for  10 Hz Loop
#define COUNT_5HZ     200       // Number of 1000 Hz frames for   5 Hz Loop
#define COUNT_1HZ     1000      // Number of 1000 Hz frames for   1 Hz Loop

uint8_t frame_1000Hz = false;
uint8_t frame_500Hz  = false;
uint8_t frame_100Hz  = false;
uint8_t frame_50Hz   = false;
uint8_t frame_10Hz   = false;

uint32_t FrameCounter = 0;

/* USER CODE BEGIN 0 */
//串口1 DMA缓存 遥控器
#define	BSP_USART1_DMA_RX_BUF_LEN               30u 
#define	RC_FRAME_LENGTH                         18u
#define UART1_RXBUFF_SIZE 100
uint8_t UART1_RXBUFF[UART1_RXBUFF_SIZE];
//串口2 Minifold
#define UART2_RXBUFF_SIZE	80
uint8_t UART2_RXBUFF[UART2_RXBUFF_SIZE];//不可自接
//串口3 上位机
uint8_t UART3_RXBUFF[UART3_RXBUFF_SIZE];

//串口6 DMA缓存 外置mpu
#define UART6_RX_BUF_LEN		                    30u
#define IMU_FRAME_LENGTH		22u
uint8_t UART6_RXBUFF[UART6_RX_BUF_LEN];


/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;



extern TIM_HandleTypeDef htim6;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
//void DMA2_Stream2_IRQHandler(void)
//{
//  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

//  /* USER CODE END DMA2_Stream2_IRQn 0 */
//  HAL_DMA_IRQHandler(&hdma_usart1_rx);
//  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

//  /* USER CODE END DMA2_Stream2_IRQn 1 */
//}

///* USER CODE BEGIN 1 */
/**
* @brief This function handles DMA2 stream1 global interrupt.
*/
void DMA2_Stream1_IRQHandler(void)
{
 
  HAL_DMA_IRQHandler(&hdma_usart6_rx);

}

void DMA1_Stream4_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_usart3_rx);

}

void DMA1_Stream5_IRQHandler(void)
{
	
  HAL_DMA_IRQHandler(&hdma_usart2_rx);

}
uint16_t U2_tx;
void DMA1_Stream6_IRQHandler(void)
{
	U2_tx++;
  HAL_DMA_IRQHandler(&hdma_usart2_tx);

}



/**
* @brief This function handles DMA2 stream2 global interrupt.
*/
void DMA2_Stream2_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_usart1_rx);

}

extern uint16_t CAN_Send1_FrameCounter;
void CAN1_TX_IRQHandler(void)
{

  HAL_CAN_IRQHandler(&hcan1);
	CAN_Send1_FrameCounter++;
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FOV0 | CAN_IT_FMP0);

}
void CAN1_RX0_IRQHandler(void)
{
	
	HAL_CAN_IRQHandler(&hcan1);
	
	
}
extern uint16_t CAN_Send2_FrameCounter;
void CAN2_TX_IRQHandler(void)
{
  CAN_Send2_FrameCounter++;
  HAL_CAN_IRQHandler(&hcan2);
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FOV0 | CAN_IT_FMP0);


}
void CAN2_RX0_IRQHandler(void)
{
	
	HAL_CAN_IRQHandler(&hcan2);
	
}

extern uint16_t CAN_Res_FrameCounter;
uint16_t can1_res=0;
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	
	if(hcan == &hcan1)
	{	
		can1_res++;
		CanReceiveMsgProcess(hcan1.pRxMsg);
		HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);
	}
	else if(hcan == &hcan2)
	{
		CAN_Res_FrameCounter++;
		CanReceiveMsgProcess_2(hcan2.pRxMsg);
		HAL_CAN_Receive_IT(&hcan2, CAN_FIFO0); //重启FIFO0消息挂号中断
	}
	
}
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan);

uint16_t IT_Uart1;
uint16_t IT_Uart2;
uint16_t IT_Uart3;
uint16_t IT_Uart6;
void USART1_IRQHandler(void)
{
	IT_Uart1++;
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_LBD);
	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);	
		HAL_UART_AbortReceive(&huart1);//关闭dma
		
		uint16_t RX1_Length = UART1_RXBUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);//计算数据长度
		if(RX1_Length == RC_FRAME_LENGTH)
		{
			RemoteDataPrcess(UART1_RXBUFF);
			DBUSFrameCounter++;
		}
	}
	HAL_UART_Receive_DMA(&huart1,UART1_RXBUFF,UART1_RXBUFF_SIZE);//重启DMA
}

#ifdef MiNi_PC_Test
//MIniPC
void USART2_IRQHandler(void)
{
	__HAL_UART_DISABLE_IT(&huart2, UART_IT_ERR);
	__HAL_UART_DISABLE_IT(&huart2, UART_IT_LBD);

	if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET)
	{
				IT_Uart2++;
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);											
		HAL_UART_AbortReceive(&huart2);
		uint16_t RX2_Length = UART2_RXBUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);//计算数据长度
		if(RX2_Length < UART2_RXBUFF_SIZE)
		{
			for(int i = 0; i<RX2_Length; i++)
			{
				//				bufferPush(&Que_Mainifold, UART2_RXBUFF[i]);	//数据压入队列/
				bufferPush(&Que_DMfold, UART2_RXBUFF[i]);	//数据压入队列
			}
		}
		HAL_UART_Receive_DMA(&huart2,UART2_RXBUFF,UART2_RXBUFF_SIZE);//重启DMA	
	}
	DMfold_Res_Task();
	//Mainifold_Res_Task();
}
#else
//LabView
void USART2_IRQHandler(void)
{
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_ERR);
	__HAL_UART_DISABLE_IT(&huart1, UART_IT_LBD);

	if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE) != RESET)
	{
				IT_Uart2++;
		__HAL_UART_CLEAR_IDLEFLAG(&huart2);											
		HAL_UART_AbortReceive(&huart2);
		uint16_t RX2_Length = UART2_RXBUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);//计算数据长度
		if(RX2_Length < UART2_RXBUFF_SIZE)
		{
			for(int i = 0; i<RX2_Length; i++)
			{
							bufferPush(&Que_Protocol, UART2_RXBUFF[i]);	//数据压入队列
			}
		}
		HAL_UART_Receive_DMA(&huart2,UART2_RXBUFF,UART2_RXBUFF_SIZE);//重启DMA	
	}
}
#endif

void USART3_IRQHandler(void)
{
	 IT_Uart3++;
	__HAL_UART_DISABLE_IT(&huart3, UART_IT_ERR);
	__HAL_UART_DISABLE_IT(&huart3, UART_IT_LBD);
	if(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);	
		HAL_UART_AbortReceive(&huart3);//关闭dma		
		uint16_t RX_Length = UART3_RXBUFF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);//计算数据长度
			
		//写入队列
		for(int i = 0;i<RX_Length;i++)
		{
				bufferPush(&Que_Mainifold, UART6_RXBUFF[i]);	//数据压入队列
		}
	}
	HAL_UART_Receive_DMA(&huart3,UART3_RXBUFF,UART3_RXBUFF_SIZE);//重启DMA
	Mainifold_Res_Task();

}


extern uint16_t IMU_Out_FrameCounter;

void USART6_IRQHandler(void)
{
   IT_Uart6++;
	__HAL_UART_DISABLE_IT(&huart6, UART_IT_ERR);
	__HAL_UART_DISABLE_IT(&huart6, UART_IT_LBD);
	if(__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);	
		HAL_UART_AbortReceive(&huart6);//关闭dma
		
		uint16_t RX6_Length = UART6_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);//计算数据长度
		if(RX6_Length == IMU_FRAME_LENGTH)
		{
			IMU_task(UART6_RXBUFF);
			IMU_Out_FrameCounter ++;
		}
	}
	HAL_UART_Receive_DMA(&huart6,UART6_RXBUFF,UART6_RX_BUF_LEN);//重启DMA
}


void TIM6_DAC_IRQHandler(void)
{

  HAL_TIM_IRQHandler(&htim6);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim6)
	{
		FrameCounter++;
		if (FrameCounter > FRAME_COUNT)
			FrameCounter = 1;
		///////////////////////////

		if(FrameCounter%COUNT_1000HZ == 0)
		{
			frame_1000Hz = true;
		}
		if(FrameCounter%COUNT_100HZ == 0)
		{
			frame_100Hz = true;
		}
		if(FrameCounter%COUNT_500HZ == 0)
		{
			frame_500Hz = true;
		}
		if(FrameCounter%COUNT_10HZ == 0)
		{
			frame_10Hz = true;
		}
	}
}

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
