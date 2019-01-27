/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "BSP_Init.h"
#include "stm32f4xx_it.h"
#include "Commondefine.h"
#include "ControlTask.h"
#include "imu.h"
#include "test_imu.h"
#include "CanBusTask.h"
#include "Driver_Protocol.h"
#include "Driver_Minifold.h"
#include "stdio.h"
#include "usart.h"
#include "imu_out.h"
#include "DM9015.h"
#include "Remote.h"


void MX_FREERTOS_Init(void);
uint16_t frame_1000count=0;
uint16_t frame_500count=0;
uint16_t frame_100count=0;

uint16_t speed=1000;
char temp[16]={0};
uint16_t  send_usart=0;
int32_t y_angle=0,y_speed=10; 

int main(void)
{

 	HAL_Init();							  	   	//HAL相关初始化
	SystemClock_Config();						//时钟配置
  ControlLoopTaskInit();
  RemoteTaskInit();
	BSP_Init();			 	            	//外设配置
	
	MPU6500_Init();
	MX_FREERTOS_Init();	 						//FreeRTOS空闲任务
  while (1)
  {
//		TIM5->CCR3=speed;
//				TIM5->CCR2=speed;

		if(frame_1000Hz)//
		{
			frame_1000count++;
			frame_1000Hz = false;
			Control_Task();
      Protocol();					//上位机回传  
//			send_usart =pitch_angle_out;
//			sprintf (temp,"%d\n",send_usart);
//			HAL_UART_Transmit(&huart2,temp ,strlen (temp),10);
		}
		
			if(frame_500Hz)
		{
			    // DM_SendPower((int16_t)RC_CtrlData.rc.ch3);

			frame_500count++;
			frame_500Hz = false;		
			
		}
		
		if(frame_100Hz)
		{
		//	Send_MiniPC();			//miniPC回传
			frame_100count++;
			frame_100Hz = false;
			

		}
  }
    
}


/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
