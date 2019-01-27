
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#define GREEN_LED_Pin 		GPIO_PIN_14
#define GREEN_LED_GPIO_Port GPIOF
#define RED_LED_Pin 		GPIO_PIN_11
#define RED_LED_GPIO_Port 	GPIOE
#define LASER_Pin 			GPIO_PIN_13
#define LASER_GPIO_Port 	GPIOG
                                      
#define RED_LED_OFF()       HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7, GPIO_PIN_RESET)
#define RED_LED_ON()        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_7, GPIO_PIN_SET)
#define RED_LED_TOGGLE()    HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_7)

#define GREEN_LED_OFF()     HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_RESET)
#define GREEN_LED_ON()      HAL_GPIO_WritePin(GPIOF,GPIO_PIN_14, GPIO_PIN_SET)
#define GREEN_LED_TOGGLE()  HAL_GPIO_TogglePin(GPIOF,GPIO_PIN_14)
	 
#define POWER1()	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_2, GPIO_PIN_SET)	
#define POWER2()  HAL_GPIO_WritePin(GPIOH,GPIO_PIN_3, GPIO_PIN_SET)
#define POWER3()	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_4, GPIO_PIN_SET)
#define POWER4()	HAL_GPIO_WritePin(GPIOH,GPIO_PIN_5, GPIO_PIN_SET)

#define Check_Button()	    (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2))//白色按键
#define Psensor_Check()     (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5))//光电门
#define Readmpu()            (HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_6))

#define LASER_ON()		{HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);}
#define LASER_OFF()		{HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);}
#define LASER_TOGGLE()  HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin)

void MX_GPIO_Init(void);

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
