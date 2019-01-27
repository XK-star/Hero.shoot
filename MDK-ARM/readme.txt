1.HAL_CAN.c 文件有改，使can中断可发，接收

USART1 遥控器
USART2 DM90
UASRT6 外置IMU

吊射时鼠标锁死

YAW  DM90
PITCH 6623

TIM2 Get_Time_Micros()  自动重装载值32位

SPI5 板载mpu6050

板载外设配置：
TIM2-CH1-4
TIM8-CH1-4
TIM12-CH1  蜂鸣器
TIM5-CH1 B PWM1
TIM5-CH2 C PWM2

USART1 遥控器
USART2 上位机
USART3 
USART6 裁判系统
UART7 
USRT8 

I2C2 
SPI4 外置mpu6050
ADC1
DAC


自定义LED Green -PF14   PG1-PG8
                 Red    -PE11
   
四路可控电源：PH2-5 置1使能

GM9015
VCC 7.4~32V
ID 1-4
默认波特率 115200

操作系统 
在stm32f427xx.h下更改
66  #define __NVIC_PRIO_BITS          4       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
core_cm4.h下
247 #define __NVIC_PRIO_BITS          4U                    

串口噪声中断
stm32f427xx_hal_uart.c 
985 //    SET_BIT(huart->Instance->CR3, USART_CR3_EIE);









