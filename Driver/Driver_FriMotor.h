#ifndef __DRIVER_FRIMOTOR_H
#define __DRIVER_FRIMOTOR_H
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"

#define BULLET_SPEED 1600
#define PWM1  TIM5->CCR2
#define PWM2  TIM5->CCR3

#define InitFrictionWheel()     \
        PWM1 = 800;             \
        PWM2 = 800;			\

#define SetFrictionWheelSpeed(x) \
        PWM1 = x;                \
        PWM2 = x;				\
				
#define REDUCTION 36.0f //¼õËÙ±È
						
typedef struct
{
	int16_t Speed;
	float	Angle;
}Turntale_ref_t;

void FriMotor_Control(void);
void Turntable_Control(void);
void Shoot_Single(uint8_t num);
void LASER_Task(void);
		
extern uint8_t  BulletNumber;
extern uint8_t shootdouble;
extern uint8_t shootnum;

#endif