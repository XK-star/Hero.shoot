#ifndef __IMU_H
#define __IMU_H

#include "stm32f4xx.h"


	
void Init_Quaternion(void);
void IMU_getYawPitchRoll(volatile float * ypr); //更新姿态
void GetPitchYawGxGyGz(void);
void IMU_Task(void);

extern int16_t MPU6050_FIFO[6][11];//[0]-[9]为最近10次数据 [10]为10次数据的平均值
//extern int16_t HMC5883_FIFO[3][11];//[0]-[9]为最近10次数据 [10]为10次数据的平均值 注：磁传感器的采样频率慢，所以单独列出

extern volatile float angle[3];
extern volatile float yaw_angle,pitch_angle,roll_angle; //使用到的角度值
extern volatile float mygetqval[9];

#endif


