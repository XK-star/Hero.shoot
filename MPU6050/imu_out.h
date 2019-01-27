#ifndef IMU_OUT_H
#define IMU_OUT_H

#include "main.h"

void IMU_task(uint8_t *pData);
void IMU_value(void);

extern volatile float Gyro_Yaw_out,Gyro_Pitch_out,Gyro_Roll_out;
extern volatile float yaw_angle_out,pitch_angle_out,roll_angle_out;

#endif
