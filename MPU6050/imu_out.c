#include "imu_out.h"

volatile float Gyro_Yaw_out,Gyro_Pitch_out,Gyro_Roll_out;
volatile float yaw_angle_out,pitch_angle_out,roll_angle_out;

/**
  * @brief  串口接收外置imu数据处理
  * @param  None
  * @retval None
  */
//串口线朝后，放炮管背面
void IMU_task(uint8_t *pData)
{
	if(pData == NULL)
    {
        return;
    }
	Gyro_Roll_out   = ((int16_t)(pData[3]<<8 |pData[2]))/32768.0*2000.0;
	Gyro_Pitch_out  = -((int16_t)(pData[5]<<8 |pData[4]))/32768.0*2000.0;
	Gyro_Yaw_out    = ((int16_t)(pData[7]<<8 |pData[6]))/32768.0*2000.0;
	
	roll_angle_out  = ((int16_t)(pData[14]<<8 |pData[13]))/32768.0*180.0;
	pitch_angle_out = -((int16_t)(pData[16]<<8 |pData[15]))/32768.0*180.0;
	yaw_angle_out   = -((int16_t)(pData[18]<<8 |pData[17]))/32768.0*180.0;
 
}




