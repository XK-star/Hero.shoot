/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "Driver_Protocol.h"
#include "data.h"
#include "stm32f4xx.h"
#include "Remote.h"
#include "test_imu.h"
#include "Driver_FriMotor.h"
#include "imu_out.h"
#include "imu.h"
#include "Driver_Gimbal.h"
#include "Driver_Chassis.h"
#include "IOTask.h"
#include "pid.h"
#include "CanBusTask.h"
#include "SuperviseTask.h"
#include "StatusMachine.h"
#include "tim.h"

/* Variables -----------------------------------------------------------------*/
#define Upload_Speed 30				//数据上传速度  单位 Hz
#define upload_time (1000000/Upload_Speed)  //计算上传的时间。单位为us

extern UART_HandleTypeDef huart2;

extern ChassisDataTypeDef ChassisData;

extern PID_Regulator_t GMPPositionPID ;
extern PID_Regulator_t GMPSpeedPID ;
extern PID_Regulator_t GMYPositionPID ;
extern PID_Regulator_t GMYSpeedPID ;
extern PID_Regulator_t CMRotatePID ; 
extern PID_Regulator_t CM1SpeedPID ;
extern PID_Regulator_t CM2SpeedPID ;
extern PID_Regulator_t CM3SpeedPID ;
extern PID_Regulator_t CM4SpeedPID ;
extern PID_Regulator_t ShootMotorSpeedPID ;
extern PID_Regulator_t ShootMotorPositionPID ;

extern AppParam_t PIDCaliData;  //保存pitch轴position校准值

//因为头文件包含问题


CIRCLE_BUFF_t Que_Protocol = {0, 0, {0}};	//数据缓冲队列

void Send_Status(void);
void Send_Sensor(void);
void SendRCData(void);
void Send_Moto_Rmp(void);
void Send_PID1(void);
void Send_PID2(void);
void Send_PID5(void);
void Data_Receive_Anl(uint8_t *data_buf,uint8_t num);
void Send_Check(uint8_t check ,uint8_t sum1);
void Data_Event(void);

flag PID_Flag;
uint32_t system_microsecond;   //系统时间 单位ms


void Protocol_SendFrame(uint8_t* pTx, uint8_t _cnt)
{
	uint16_t sum = 0;
	//求和
	for(uint8_t i=0;i<_cnt*2;i++)
	{
		sum += pTx[i];
	}
	//写入帧头
	for(int i = 0;i<4;i++)
	{
		bufferPush(&Que_USART2_Tx, pTx[i]);
	}
	//写入数据
	for(int i = 2;i<_cnt;i++)
	{
		bufferPush(&Que_USART2_Tx, pTx[i*2+1]);//高八位
		bufferPush(&Que_USART2_Tx, pTx[i*2]);  //低八位
	}
	//写入校验值
	bufferPush(&Que_USART2_Tx, sum&0xFF);
}




void Send_Status(void)
{
	uint8_t  _cnt=2;		 //将要发送的数据个数
	int16_t Data[50];		 //将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列

	Data[_cnt++] = (int)(pitch_angle*100);
	Data[_cnt++] = (int)(roll_angle*100);
	Data[_cnt++] = (int)(yaw_angle*100);
	Data[_cnt++] = (int)GMPitchEncoder.filter_rate ;
	Data[_cnt++] = -MPU6050_Real_Data.Gyro_Pitch;
	Data[_cnt++] = 0;
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x01;
	pData_u8[3]	 = (_cnt-2)*2;
	
	Protocol_SendFrame(pData_u8, _cnt);
}



void Send_Sensor(void)
{
	uint8_t  _cnt=2;		 //将要发送的数据个数(u16)
	int16_t Data[50];		 //将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列
	
	
	
	Data[_cnt++] = MPU6050_Raw_Data.Accel_X;
	Data[_cnt++] = MPU6050_Raw_Data.Accel_Y;
	Data[_cnt++] = MPU6050_Raw_Data.Accel_Z;
	Data[_cnt++] = MPU6050_Raw_Data.Gyro_X;
	Data[_cnt++] = MPU6050_Raw_Data.Gyro_Y;
	Data[_cnt++] = MPU6050_Raw_Data.Gyro_Z;
	Data[_cnt++] = AppParamRealUsed.GyroCaliData.GyroXOffset;
	Data[_cnt++] = AppParamRealUsed.GyroCaliData.GyroYOffset;
	Data[_cnt++] = AppParamRealUsed.GyroCaliData.GyroZOffset;
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x02;
	pData_u8[3]	 = (_cnt-2)*2;


	Protocol_SendFrame(pData_u8, _cnt);	
}



void Send_RC_Data(void)
{
	uint8_t  _cnt=2;		 //将要发送的数据个数
	int16_t Data[50];		 //将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列
	
	Data[_cnt++] = RC_CtrlData.rc.ch0;
	Data[_cnt++] = RC_CtrlData.rc.ch1;
	Data[_cnt++] = RC_CtrlData.rc.ch2;
	Data[_cnt++] = RC_CtrlData.rc.ch3;
	Data[_cnt++] = RC_CtrlData.rc.s1;
	Data[_cnt++] = RC_CtrlData.rc.s2;
	Data[_cnt++] = RC_CtrlData.key.v;
	Data[_cnt++] = RC_CtrlData.mouse.x;
	Data[_cnt++] = RC_CtrlData.mouse.y;
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x03;
	pData_u8[3]	 = (_cnt-2)*2;
	
	Protocol_SendFrame(pData_u8, _cnt);
}



void Send_Moto_Rmp(void)
{
	uint8_t  _cnt=2;		 			//将要发送的数据个数
	int16_t Data[50];		 			//将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列
	
	Data[_cnt++] = CM1_Measure.speed_rpm;
	Data[_cnt++] = CM2_Measure.speed_rpm;
	Data[_cnt++] = CM3_Measure.speed_rpm;
	Data[_cnt++] = CM4_Measure.speed_rpm;//原版
	Data[_cnt++] = TurntableEncoder.ecd_angle/96.0f;//临时
	Data[_cnt++] = GMYawEncoder.ecd_angle*10;
	Data[_cnt++] = yaw_angle*100;
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x04;
	pData_u8[3]	 = (_cnt-2)*2;

	Protocol_SendFrame(pData_u8, _cnt);
}


void Send_GMP_PID(void)
{
	uint8_t  _cnt=2;		 			//将要发送的数据个数
	int16_t Data[50];		 			//将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列
	
	
	Data[_cnt++] = GMPPositionPID.kp;		
	Data[_cnt++] = GMPPositionPID.ki*1000;	
	Data[_cnt++] = GMPPositionPID.kd;		
	Data[_cnt++] = GMPSpeedPID.kp;			
	Data[_cnt++] = GMPSpeedPID.ki*1000;	
	Data[_cnt++] = GMPSpeedPID.kd;				
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x05;
	pData_u8[3]	 = (_cnt-2)*2;

	Protocol_SendFrame(pData_u8, _cnt);		
}

void Send_GMY_PID(void)
{
	uint8_t  _cnt=2;		 			//将要发送的数据个数
	int16_t Data[50];		 			//将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列	
	
	Data[_cnt++] = GMYPositionPID.kp;			
	Data[_cnt++] = GMYPositionPID.ki*1000;
	Data[_cnt++] = GMYPositionPID.kd;	
	Data[_cnt++] = GMYSpeedPID.kp;		
	Data[_cnt++] = GMYSpeedPID.ki*1000;	
	Data[_cnt++] = GMYSpeedPID.kd;		
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x06;
	pData_u8[3]	 = (_cnt-2)*2;
	
	
	Protocol_SendFrame(pData_u8, _cnt);		
}

void Send_CM_PID(void)
{
	uint8_t  _cnt=2;		 			//将要发送的数据个数
	int16_t Data[50];		 			//将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列	
	
	Data[_cnt++] = CMRotatePID.kp;		
	Data[_cnt++] = CMRotatePID.ki*1000;	
	Data[_cnt++] = CMRotatePID.kd;		
	Data[_cnt++] = CM1SpeedPID.kp;		
	Data[_cnt++] = CM1SpeedPID.ki*1000;	
	Data[_cnt++] = CM1SpeedPID.kd;			
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x07;
	pData_u8[3]	 = (_cnt-2)*2;
	
	
	Protocol_SendFrame(pData_u8, _cnt);		
}


void Send_R_F_PID(void)
{
	uint8_t  _cnt=2;		 //将要发送的数据个数
	int16_t Data[50];		 //将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列
	
	Data[_cnt++] = GMPPositionPID.ref*100;	
	Data[_cnt++] = GMPPositionPID.fdb*100;	
	Data[_cnt++] = GMPSpeedPID.ref;							
	Data[_cnt++] = GMYPositionPID.ref*100;	
	Data[_cnt++] = GMYPositionPID.fdb*100;	
	Data[_cnt++] = GMPSpeedPID.fdb;					
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x08;
	pData_u8[3]	 = (_cnt-2)*2;	
	
	Protocol_SendFrame(pData_u8, _cnt);	
}

void Send_GM_Encoder(void)
{
	uint8_t  _cnt=2;		 //将要发送的数据个数
	int16_t Data[50];		 //将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列
	
	Data[_cnt++] = GMPitchEncoder.raw_value;	
	Data[_cnt++] = GMPitchEncoder.ecd_bias;	
	Data[_cnt++] = 0;							
	Data[_cnt++] = GMYawEncoder.raw_value;	
	Data[_cnt++] = GMYawEncoder.ecd_bias;		
	Data[_cnt++] = 0;							
	Data[_cnt++] = GetWorkState();			
	Data[_cnt++] = RC_CtrlData.key.v;	
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x09;
	pData_u8[3]	 = (_cnt-2)*2;
	
	Protocol_SendFrame(pData_u8, _cnt);	
}

void Send_Shoot_PID(void)
{
	uint8_t  _cnt=2;		 //将要发送的数据个数
	int16_t Data[50];		 //将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列

	Data[_cnt++] = AppParamRealUsed.TurntableSpeedPID.kp_offset*100;	
	Data[_cnt++] = AppParamRealUsed.TurntableSpeedPID.ki_offset*100;	
	Data[_cnt++] = AppParamRealUsed.TurntableSpeedPID.kd_offset;	
	Data[_cnt++] = AppParamRealUsed.TurntableSpeed_target;	
	Data[_cnt++] = AppParamRealUsed.FrictionWheelMaxSpeed.kp_offset;	
	
	Data[_cnt++] = AppParamRealUsed.TurntablePositionPID.kp_offset;	
	Data[_cnt++] = AppParamRealUsed.TurntablePositionPID.ki_offset*100;	
	Data[_cnt++] = AppParamRealUsed.TurntablePositionPID.kd_offset;	
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x0A;
	pData_u8[3]	 = (_cnt-2)*2;
	
	Protocol_SendFrame(pData_u8, _cnt);		
}

void Send_StatusMachine(void)
{
	uint8_t  _cnt=2;		 //将要发送的数据个数
	int16_t Data[50];		 //将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列
	
	Data[_cnt++] = GetWorkState();		
	Data[_cnt++] = Get_AttackMode();		
	Data[_cnt++] = GetFrictionState();	
	Data[_cnt++] = GetShootState();		
	Data[_cnt++] = GetGimbal_MoveMode();	
	Data[_cnt++] = GetChassis_MoveMode();	
	Data[_cnt++] = 0;//lost_err&0xffff;		//错误位
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x0B;
	pData_u8[3]	 = (_cnt-2)*2;
	
	Protocol_SendFrame(pData_u8, _cnt);		
}

void Send_Ref_Fdb(void)
{
	uint8_t  _cnt=2;		 //将要发送的数据个数
	int16_t Data[50];		 //将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列
	
	Data[_cnt++] = ShootMotorSpeedPID.ref;		
	Data[_cnt++] = Turntable_Measure.speed_rpm;	
	
	Data[_cnt++] = ChassisData.ChassisSpeedRef.Y;
	Data[_cnt++] = ChassisData.ChassisSpeedRef.X;
	Data[_cnt++] = ChassisData.ChassisSpeedRef.Omega;
	
	Data[_cnt++] = CM1SpeedPID.ref;
	Data[_cnt++] = CM2SpeedPID.ref;
	Data[_cnt++] = CM3SpeedPID.ref;
	Data[_cnt++] = CM4SpeedPID.ref;

//	Data[_cnt++] = JudgingInfo.StudentGameProcessInfo.remainPower;
//	Data[_cnt++] = JudgingInfo.StudentGameProcessInfo.remainLifeValue;
//	Data[_cnt++] = executionTime1000Hz;
//	Data[_cnt++] = JudgingInfo.LifeChangedInfo.strikeId);
	
	Data[_cnt++] = GMYSpeedPID.ref;	
	Data[_cnt++] = GMPSpeedPID.ref;	
	Data[_cnt++] = GMYPositionPID.ref*100;	
	Data[_cnt++] = GMPPositionPID.ref*100;
	
	Data[_cnt++] = GMYSpeedPID.fdb;
	Data[_cnt++] = GMPSpeedPID.fdb;
	Data[_cnt++] = GMYPositionPID.fdb*100;
	Data[_cnt++] = GMPPositionPID.fdb*100;
	Data[_cnt++] = GMYawEncoder.ecd_angle*100;
	Data[_cnt++] = GMPitchEncoder.ecd_angle*100;
	Data[_cnt++] =	ShootMotorPositionPID.ref;	
	Data[_cnt++] =	ShootMotorPositionPID.fdb;	
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x0C;
	pData_u8[3]	 = (_cnt-2)*2;
	
	Protocol_SendFrame(pData_u8, _cnt);	
}

void Send_Judge(void)
{
//	uint8_t  _cnt=2;		 //将要发送的数据个数
//	uint16_t Data[50];		 //将要发送的数据(u16)
//	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列
//	
//	memcpy(&Data[_cnt], &JudgingInfo, sizeof(JudgingInfo_t));
//	_cnt+=sizeof(JudgingInfo_t)/2;
//	pData_u8[0]	 = 0xAA;
//	pData_u8[1]	 = 0xAA;
//	pData_u8[2]	 = 0x0D;
//	pData_u8[3]	 = (_cnt-2)*2;
//	
//	Protocol_SendFrame(pData_u8, _cnt);	
}

void Send_Mainifold(void)
{
	uint8_t  _cnt=2;		 //将要发送的数据个数
	uint16_t Data[50];		 //将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列
	
//	Data[_cnt++] = Mainifold_RawData.Yaw_angle*100);	
//	Data[_cnt++] = Mainifold_RawData.Pitch_angle*100);	
//	Data[_cnt++] = (Get_Time_Micros()-Mainifold_RawData.TimeStamp)/1000000.0f*100);	//发送时间 单位s 
//	Data[_cnt++] = Mainifold_RawData.Mode);
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x0E;
	pData_u8[3]	 = (_cnt-2)*2;
	
	Protocol_SendFrame(pData_u8, _cnt);	
}

void Send_FrameRate(void)
{
	uint8_t  _cnt=2;		 //将要发送的数据个数(u16)
	int16_t Data[50];		 //将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列

	Data[_cnt++] = DBUSFrameRate;	
	Data[_cnt++] = PitchFrameRate;	
	Data[_cnt++] = YawFrameRate;	//发送时间 单位s 
	Data[_cnt++] = ChassisFrameRate[0];
	Data[_cnt++] = ChassisFrameRate[1];
	Data[_cnt++] = ChassisFrameRate[2];
	Data[_cnt++] = ChassisFrameRate[3];
	Data[_cnt++] = JudgeFrameRate;
	Data[_cnt++] = IMUFrameRate;
	Data[_cnt++] = MiniPCFrameRate;
	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x0F;
	pData_u8[3]	 = (_cnt-2)*2;
	

	Protocol_SendFrame(pData_u8, _cnt);		
}

//接受数据解包
static AppParam_t calibrate_temp;
void Data_Receive_Anl(uint8_t *data_buf,uint8_t num)
{
	uint8_t sum = 0;
	uint8_t cmd = 0;
	//求和
	for(uint8_t i=0;i<(num-1);i++)
	{
		sum += data_buf[i];
	}
	//校验
	if(sum!= data_buf[num-1])	
	{	
		return;
	}	
	
	static uint8_t RX_flag[4] = {0};
	cmd = data_buf[2];
	if(cmd==0X01)
	{
		uint8_t data = data_buf[4];
		if(data==0X02)
		{
			SetCaliCmdFlag(CALI_FLAG_GYRO);
			return;
		}
		if(data==0X04)
		{
			SetCaliCmdFlag(CALI_END_FLAG_MAG);	
			return;			
		}
		if(data==0X05)
		{
			SetCaliCmdFlag(CALI_FLAG_GIMBAL);	
			return;			
		}
		if(data == 0x06)
		{
			SetCaliCmdFlag(CALI_FLAG_LITTLEGYRO);	
		}
			return;
	}
	
	if(cmd==0X02)
	{
		uint8_t data = data_buf[4];
		if(data==0X01)//上位机获取PID命令
		{
			PID_Flag.SendPID1 = 1;
			PID_Flag.SendPID2 = 1;
			PID_Flag.SendPID5 = 1;
			return;
		}
		if(data==0X02)//上位机获取Judge命令
		{
			PID_Flag.SendJudge = 1;
			return;
		}
		return;
	}
	
	if(cmd==0X09)	
	{
		
		calibrate_temp.TurntableSpeedPID.kp_offset = 	(int16_t)(((data_buf[4])<<8)|(data_buf[5]));
		calibrate_temp.TurntableSpeedPID.ki_offset = 	(int16_t)(((data_buf[6])<<8)|(data_buf[7]));
		calibrate_temp.TurntableSpeedPID.kd_offset =		(int16_t)(((data_buf[8])<<8)|(data_buf[9]));
		
		calibrate_temp.FrictionWheelMaxSpeed.kp_offset = (int16_t)(((data_buf[10])<<8)|(data_buf[11]));
		calibrate_temp.TurntableSpeed_target = 			(int16_t)(((data_buf[12])<<8)|(data_buf[13]));
		
		calibrate_temp.TurntablePositionPID.kp_offset =	(int16_t)(((data_buf[14])<<8)|(data_buf[15]));
		calibrate_temp.TurntablePositionPID.ki_offset =	(int16_t)(((data_buf[16])<<8)|(data_buf[17]));
		calibrate_temp.TurntablePositionPID.kd_offset =	(int16_t)(((data_buf[18])<<8)|(data_buf[19]));
		RX_flag[0] =1;
			
	}
	
	if(cmd==0X10)	
	{
		calibrate_temp.PitchPositionPID.kp_offset = (int16_t)(((data_buf[4])<<8)|(data_buf[5]));
		calibrate_temp.PitchPositionPID.ki_offset = (int16_t)(((data_buf[6])<<8)|(data_buf[7]));
		calibrate_temp.PitchPositionPID.kd_offset = (int16_t)(((data_buf[8])<<8)|(data_buf[9]));
		calibrate_temp.PitchSpeedPID.kp_offset 	 = (int16_t)(((data_buf[10])<<8)|(data_buf[11]));
		calibrate_temp.PitchSpeedPID.ki_offset 	 = (int16_t)(((data_buf[12])<<8)|(data_buf[13]));
		calibrate_temp.PitchSpeedPID.kd_offset 	 = (int16_t)(((data_buf[14])<<8)|(data_buf[15]));
		
		RX_flag[1] =1;
	}
	
	if(cmd==0X11)	
	{
		calibrate_temp.YawPositionPID.kp_offset  = (int16_t)(((data_buf[4])<<8)|(data_buf[5]));
		calibrate_temp.YawPositionPID.ki_offset  = (int16_t)(((data_buf[6])<<8)|(data_buf[7]));
		calibrate_temp.YawPositionPID.kd_offset  = (int16_t)(((data_buf[8])<<8)|(data_buf[9]));
		calibrate_temp.YawSpeedPID.kp_offset    = (int16_t)(((data_buf[10])<<8)|(data_buf[11]));
		calibrate_temp.YawSpeedPID.ki_offset    = (int16_t)(((data_buf[12])<<8)|(data_buf[13]));
		calibrate_temp.YawSpeedPID.kd_offset    = (int16_t)(((data_buf[14])<<8)|(data_buf[15]));
		
		RX_flag[2] =1;				
	}
	
	if(cmd==0X12)	
	{
		calibrate_temp.RotateSpeedPID.kp_offset = (int16_t)(((data_buf[4])<<8)|(data_buf[5]));
		calibrate_temp.RotateSpeedPID.ki_offset = (int16_t)(((data_buf[6])<<8)|(data_buf[7]));
		calibrate_temp.RotateSpeedPID.kd_offset = (int16_t)(((data_buf[8])<<8)|(data_buf[9]));
		calibrate_temp.ChassisSpeedPID.kp_offset = (int16_t)(((data_buf[10])<<8)|(data_buf[11]));
		calibrate_temp.ChassisSpeedPID.ki_offset = (int16_t)(((data_buf[12])<<8)|(data_buf[13]));
		calibrate_temp.ChassisSpeedPID.kd_offset = (int16_t)(((data_buf[14])<<8)|(data_buf[15]));
		
		RX_flag[3] =1;				
	}
	
	if(cmd==0X14)	
	{
		Send_Check(0x14,sum);
		return;		
	}
	if(RX_flag[0] || RX_flag[1] || RX_flag[2] || RX_flag[3])
	{
		RX_flag[0] = 0;
		RX_flag[1] = 0;
		RX_flag[2] = 0;
		RX_flag[3] = 0;
		SetWorkState(STOP_STATE);		//停机 防止发疯
		PIDCaliProcess(&calibrate_temp);   //保存calidata到static变量中		
		SetCaliCmdFlag(CALI_FLAG_PID);	//PID参数
		Send_Check(0x09,114);			//忽略114， 上位机坑先留着
	}
}

void Send_Check(uint8_t check ,uint8_t sum1)
{
	uint16_t Data[50];		 //将要发送的数据(u16)
	uint8_t* pData_u8 = (uint8_t*)Data;//u8形式写入队列

	pData_u8[0]	 = 0xAA;
	pData_u8[1]	 = 0xAA;
	pData_u8[2]	 = 0x11;
	pData_u8[3]	 = 2;
	pData_u8[4] = check;
	pData_u8[5] = sum1;

	uint16_t sum = 0;
	//求和
	for(uint8_t i=0;i<6;i++)
	{
		sum += pData_u8[i];
	}
	pData_u8[6] = sum&0xFF;
	HAL_UART_Transmit(&huart2,pData_u8, 7,10);
	
}
void Data_Event(void)
{
	 if(PID_Flag.SendPID1 == 1 )	 
	 {
		 PID_Flag.SendPID1 = 0;
		 Send_GMP_PID();
	 }
	if(PID_Flag.SendPID2 == 1 )	 
	 {
		 PID_Flag.SendPID2 = 0;
		 Send_GMY_PID();
		 Send_Shoot_PID();
	 }
	 if(PID_Flag.SendJudge == 1)
	 {
		PID_Flag.SendJudge = 0;
		Send_Judge();
	 }
}


//帧格式FrameHead AAAF(2Byte)
//	  	Function	  (1Byte)
//	  	Length		  (1Byte)
//	  	Data		  (nByte)
//	  	Sum			  (1Byte)
static uint8_t data_len;
uint8_t frame_temp[50];
uint16_t get_labview=0;
void Dataframe_Process(void)
{
	while(bufferlen(&Que_Protocol) != 0)
	{
		get_labview++;
		//寻找帧头AAAF
		if(bufferPop(&Que_Protocol,frame_temp) == 0xAA)
		{
			if(bufferPop(&Que_Protocol,&frame_temp[1]) == 0xAF)
			{
				buffer_multiPop(&Que_Protocol, &frame_temp[2], 2);
				data_len = frame_temp[3];
				buffer_multiPop(&Que_Protocol, &frame_temp[4], data_len+1);
				Data_Receive_Anl(frame_temp, data_len+5);
			}
		}
	}
}

uint8_t Protocol_Send[300] = {0}; 
uint16_t Protocol_len;
uint16_t Pro_count;
void Protocol(void)
{
	Pro_count++;
	if(Get_Time_Micros() - system_microsecond > upload_time)
	{
		system_microsecond = Get_Time_Micros();

//		Send_Judge();	//64 数据量太大 一起发会卡死 
		Send_GMP_PID();
    Send_GMY_PID();
    Send_CM_PID();
		Send_R_F_PID();
		Send_Sensor();//9
		Send_Status();//6
		Send_RC_Data();	//9
		Send_Moto_Rmp();//6	
		Send_Ref_Fdb();	//22 最长
		Send_StatusMachine();//7
//		Send_Mainifold();//4		
		Send_FrameRate();		
	}
	Dataframe_Process();//解包接收信息
	Data_Event();
	Protocol_len = bufferlen(&Que_USART2_Tx);
	if(Protocol_len != 0)
	{
		buffer_multiPop(&Que_USART2_Tx, Protocol_Send, Protocol_len);
		HAL_UART_Transmit_DMA(&huart2,Protocol_Send, Protocol_len);
	}
}

