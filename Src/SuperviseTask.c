#include "SuperviseTask.h"
#include "tim.h"
uint16_t DBUSFrameRate = 0;
uint16_t DBUSFrameCounter = 0;

uint16_t ChassisFrameRate[4] = {0};
uint16_t ChassisFrameCounter[4] = {0};

uint16_t PitchFrameRate = 0;
uint16_t PitchFrameCounter = 0;
	
uint16_t YawFrameRate = 0;
uint16_t YawFrameCounter = 0;

uint16_t TurntableFrameRate = 0;
uint16_t TurntableFrameCounter = 0;

uint16_t IMUFrameRate = 0;
uint16_t IMUFrameCounter = 0;

uint16_t UART2FrameRate = 0;
uint16_t UART2FrameCounter = 0;

uint16_t CAN_Send1_FrameRate = 0;
uint16_t CAN_Send1_FrameCounter = 0;

uint16_t CAN_Send2_FrameRate = 0;
uint16_t CAN_Send2_FrameCounter = 0;


uint16_t CAN_Res_FrameRate = 0;
uint16_t CAN_Res_FrameCounter = 0;

uint16_t JudgeFrameRate = 0;
uint16_t JudgeFrameCounter = 0;

uint16_t MiniPCFrameRate = 0;
uint16_t MiniPCFrameCounter = 0;

uint16_t IMU_Out_FrameRate = 0;
uint16_t IMU_Out_FrameCounter = 0;

void Task_Monitor(void)
{
//******************  ↓   数据帧率统计   ↓  **************************/
	static uint32_t time_mark = 0;
	if(Get_Time_Micros() - time_mark > 500000)
	{
		time_mark = Get_Time_Micros();
        //DBUS帧率统计
        DBUSFrameRate = DBUSFrameCounter*2;	
        DBUSFrameCounter = 0;
        
        //云台电机帧率统计
        PitchFrameRate    = PitchFrameCounter*2;
		PitchFrameCounter = 0;
		
        YawFrameRate	  = YawFrameCounter*2;
        YawFrameCounter   = 0;
        
        //底盘电机帧率统计
        ChassisFrameRate[0] = ChassisFrameCounter[0]*2;
        ChassisFrameCounter[0] = 0;
        ChassisFrameRate[1] = ChassisFrameCounter[1]*2;
        ChassisFrameCounter[1] = 0;
        ChassisFrameRate[2] = ChassisFrameCounter[2]*2;
        ChassisFrameCounter[2] = 0;
        ChassisFrameRate[3] = ChassisFrameCounter[3]*2;
        ChassisFrameCounter[3] = 0;
        
				//CAN发送
        CAN_Send1_FrameRate = CAN_Send1_FrameCounter*2;
        CAN_Send1_FrameCounter = 0;
				
				CAN_Send2_FrameRate = CAN_Send1_FrameCounter*2;
        CAN_Send2_FrameCounter = 0;

				//CAN接收
        CAN_Res_FrameRate = CAN_Res_FrameCounter*2;
        CAN_Res_FrameCounter = 0;

        //裁判系统帧率统计
        JudgeFrameRate = JudgeFrameCounter*2;
        JudgeFrameCounter = 0;
		
		//IMU帧率统计
        IMUFrameRate = IMUFrameCounter*2;
        IMUFrameCounter = 0;
				//外部IMU帧率统计
        IMU_Out_FrameRate = IMU_Out_FrameCounter*2;
        IMU_Out_FrameCounter = 0;
		
		//妙算帧率检测
		MiniPCFrameRate = MiniPCFrameCounter*2;
		MiniPCFrameCounter = 0;
	}
        
}
