#ifndef __COMMONDEFINE_H
#define __COMMONDEFINE_H


#define false 0
#define true 1

#define M_E (float)2.718281828f
#define M_PI  (float)3.1415926535f
	
#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

//键盘模式底盘速度
#define NORMAL_FORWARD_BACK_SPEED 			400
#define NORMAL_LEFT_RIGHT_SPEED   			300
#define HIGH_FORWARD_BACK_SPEED 			600
#define HIGH_LEFT_RIGHT_SPEED   			500
#define LOW_FORWARD_BACK_SPEED 				220
#define LOW_LEFT_RIGHT_SPEED   				220

#define MOUSE_LR_RAMP_TICK_COUNT			50
#define MOUSR_FB_RAMP_TICK_COUNT			300

#endif
