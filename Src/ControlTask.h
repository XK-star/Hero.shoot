#ifndef _CONTROLTASK_H
#define _CONTROLTASK_H
#include "stdint.h"

#define PREPARE_TIME_TICK_MS 4000      //prapare time in ms
#define PITCH_PREPARE_TIME_TICK_MS 1500
#define YAW_PREPARE_TIME_TICK_MS 1500
#define STATE_SWITCH_DELAY_TICK 100000   //mode change delay count in ms

void Control_Task(void);
void ControlLoopTaskInit(void);


extern uint32_t time_task_ms;

#endif
