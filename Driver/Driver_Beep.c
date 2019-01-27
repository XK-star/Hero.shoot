#include "Driver_Beep.h"
#include "stdint.h"
#include "tim.h"
#include "gpio.h"
#include "IOTask.h"

#define MUSIC_LENGTH(x)  (sizeof(x)/sizeof(x[0]))

extern uint8_t Gimbal_Cali_Success;




const uint16_t tone_tab[] = 
{
  3822,  3405, 3033, 2863, 2551, 2272, 2024,	//bass 1~7
  1911,  1702, 1526, 1431, 1275, 1136, 1012,	//mid 1~7
  955,   851,  758,  715,   637, 568,   506,	//treble 1~7
};






//播放单个音符
void Sing(Sound_tone_e tone)
{
  if(Silent == tone)
    BEEP_CH = 0;
  else 
  {
    BEEP_ARR = tone_tab[tone];
    BEEP_CH = tone_tab[tone] / 2;
  }
}


void Sing_Music(const Sound_tone_e *music, uint8_t length)
{
	static int i = 0;
	if(i < length)
	{
		Sing(music[i++]);
	}
	else
	{
		i = 0;
	}
}

//默认 静音
const Sound_tone_e No_music[] = 
{
  Silent,Silent,Silent,Silent,Silent,Silent,Silent,Silent
};

//单次播放 开机音乐
const Sound_tone_e Startup_music[] = 
{
  Do1H, Mi3H, So5H, So5H, So5H, So5H, Silent,Silent,Silent,Silent
};


//自动模式 警告声
const Sound_tone_e Cali_Mode_music[] = 
{
  So5H, So5H, So5H, So5H,Silent,Silent,Silent,Silent,Silent
};

//卡弹 警告声
const Sound_tone_e Bullet_Block_music[] = 
{
  Do1H, Mi3H, So5H, So5H,So5H,
};




void BEEP_Task(void)
{
	static uint32_t time = 0;
	if((Get_Time_Micros() -time > 150000))		//蜂鸣器启动音乐 150ms周期更换音符
	{
		time = Get_Time_Micros();
		if(Get_Time_Micros() < 1500000)						//开机音乐
		{
			Sing_Music(Startup_music,MUSIC_LENGTH(Startup_music));
		}
		else if(Gimbal_Cali_Success == 1)//云台校准成功
		{
			static uint8_t count = 0;
			Sing_Music(Bullet_Block_music,MUSIC_LENGTH(Bullet_Block_music));
			if(count++ > 3)
			{
				count = 0;
				Gimbal_Cali_Success = 0;
			}	
		}
		else												//静音
		{
			Sing_Music(No_music,MUSIC_LENGTH(No_music));
		}
		RED_LED_TOGGLE();
		GREEN_LED_TOGGLE();
	}
	
}


