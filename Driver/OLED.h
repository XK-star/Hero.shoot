#ifndef _OLED_H
#define _OLED_H
#include "main.h"

#define OLED_ADDRESS	0x78 //通过调整0R电阻,屏可以0x78和0x7A两个地址 -- 默认0x78

void OLED_Init(void);
void Oled_show(void);


#endif
