#ifndef __PS2_CONTROL_H
#define __PS2_CONTROL_H

#include "stdint.h"

// 初始化PS2控制系统
void PS2_Control_Init(void);

// 主处理循环（在main的while中调用）
void PS2_Control_Process(void);
void PS2_Control_TIM3_Callback(void);
// 新增：获取当前PS2模式
uint8_t PS2_GetCurrentMode(void);
//extern uint8_t print_mode;
#endif

