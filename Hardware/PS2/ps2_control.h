#ifndef __PS2_CONTROL_H
#define __PS2_CONTROL_H

#include "stdint.h"

typedef struct
{
    uint8_t raw_lx;
    uint8_t raw_ly;
    uint8_t raw_rx;
    uint8_t raw_ry;
    uint8_t filtered_lx;
    uint8_t filtered_ly;
    uint8_t filtered_rx;
    uint8_t filtered_ry;
    int16_t offset_lx;
    int16_t offset_ly;
    int16_t offset_rx;
    int16_t offset_ry;
} PS2_JoystickDebug_t;

// 初始化PS2控制系统
void PS2_Control_Init(void);

// 主处理循环（在main的while中调用）
void PS2_Control_Process(void);
void PS2_Control_TIM3_Callback(void);
// 新增：获取当前PS2模式
uint8_t PS2_GetCurrentMode(void);
void PS2_GetJoystickDebug(PS2_JoystickDebug_t *debug);
//extern uint8_t print_mode;
#endif

