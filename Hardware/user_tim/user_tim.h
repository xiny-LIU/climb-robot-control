#ifndef __USER_TIM_H
#define __USER_TIM_H

#include "stdint.h"

// TIM3管理结构体
typedef struct {
    volatile uint8_t flag_5ms;      // 每5ms置1
    volatile uint16_t counter_25ms; // 25ms分频计数器
    volatile uint32_t tick_count;   // 累计时间戳（用于消抖）
    
    // IMU任务相关 (5ms)
    volatile uint8_t flag_imu;      
    volatile uint8_t imu_busy;      
    
    // 编码器任务相关 (5ms)
    volatile uint8_t flag_encoder;  
    volatile uint8_t encoder_busy;  
    
    // 新增：PS2遥控器任务相关 (25ms)
    volatile uint8_t flag_ps2;      
    volatile uint8_t ps2_busy;      

    // 1秒打印分频 (500ms)
    volatile uint16_t counter_500ms;   
    volatile uint8_t flag_500ms;       
    volatile uint8_t print_busy;    
    
} TIM3_Manager_t;

extern TIM3_Manager_t tim3_mgr;

// 函数声明
void TIM3_PeriodElapsed_Handler(void);
extern void IMU_Process_Task(void);
void TIM3_Task_Execute(void);

#endif
