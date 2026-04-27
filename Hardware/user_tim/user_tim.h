#ifndef __USER_TIM_H
#define __USER_TIM_H

#include "stdint.h"
//extern volatile uint8_t tim3_flag;
//extern volatile uint8_t tim3_counter; 

// TIM3管理结构体
typedef struct {
    volatile uint8_t flag_5ms;      // 每5ms置1
    volatile uint16_t counter_25ms; // 25ms分频计数器
    volatile uint32_t tick_count;   // 累计时间戳（用于消抖）
    
    // 新增：IMU任务相关
    volatile uint8_t flag_imu;      // IMU任务执行标志
    volatile uint8_t imu_busy;      // IMU任务防重入标志
    
    // 新增：1秒打印分频
    volatile uint16_t counter_500ms;   // 1秒分频计数器（5ms * 100 = 500ms）
    volatile uint8_t flag_500ms;       // 1秒标志
    volatile uint8_t print_busy;    // 打印任务防重入标志
} TIM3_Manager_t;

extern TIM3_Manager_t tim3_mgr;

// 在stm32f4xx_it.c中调用这个函数
void TIM3_PeriodElapsed_Handler(void);

// 新增：IMU任务函数声明（在imu_usart6.c中实现）
extern void IMU_Process_Task(void);

// 新增：任务执行函数
void TIM3_Task_Execute(void);
#endif

