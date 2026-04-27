#include "user_tim.h"
#include "main.h"  // 用于LED引脚定义
#include "string.h"
#include <stdio.h>
#include "tim.h"
#include "PID.h"
#include "usart6.h"
#include "user_usart.h"
TIM3_Manager_t tim3_mgr = {0};
volatile uint8_t tim3_flag = 0;
//volatile uint8_t tim3_counter = 0;   // 软件计数器

void TIM3_PeriodElapsed_Handler(void)
{
    tim3_mgr.flag_5ms = 1;
    tim3_mgr.tick_count += 5;  // 每5ms+5
    
    // 新增：设置IMU任务标志（与5ms同频）
    tim3_mgr.flag_imu = 1;
    
    // 软件分频到25ms
    tim3_mgr.counter_25ms++;
    if (tim3_mgr.counter_25ms >= 5)  // 5 * 5ms = 25ms
    {
        tim3_mgr.counter_25ms = 0;
        // 外部回调函数在ps2_control.c中实现
        extern void PS2_Control_TIM3_Callback(void);
        PS2_Control_TIM3_Callback();  // 执行PS2控制
    }
    
    // 新增：软件分频到0.5秒（500ms / 5ms = 100次）
    tim3_mgr.counter_500ms++;
    if (tim3_mgr.counter_500ms >= 100)  // 100 * 5ms = 500ms = 0.5s
    {
        tim3_mgr.counter_500ms = 0;
        tim3_mgr.flag_500ms = 1;  // 设置0.5秒标志
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  
{
    if (htim->Instance == TIM3)//5ms
    {
        tim3_flag = 1;  // 设置标志，不执行耗时操作
        TIM3_PeriodElapsed_Handler();  // 调用封装函数
//        PID_Loop_1ms();
    }
    if (htim->Instance == TIM6)
    {
        HAL_IncTick();
    }
    if (htim->Instance == TIM5)  // 1ms
    {
    PID_Loop_1ms();
    }
}


// 新增：任务执行函数（在主循环中调用）
void TIM3_Task_Execute(void)
{
    // IMU任务：5ms周期，带防重入保护（保持原有）
    if (tim3_mgr.flag_imu && !tim3_mgr.imu_busy)
    {
        tim3_mgr.flag_imu = 0;      // 清除标志
        tim3_mgr.imu_busy = 1;      // 设置执行中标志
        
        IMU_Process_Task();         // 执行IMU数据处理（此处不再打印，只处理数据）
        
        tim3_mgr.imu_busy = 0;      // 清除执行标志
    }
    
    // 新增：0.5秒打印任务（从IMU处理中分离出来）
    if (tim3_mgr.flag_500ms && !tim3_mgr.print_busy)
    {
        tim3_mgr.flag_500ms = 0;       // 清除标志
        tim3_mgr.print_busy = 1;    // 设置执行中标志
        
        Print_Task();            // 执行打印任务
        
        tim3_mgr.print_busy = 0;    // 清除执行标志
    }
}
