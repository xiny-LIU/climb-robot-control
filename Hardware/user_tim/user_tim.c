#include "user_tim.h"
#include "main.h"  // 用于LED引脚定义
#include "string.h"
#include <stdio.h>
#include "tim.h"
#include "PID.h"
#include "usart6.h"
#include "user_usart.h"
#include "spi4.h"
#include "climb_control.h"

// 统一声明外部调用的任务函数
extern void PS2_Control_TIM3_Callback(void);
extern void Print_Task(void);

TIM3_Manager_t tim3_mgr = {0};

// 中断内执行的“打卡”函数（极速退出）
void TIM3_PeriodElapsed_Handler(void)
{
    tim3_mgr.flag_5ms = 1;
    tim3_mgr.tick_count += 5;  // 每5ms+5
    
    // 5ms 同频任务触发
    tim3_mgr.flag_imu = 1;
    tim3_mgr.flag_encoder = 1;
    
    // 25ms 分频任务触发 (PS2控制)
    tim3_mgr.counter_25ms++;
    if (tim3_mgr.counter_25ms >= 5)  
    {
        tim3_mgr.counter_25ms = 0;
        tim3_mgr.flag_ps2 = 1;       // 仅置位标志，不执行耗时函数
    }
    
    // 500ms 分频任务触发 (串口打印)
    tim3_mgr.counter_500ms++;
    if (tim3_mgr.counter_500ms >= 100)  
    {
        tim3_mgr.counter_500ms = 0;
        tim3_mgr.flag_500ms = 1;  
    }
}

//在main文件调用
//// 定时器中断回调底层接口
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  
//{
//    if (htim->Instance == TIM3)  // 5ms
//    {
//        TIM3_PeriodElapsed_Handler(); 
//    }
//    else if (htim->Instance == TIM6)
//    {
//        HAL_IncTick();
//    }
//    else if (htim->Instance == TIM5)  // 1ms
//    {
//        PID_Loop_1ms();  // PID计算通常要求极高实时性，放在中断内执行是合理的
//    }
//}

// 主循环任务执行调度器（放在 main 的 while(1) 中执行）
void TIM3_Task_Execute(void)
{
    /* ---------------- 1. IMU 数据处理任务 (5ms) ---------------- */
    // IMU由DMA接收，此处仅需处理已在内存中的数据，耗时极短
    if (tim3_mgr.flag_imu && !tim3_mgr.imu_busy)
    {
        tim3_mgr.flag_imu = 0;      
        tim3_mgr.imu_busy = 1;      
        IMU_Process_Task(); 
        Climb_Control_Loop_5ms();        
        tim3_mgr.imu_busy = 0;      
    }
    
    /* ---------------- 2. 编码器 SPI 读取任务 (5ms) ---------------- */
    if (tim3_mgr.flag_encoder && !tim3_mgr.encoder_busy)
    {
        tim3_mgr.flag_encoder = 0;      
        tim3_mgr.encoder_busy = 1;      
        Update_All_Encoders();          
        tim3_mgr.encoder_busy = 0;      
    }
    
    /* ---------------- 3. PS2 遥控器控制任务 (25ms) ---------------- */
    // 移出中断，防止串口/SPI按键解析阻塞中断
    if (tim3_mgr.flag_ps2 && !tim3_mgr.ps2_busy)
    {
        tim3_mgr.flag_ps2 = 0;
        tim3_mgr.ps2_busy = 1;
        PS2_Control_TIM3_Callback();
        tim3_mgr.ps2_busy = 0;
    }

    /* ---------------- 4. 终端状态打印任务 (500ms) ---------------- */
    // 极其耗时，必须放在主循环且优先级应视作最低
    if (tim3_mgr.flag_500ms && !tim3_mgr.print_busy)
    {
        tim3_mgr.flag_500ms = 0;       
        tim3_mgr.print_busy = 1;    
        Print_Task();            
        tim3_mgr.print_busy = 0;    
    }
}
