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
#include "pwm_angle_servo.h"
#include "m3508_position.h"
#include "MotorSample.h"

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
    /* 记录实验任务已经处理过的TIM3节拍，避免主循环空转时重复调用。 */
    static uint32_t motor_sample_last_tick = 0U;

    /* ---------------- 1. IMU 数据处理任务 (5ms) ---------------- */
    // IMU由DMA接收，此处仅需处理已在内存中的数据，耗时极短
    if (tim3_mgr.flag_imu && !tim3_mgr.imu_busy)
    {
        tim3_mgr.flag_imu = 0;      
        tim3_mgr.imu_busy = 1;      
        IMU_Process_Task();       
        tim3_mgr.imu_busy = 0;      
    }
    
    /* ---------------- 2. 编码器 SPI 读取任务 (5ms) ---------------- */
    if (tim3_mgr.flag_encoder && !tim3_mgr.encoder_busy)
    {
    static uint8_t encoder_update_count = 0;
    static uint8_t pwm_angle_servo_started = 0;
        tim3_mgr.flag_encoder = 0;
        tim3_mgr.encoder_busy = 1;

        Update_All_Encoders();

    /*
     * 上电后等待编码器刷新几次，再锁死当前位置并开启闭环。
     * 这样可以避免 encoder_data[] 还是默认值时就锁死。
     */
    if (!pwm_angle_servo_started)
    {
        if (encoder_update_count < 3)
        {
            encoder_update_count++;
        }
        else
        {
            PWM_AngleServo_LockCurrentPosition();
            PWM_AngleServo_Enable(0);

            pwm_angle_servo_started = 1;
        }
    }
    if (PWM_AngleServo_IsEnabled()) {
        PWM_AngleServo_Update_5ms();
    }

        tim3_mgr.encoder_busy = 0;
    }
    
    /* ---------------- 3. 串口调试指令解析任务 (5ms) ---------------- */
    // 借用 flag_imu 或者 flag_encoder 作为 5ms 的触发节拍即可（不需要单开变量）
    if (tim3_mgr.flag_imu == 0 && tim3_mgr.flag_encoder == 0) 
    {
        USART2_ProcessCommand();

    if (M3508_Position_IsEnabled()) {
    M3508_Position_Update_5ms();
    /*
     * 位置环先更新，再推进实验状态机。状态机内部使用HAL_GetTick计时，
     * 不使用HAL_Delay，因此3 s/10 s停留不会阻塞其他控制任务。
     */
    if (motor_sample_last_tick != tim3_mgr.tick_count)
    {
        motor_sample_last_tick = tim3_mgr.tick_count;
        MotorSample_ExperimentTask5ms();
    }
    }
    }
    
    /* ------------- 4. 攀爬运动学协同控制任务 (5ms) ------------- */
// 此时 IMU 和 编码器都刚刚刷新完，数据是最热乎的！
// 借用 flag_imu 或 flag_encoder 作为 5ms 的触发条件即可
//    if (tim3_mgr.flag_imu == 0 && tim3_mgr.flag_encoder == 0) // 确保前置传感器都读完了
//    {
//        Climb_Control_Loop_5ms();
//    }

    /* ---------------- 5. PS2 遥控器控制任务 (25ms) ---------------- */
    // 移出中断，防止串口/SPI按键解析阻塞中断
    if (tim3_mgr.flag_ps2 && !tim3_mgr.ps2_busy)
    {
        tim3_mgr.flag_ps2 = 0;
        tim3_mgr.ps2_busy = 1;
        PS2_Control_TIM3_Callback();
        tim3_mgr.ps2_busy = 0;
    }

    /* ---------------- 6. 终端状态打印任务 (500ms) ---------------- */
    // 极其耗时，必须放在主循环且优先级应视作最低
    if (tim3_mgr.flag_500ms && !tim3_mgr.print_busy)
    {
        tim3_mgr.flag_500ms = 0;       
        tim3_mgr.print_busy = 1;    
        Print_Task();            
        tim3_mgr.print_busy = 0;    
    }
}
