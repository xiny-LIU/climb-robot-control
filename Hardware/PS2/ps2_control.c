#include "ps2_control.h"
#include "ps2.h"
#include "pwm_motor.h"
#include "main.h"  // 用于LED引脚定义
#include "CAN_receive.h"
#include "bsp_can.h"
#include "encoder_counter.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "user_tim.h"  // 引入TIM3管理器
#include "PID.h"
#include "ps2_filter.h"
#include "pwm_angle_servo.h"
#include "m3508_position.h"

// 内部静态变量（文件作用域，外部不可访问）
static uint8_t last_mode = 0xFF;  // 记录上一次的PS2模式
static uint8_t current_mode = 0xFF;

static uint32_t led_flash_timer = 0;  // LED闪烁计时器
static uint32_t motor_stop_timer = 0;  // 改用TIM3时间戳 电机停止计时器（消抖用）
static uint8_t KeyNum = 0;

// 摇杆死区定义
#define STICK_DEAD_ZONE         15      // 摇杆死区范围
#define MOTOR_STOP_DELAY_MS     200    // 无输入后延迟停止时间
#define BUTTON_STABLE_DELAY_MS 50  // 消抖时间 20ms
// 电机转速定义
int CAN_MOTOR_SPEED = 1000;    // M3508
int PWM_MOTOR_SPEED = 10;    // 蜗轮蜗杆电机
// 内部函数声明（静态函数，仅本文件内使用）
static void handle_mode_switch(void);
static void handle_lock_unlock(void);
static void process_can_control(void);
static void process_motor_control(void);
static void handle_led_feedback(void);
//static void printmode_switch(void);
static void ps2_redlight_reset_handle(void);
// 在tim3.c中调用的回调函数
void PS2_Control_TIM3_Callback(void);

// --------------------------------------------------------------
// 接口函数实现
// --------------------------------------------------------------

void PS2_Control_Init(void)
{
    last_mode = 0xFF;
    current_mode = 0xFF;

    motor_stop_timer = 0;
    // 初始化所有电机（冗余调用，确保主程序已初始化）
    Motor_Init_All();
    PS2_Filter_Init();
    CAN_cmd_chassis(0,0,0,0);
}


void PS2_Control_TIM3_Callback(void)
{

    
    // 1. 读取PS2数据
    current_mode = ps2_mode_get();
    KeyNum = ps2_key_serch();  // 内部会调用PS2_ReadData()
    
    // 2. 处理模式切换
    handle_mode_switch();
    
    // 3. 处理锁定/解锁
    handle_lock_unlock();
    
    // 4. 处理CAN控制（独立于电机锁定）
    process_can_control();
    
    // 5. 处理电机控制（使用滤波后的摇杆值）
    process_motor_control();  // 改为新的处理函数
    
    // 6. LED反馈处理
    handle_led_feedback();
    
//    //7. 切换打印模式
//    printmode_switch();

    // 8. 置零处理
    ps2_redlight_reset_handle();
    
}
uint8_t PS2_GetCurrentMode(void) {      // 外部通过函数访问current_mode
    return current_mode;
}

// --------------------------------------------------------------
// 内部静态函数实现
// --------------------------------------------------------------

/**
 * @brief 处理PS2模式切换
 */
static void handle_mode_switch(void)
{
    static uint8_t stable_mode = 0xFF;
    static uint32_t mode_timestamp = 0;
    
    // 模式稳定化：持续200ms不变才确认
    if (current_mode != stable_mode)
    {
        mode_timestamp = tim3_mgr.tick_count;
        stable_mode = current_mode;
        return;
    }
    
    if (tim3_mgr.tick_count - mode_timestamp < 200) return;  // 等待稳定
    
    // 模式已稳定
    if (stable_mode != last_mode)
    {
        Motor_Stop_All();
        CAN_cmd_chassis(0,0,0,0);
        PID_SetTargetSpeed(0, 0);
        PID_SetTargetSpeed(1, 0);
        
        if (stable_mode == PSB_REDLIGHT_MODE)
        {
            M3508_Position_Enable(0);
            PWM_AngleServo_Enable(0);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET); //led2灭
        }
        else if (stable_mode == PSB_GREENLIGHT_MODE)
        {
            M3508_Position_SyncTargetToCurrentBoth();
            PWM_AngleServo_LockCurrentPosition();
            M3508_Position_Enable(1);
            PWM_AngleServo_Enable(1);
            HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET); //led2亮
        }
        
        last_mode = stable_mode;
    }
}

/**
 * @brief 处理锁定/解锁按键
 */
static void handle_lock_unlock(void)
{
    if (current_mode != PSB_REDLIGHT_MODE) return;
    
    // 按键消抖：只在按下瞬间触发
    static uint8_t blue_last = 0, red_last = 0;
    uint8_t blue_now = ps2_get_key_state(PSB_BLUE);
    uint8_t red_now = ps2_get_key_state(PSB_RED);
    
    if (blue_now && !blue_last)
    {
        Motor_Lock_All();
//        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    }
    blue_last = blue_now;
    
    if (red_now && !red_last)
    {
        Motor_Unlock_All();
//        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    }
    red_last = red_now;
}
/**
 * @brief 处理CAN底盘控制（不受电机锁定影响）
 */
static void process_can_control(void)
{
    uint8_t can_active = 0;

    if (M3508_Position_IsEnabled())
    {
        return;
    }

    if (ps2_get_key_state(PSB_L1))
    {
//电机1正转
//        CAN_cmd_chassis(CAN_MOTOR_SPEED, 0, 0, 0);
        PID_SetTargetSpeed(0, CAN_MOTOR_SPEED);
        can_active = 1;
//        printf("PSB_L1\r\n");
    }
    else if (ps2_get_key_state(PSB_L2))
    {
//电机1反转
//        CAN_cmd_chassis(-CAN_MOTOR_SPEED, 0, 0, 0);
        PID_SetTargetSpeed(0, -CAN_MOTOR_SPEED);
        can_active = 1;
//        printf("PSB_L2\r\n");
    }
    else if (ps2_get_key_state(PSB_R1))
    {
//电机2正转
//        CAN_cmd_chassis(0, CAN_MOTOR_SPEED, 0, 0);
        PID_SetTargetSpeed(1, -CAN_MOTOR_SPEED);
        can_active = 1;
//        printf("PSB_R1\r\n");
    }
    else if (ps2_get_key_state(PSB_R2))
    {
//电机2反转
//        CAN_cmd_chassis(0, -CAN_MOTOR_SPEED, 0, 0);
        PID_SetTargetSpeed(1, CAN_MOTOR_SPEED);
        can_active = 1;
//        printf("PSB_R2\r\n");
    }
   // 只在红灯模式下处理CAN控制
   if (ps2_mode_get() == PSB_REDLIGHT_MODE)
 { 
   //重置电机圈数计数器
   if (ps2_get_key_state(PSB_GREEN))
   {
       Encoder_Counter_Reset(0);
       Encoder_Counter_Reset(1);
   }
   //重置电机编码器零点
   else if (ps2_get_key_state(PSB_PINK))
   {
   PWM_AngleServo_SetAllCurrentAsZero();
   }
   else if (ps2_get_key_state(PSB_PAD_UP))
   {
//电机12同时正转
//        CAN_cmd_chassis(0, -CAN_MOTOR_SPEED, 0, 0);
       PID_SetTargetSpeed(0, CAN_MOTOR_SPEED);
       PID_SetTargetSpeed(1, -CAN_MOTOR_SPEED);
       can_active = 1;
//        printf("PSB_PAD_LEFT\r\n");
   }
   else if (ps2_get_key_state(PSB_PAD_DOWN))
   {
//电机12同时反转
//        CAN_cmd_chassis(0, -CAN_MOTOR_SPEED, 0, 0);
       PID_SetTargetSpeed(0, -CAN_MOTOR_SPEED);
       PID_SetTargetSpeed(1, CAN_MOTOR_SPEED);
       can_active = 1;
//        printf("PSB_PAD_RIGHT\r\n");
   }

   //转速加减
   if (KeyNum)
   {
       if (ps2_get_key_state(PSB_PAD_RIGHT))
       {
           CAN_MOTOR_SPEED += 100;
           can_active = 1;
       }
       else if (ps2_get_key_state(PSB_PAD_LEFT))
       {
           CAN_MOTOR_SPEED -= 100;
           can_active = 1;
       } 
       // 速度限幅（500RPM~5000RPM）
       if (CAN_MOTOR_SPEED > 5000) CAN_MOTOR_SPEED = 5000;
       if (CAN_MOTOR_SPEED < 500) CAN_MOTOR_SPEED = 500;
   }
 }
//    // 无CAN输入时发送停止命令（可选）
//    if (!can_active)
//    {
//        CAN_cmd_chassis(0, 0, 0, 0);  // 根据需求决定是否启用
//    }
    // 消抖处理：只有持续无输入超过阈值才停止
    if (can_active)
    {
        motor_stop_timer = tim3_mgr.tick_count;
    }
    else if (tim3_mgr.tick_count - motor_stop_timer > MOTOR_STOP_DELAY_MS)
    {
//       CAN_cmd_chassis(0, 0, 0, 0);
        PID_SetTargetSpeed(0, 0);
        PID_SetTargetSpeed(1, 0);
    }

}

/**
 * @brief 将摇杆模拟值(0x00-0xFF)映射为电机速度百分比
 * @param stick_value: 摇杆原始值 0-255
 * @return speed_percent: 电机速度百分比 0-100
 * @note 摇杆中间值128为停止区，偏离越大速度越快
 *       摇杆极限位置(0或255)对应电机最大转速12rpm
 */
static uint8_t stick_to_speed(unsigned char stick_value)
{
    const int CENTER_VALUE = 128;
    const int MAX_DEVIATION = 128 - STICK_DEAD_ZONE; // 最大有效偏离值
    
    // 计算与中心的绝对偏离值
    int deviation = abs(stick_value - CENTER_VALUE);
    
    // 扣除死区
    int effective_deviation = deviation - STICK_DEAD_ZONE;
    
    // 在死区内返回0速度
    if (effective_deviation <= 0)
    {
        return 0;
    }
    
    // 线性映射到速度百分比 (0-100%)
    // 有效偏离越大，速度百分比越高
    uint8_t speed = (effective_deviation * 100) / MAX_DEVIATION;
    
//    // 限制最大值不超过100
//    if (speed > 100) 
//        speed = 100;
     // 限幅保护
    return (speed > 100) ? 100 : speed;
//    return 100 - speed; // 反比关系
}

/**
 * @brief 处理电机控制（受锁定保护）
 */
static void process_motor_control(void)
{
    uint8_t motor_control_active = 0;

    if (PWM_AngleServo_IsEnabled())
    {
        return;
    }

    // 系统锁定时跳过所有电机控制
    if (Motor_Is_Locked())
    {
        return;  // 不执行任何电机操作
    }
    
    // 红灯模式下处理电机控制
    if (ps2_mode_get() == PSB_REDLIGHT_MODE)
    {
        // 获取原始值
        uint8_t raw_lx = ps2_get_anolog_data(PSS_LX);
        uint8_t raw_ly = ps2_get_anolog_data(PSS_LY);
        uint8_t raw_rx = ps2_get_anolog_data(PSS_RX);
        uint8_t raw_ry = ps2_get_anolog_data(PSS_RY);
        
        // 应用滤波（关键修改点）
        uint8_t ps2_lx = PS2_Filter_Get_LX(raw_lx);
        uint8_t ps2_ly = PS2_Filter_Get_LY(raw_ly);
        uint8_t ps2_rx = PS2_Filter_Get_RX(raw_rx);
        uint8_t ps2_ry = PS2_Filter_Get_RY(raw_ry);
        
        // 调试输出（确认值正确）
//        printf("RAW LY:%d RX:%d | FLT LY:%d RX:%d\r\n", raw_ly, raw_rx, ps2_ly, ps2_rx);
//        printf("LY:%d LX:%d RY:%d RX:%d Active:%d\r\n", ps2_ly, ps2_lx, ps2_ry, ps2_rx, motor_control_active);   //rx自动漂移为0
//        printf("LY:%d LX:%d RY:%d RX:%d Active:%d\r\n", raw_ly, raw_lx, raw_ry, raw_rx, motor_control_active);   //rx自动漂移为0

        // 计算偏移量（后续逻辑完全不变）
        int ly_offset = (int)ps2_ly - 128;
        int lx_offset = (int)ps2_lx - 128;
        int ry_offset = (int)ps2_ry - 128;
        int rx_offset = (int)ps2_rx - 128;
     
     // MOTOR_A控制：左摇杆Y轴（前后推）
    if (ly_offset < -STICK_DEAD_ZONE)  // 向前推
    {
        Motor_SetSpeed(MOTOR_A, stick_to_speed(ps2_ly));
        Motor_SetDirection(MOTOR_A, DIRECTION_FORWARD);
        motor_control_active = 1;
    }
    else if (ly_offset > STICK_DEAD_ZONE)  // 向后拉
    {
        Motor_SetSpeed(MOTOR_A, stick_to_speed(ps2_ly));
        Motor_SetDirection(MOTOR_A, DIRECTION_REVERSE);
        motor_control_active = 1;
    }
    
    // MOTOR_B控制：左摇杆X轴（左右推）
    if (lx_offset < -STICK_DEAD_ZONE)  // 向左推
    {
        Motor_SetSpeed(MOTOR_B, stick_to_speed(ps2_lx));
        Motor_SetDirection(MOTOR_B, DIRECTION_REVERSE);
        motor_control_active = 1;
    }
    else if (lx_offset > STICK_DEAD_ZONE)  // 向右推
    {
        Motor_SetSpeed(MOTOR_B, stick_to_speed(ps2_lx));
        Motor_SetDirection(MOTOR_B, DIRECTION_FORWARD);
        motor_control_active = 1;
    }
    
    // MOTOR_C控制：右摇杆Y轴
    if (ry_offset < -STICK_DEAD_ZONE)  // 向前推
    {
        Motor_SetSpeed(MOTOR_C, stick_to_speed(ps2_ry));
        Motor_SetDirection(MOTOR_C, DIRECTION_REVERSE);
        motor_control_active = 1;
    }
    else if (ry_offset > STICK_DEAD_ZONE)  // 向后拉
    {
        Motor_SetSpeed(MOTOR_C, stick_to_speed(ps2_ry));
        Motor_SetDirection(MOTOR_C, DIRECTION_FORWARD);
        motor_control_active = 1;
    }
    
    // MOTOR_D控制：右摇杆X轴
    if (rx_offset < -STICK_DEAD_ZONE)  // 向左推
    {
        Motor_SetSpeed(MOTOR_D, stick_to_speed(ps2_rx));
        Motor_SetDirection(MOTOR_D, DIRECTION_REVERSE);
        motor_control_active = 1;
    }
    else if (rx_offset > STICK_DEAD_ZONE)  // 向右推
    {
        Motor_SetSpeed(MOTOR_D, stick_to_speed(ps2_rx));
        Motor_SetDirection(MOTOR_D, DIRECTION_FORWARD);
        motor_control_active = 1;
    }
    

  }
     // 绿灯模式下处理电机控制
    if (ps2_mode_get() == PSB_GREENLIGHT_MODE)
    {         
      if (KeyNum)
     {
        if (ps2_get_key_state(PSB_PAD_UP))
        {
        Motor_SetSpeed(MOTOR_A, PWM_MOTOR_SPEED);
        Motor_SetDirection(MOTOR_A, DIRECTION_FORWARD);
        motor_control_active = 1;
        }
        else if (ps2_get_key_state(PSB_PAD_DOWN))
        {
        Motor_SetSpeed(MOTOR_A, PWM_MOTOR_SPEED);
        Motor_SetDirection(MOTOR_A, DIRECTION_REVERSE);
        motor_control_active = 1;
        } 
        else if (ps2_get_key_state(PSB_PAD_LEFT))
        {
        Motor_SetSpeed(MOTOR_B, PWM_MOTOR_SPEED);
        Motor_SetDirection(MOTOR_B, DIRECTION_REVERSE);
        motor_control_active = 1;
        }
        else if (ps2_get_key_state(PSB_PAD_RIGHT))
        {
        Motor_SetSpeed(MOTOR_B, PWM_MOTOR_SPEED);
        Motor_SetDirection(MOTOR_B, DIRECTION_FORWARD);
        motor_control_active = 1;
        }
        else if (ps2_get_key_state(PSB_GREEN))
        {
        Motor_SetSpeed(MOTOR_C, PWM_MOTOR_SPEED);
        Motor_SetDirection(MOTOR_C, DIRECTION_REVERSE);
        motor_control_active = 1;
        }
        else if (ps2_get_key_state(PSB_BLUE))
        {
        Motor_SetSpeed(MOTOR_C, PWM_MOTOR_SPEED);
        Motor_SetDirection(MOTOR_C, DIRECTION_FORWARD);
        motor_control_active = 1;
        }
        else if (ps2_get_key_state(PSB_PINK))
        {
        Motor_SetSpeed(MOTOR_D, PWM_MOTOR_SPEED);
        Motor_SetDirection(MOTOR_D, DIRECTION_REVERSE);
        motor_control_active = 1;
        }
        else if (ps2_get_key_state(PSB_RED))
        {
        Motor_SetSpeed(MOTOR_D, PWM_MOTOR_SPEED);
        Motor_SetDirection(MOTOR_D, DIRECTION_FORWARD);
        motor_control_active = 1;
        }
        
    }
}
//    printf("%d\r\n",motor_control_active);        
// 消抖处理：只有持续无输入超过阈值才停止
    if (motor_control_active)
    {
        motor_stop_timer = tim3_mgr.tick_count;
    }
    else if (tim3_mgr.tick_count - motor_stop_timer > MOTOR_STOP_DELAY_MS)
    {
        Motor_Stop_All();
    }
}

/**
 * @brief LED反馈处理（系统锁定状态指示）
 */
static void handle_led_feedback(void)
{
    if (Motor_Is_Locked())
    {
        if (tim3_mgr.tick_count - led_flash_timer > 100)  // 100ms闪烁
        {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            led_flash_timer = tim3_mgr.tick_count;
        }
    }
    else
    {
        HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    }
//    if (print_mode == 1)
//    {
//        HAL_GPIO_WritePin(LED2_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
//    }
//    else
//    {
//        HAL_GPIO_WritePin(LED2_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
//    }   
}

///**
// * @brief 红灯模式下按下pink键切换打印模式
// */
//uint8_t print_mode = 0;      // 0=打印M3508数据, 1=打印姿态数据
//static void printmode_switch(void)
//{
//    static uint8_t pink_last_raw = 0;      // 记录上一次的【瞬时】状态
//    static uint8_t pink_stable = 0;        // 记录确认【稳定】的状态
//    static uint8_t pink_last_stable = 0;   // 记录上一次的【稳定】状态
//    static uint32_t state_change_timer = 0;// 状态跳变计时器

//    uint8_t pink_now = ps2_get_key_state(PSB_PINK);

//    /* --------------- 消抖过滤层 --------------- */
//    // 如果当前的瞬时状态和上一次读到的不一样，说明状态发生跳变（可能是按下，也可能是抖动）
//    if (pink_now != pink_last_raw)
//    {
//        state_change_timer = tim3_mgr.tick_count; // 只要有跳变，就重置计时器 
//    }
//    // 如果状态没有跳变，并且持续时间超过了消抖阈值 20ms
//    else if (tim3_mgr.tick_count - state_change_timer > BUTTON_STABLE_DELAY_MS)
//    {
//        pink_stable = pink_now; // 状态已经稳定了 20ms，更新稳定状态 
//    }
//    pink_last_raw = pink_now; // 记录瞬时状态给下次比较

//    /* --------------- 逻辑执行层 --------------- */
//    // 用过滤后的“干净状态”做下降沿检测
//    if ((current_mode == PSB_REDLIGHT_MODE) && pink_stable && !pink_last_stable)
//    {
//        print_mode = !print_mode;
//    }
//    pink_last_stable = pink_stable;
//}
    
/**
 * @brief PS2 按键消抖结构体
 */
typedef struct
{
    uint8_t last_raw;              // 上一次读取到的瞬时状态
    uint8_t stable;                // 当前确认稳定后的状态
    uint8_t last_stable;           // 上一次稳定状态，用于检测按下沿
    uint32_t state_change_timer;   // 状态发生跳变时的时间戳
} PS2_KeyDebounce_t;


/**
 * @brief PS2 按键消抖 + 按下沿检测
 * @param key PS2 按键宏，例如 PSB_GREEN、PSB_PINK
 * @param db  对应该按键的消抖状态结构体
 * @return 1 = 按键完成一次稳定按下；0 = 没有新的稳定按下
 */
static uint8_t ps2_key_pressed_debounce(uint16_t key, PS2_KeyDebounce_t *db)
{
    uint8_t key_now = ps2_get_key_state(key);

    /* --------------- 消抖过滤层 --------------- */
    if (key_now != db->last_raw)
    {
        // 只要瞬时状态发生变化，就重新计时
        db->state_change_timer = tim3_mgr.tick_count;
    }
    else if ((uint32_t)(tim3_mgr.tick_count - db->state_change_timer) > BUTTON_STABLE_DELAY_MS)
    {
        // 状态持续稳定超过消抖时间，确认当前状态
        db->stable = key_now;
    }

    db->last_raw = key_now;

    /* --------------- 按下沿检测层 --------------- */
    if (db->stable && !db->last_stable)
    {
        db->last_stable = db->stable;
        return 1;
    }

    db->last_stable = db->stable;
    return 0;
}


/**
 * @brief 红灯模式下的重置功能
 * 
 * GREEN：重置电机圈数计数器
 * PINK ：将当前 PWM 电机角度位置设为零点
 * 
 * 注意：本函数需要在主循环中周期性调用。
 */
static void ps2_redlight_reset_handle(void)
{
    static PS2_KeyDebounce_t green_db = {0};
    static PS2_KeyDebounce_t pink_db  = {0};

    uint8_t green_pressed = ps2_key_pressed_debounce(PSB_GREEN, &green_db);
    uint8_t pink_pressed  = ps2_key_pressed_debounce(PSB_PINK,  &pink_db);

    if (ps2_mode_get() == PSB_REDLIGHT_MODE)
    {
        // GREEN 键：重置电机圈数计数器
        if (green_pressed)
        {
            M3508_Position_ResetEncoderAndSyncTargetBoth();
        }
        // PINK 键：重置电机编码器零点
        else if (pink_pressed)
        {
            PWM_AngleServo_SetAllCurrentAsZero();

        }
    }
}
