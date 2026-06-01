#include "pwm_motor.h"
#include "tim.h"
#include <stdio.h>

// 电机PWM配置参数
#define PWM_FREQUENCY_HZ        20000   // PWM频率20kHz
#define PWM_TIMER_HANDLE        &htim2  // TIM2作为PWM源
#define PWM_PRESCALER           89
#define PWM_PERIOD_ARR          49

// PWM通道映射
static const uint32_t motor_pwm_channels[] = {
    TIM_CHANNEL_1,  // MOTOR_A - PA0
    TIM_CHANNEL_2,  // MOTOR_B - PA1
    TIM_CHANNEL_3,  // MOTOR_C - PA2
    TIM_CHANNEL_4   // MOTOR_D - PA3
};

// 电机控制实例
Motor_Control_t motor_A = {.id = MOTOR_A, .state = MOTOR_STATE_STOPPED, .direction = DIRECTION_FORWARD, .speed_percent = 0};
Motor_Control_t motor_B = {.id = MOTOR_B, .state = MOTOR_STATE_STOPPED, .direction = DIRECTION_FORWARD, .speed_percent = 0};
Motor_Control_t motor_C = {.id = MOTOR_C, .state = MOTOR_STATE_STOPPED, .direction = DIRECTION_FORWARD, .speed_percent = 0};
Motor_Control_t motor_D = {.id = MOTOR_D, .state = MOTOR_STATE_STOPPED, .direction = DIRECTION_FORWARD, .speed_percent = 0};

// 内部辅助函数
static Motor_Control_t* get_motor_instance(Motor_ID_t motor_id);
static void update_motor_gpio(Motor_ID_t motor_id);

// 电机锁定控制（用于PS2紧急停止）
static uint8_t motor_system_locked = 0;  // 0=解锁, 1=锁定

/**
 * @brief 根据电机ID获取控制实例
 */
static Motor_Control_t* get_motor_instance(Motor_ID_t motor_id)
{
    switch(motor_id) {
        case MOTOR_A: return &motor_A;
        case MOTOR_B: return &motor_B;
        case MOTOR_C: return &motor_C;
        case MOTOR_D: return &motor_D;
        default: return &motor_A;
    }
}

/**
 * @brief 更新电机GPIO状态
 */
static void update_motor_gpio(Motor_ID_t motor_id)
{
    Motor_Control_t* motor = get_motor_instance(motor_id);
    
    // 根据电机状态设置GPIO
    if (motor->state == MOTOR_STATE_STOPPED || motor->speed_percent == 0) {
        // 停止状态：刹车生效，电源切断
        switch(motor_id) {
            case MOTOR_A:
                HAL_GPIO_WritePin(A_Brake_GPIO_Port, A_Brake_Pin, GPIO_PIN_RESET);    // 刹车
                HAL_GPIO_WritePin(POWER1_GPIO_Port, POWER1_Pin, GPIO_PIN_RESET);      // 断电
                break;
            case MOTOR_B:
                HAL_GPIO_WritePin(B_Brake_GPIO_Port, B_Brake_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(POWER2_GPIO_Port, POWER2_Pin, GPIO_PIN_RESET);
                break;
            case MOTOR_C:
                HAL_GPIO_WritePin(C_Brake_GPIO_Port, C_Brake_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(POWER3_GPIO_Port, POWER3_Pin, GPIO_PIN_RESET);
                break;
            case MOTOR_D:
                HAL_GPIO_WritePin(D_Brake_GPIO_Port, D_Brake_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(POWER4_GPIO_Port, POWER4_Pin, GPIO_PIN_RESET);
                break;
        }
    } else {
        // 运行状态：解除刹车，接通电源
        switch(motor_id) {
            case MOTOR_A:
                HAL_GPIO_WritePin(A_Brake_GPIO_Port, A_Brake_Pin, GPIO_PIN_SET);      // 解除刹车
                HAL_GPIO_WritePin(POWER1_GPIO_Port, POWER1_Pin, GPIO_PIN_SET);        // 通电
                // 设置方向
                HAL_GPIO_WritePin(A_Reverse_GPIO_Port, A_Reverse_Pin, 
                    motor->direction == DIRECTION_REVERSE ? GPIO_PIN_RESET : GPIO_PIN_SET);
                break;
            case MOTOR_B:
                HAL_GPIO_WritePin(B_Brake_GPIO_Port, B_Brake_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(POWER2_GPIO_Port, POWER2_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(B_Reverse_GPIO_Port, B_Reverse_Pin, 
                    motor->direction == DIRECTION_REVERSE ? GPIO_PIN_RESET : GPIO_PIN_SET);
                break;
            case MOTOR_C:
                HAL_GPIO_WritePin(C_Brake_GPIO_Port, C_Brake_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(POWER3_GPIO_Port, POWER3_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(C_Reverse_GPIO_Port, C_Reverse_Pin, 
                    motor->direction == DIRECTION_REVERSE ? GPIO_PIN_RESET : GPIO_PIN_SET);
                break;
            case MOTOR_D:
                HAL_GPIO_WritePin(D_Brake_GPIO_Port, D_Brake_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(POWER4_GPIO_Port, POWER4_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(D_Reverse_GPIO_Port, D_Reverse_Pin, 
                    motor->direction == DIRECTION_REVERSE ? GPIO_PIN_RESET : GPIO_PIN_SET);
                break;
        }
    }
}

/**
 * @brief 计算PWM占空比对应的CCR值
 */
static uint32_t calculate_ccr_value(uint8_t speed_percent)
{
    if (speed_percent > 100) speed_percent = 100;
    
    // 反比映射：speed_percent=100 → CCR=0（转速最快）
    // speed_percent=0 → CCR=ARR（转速最慢）
    float inverted_percent = 100.0f - speed_percent;
    
    // 精确公式：(CCR + 1) / (ARR + 1) = inverted_percent / 100
    // → CCR = inverted_percent * (ARR + 1) / 100 - 1
    float ccr_float = inverted_percent * (PWM_PERIOD_ARR + 1) / 100.0f - 1.0f;    
    
    // 边界保护
    if (ccr_float < 0) return 0;
    if (ccr_float > PWM_PERIOD_ARR) return PWM_PERIOD_ARR;
    
    return (uint32_t)ccr_float;
    

}

// 公开API函数实现

void Motor_Init(Motor_ID_t motor_id)
{
    // 启动对应PWM通道
    HAL_TIM_PWM_Start(PWM_TIMER_HANDLE, motor_pwm_channels[motor_id]);
    
    // 初始占空比为0
    __HAL_TIM_SET_COMPARE(PWM_TIMER_HANDLE, motor_pwm_channels[motor_id], calculate_ccr_value(0));
    
    // 更新GPIO状态（初始为停止状态）
    update_motor_gpio(motor_id);
}

void Motor_Init_All(void)
{
    Motor_Init(MOTOR_A);
    Motor_Init(MOTOR_B);
    Motor_Init(MOTOR_C);
    Motor_Init(MOTOR_D);
}

void Motor_Start(Motor_ID_t motor_id)
{
    if (motor_system_locked) return;  // 如果系统锁定，直接返回
    Motor_Control_t* motor = get_motor_instance(motor_id);
    
    if (motor->speed_percent == 0) {
        // 如果没有设置速度，默认30%
        Motor_SetSpeed(motor_id, 70);
    } else {
        motor->state = MOTOR_STATE_RUNNING;
        update_motor_gpio(motor_id);
    }
}

void Motor_Stop(Motor_ID_t motor_id)
{
     if (motor_system_locked) return;  // 如果系统锁定，直接返回
    Motor_Control_t* motor = get_motor_instance(motor_id);
    
    motor->state = MOTOR_STATE_STOPPED;
    motor->speed_percent = 0;
    
    // 设置PWM占空比为0
    __HAL_TIM_SET_COMPARE(PWM_TIMER_HANDLE, motor_pwm_channels[motor_id], calculate_ccr_value(0));
    
    // 更新GPIO状态
    update_motor_gpio(motor_id);
}

void Motor_SetSpeed(Motor_ID_t motor_id, uint8_t speed_percent)
{
     if (motor_system_locked) return;  // 如果系统锁定，直接返回
    Motor_Control_t* motor = get_motor_instance(motor_id);
    
    motor->speed_percent = speed_percent;//期望占空比
    
    // 设置PWM占空比
    uint32_t ccr_value = calculate_ccr_value(speed_percent);//实际占空比
    __HAL_TIM_SET_COMPARE(PWM_TIMER_HANDLE, motor_pwm_channels[motor_id], ccr_value);
    
    // 更新状态和GPIO
    if (speed_percent > 0) {
        motor->state = MOTOR_STATE_RUNNING;
    } else {
        motor->state = MOTOR_STATE_STOPPED;
    }
    update_motor_gpio(motor_id);
    //打印测试
//    update_motor_gpio(motor_id);
//    
//        if (motor_id == MOTOR_A) {
//        printf("[MOTOR_A] speed=%d, ccr=%u, state=%d, dir=%d, locked=%d\r\n",
//               speed_percent,
//               ccr_value,
//               motor->state,
//               motor->direction,
//               motor_system_locked);
//    }

}

void Motor_SetDirection(Motor_ID_t motor_id, Motor_Direction_t direction)
{
     if (motor_system_locked) return;  // 如果系统锁定，直接返回
    Motor_Control_t* motor = get_motor_instance(motor_id);
    
    motor->direction = direction;
    
    // 如果电机正在运行，立即更新方向GPIO
    if (motor->state == MOTOR_STATE_RUNNING) {
        update_motor_gpio(motor_id);
    }
}

void Motor_Reverse(Motor_ID_t motor_id)
{
     if (motor_system_locked) return;  // 如果系统锁定，直接返回
    Motor_Control_t* motor = get_motor_instance(motor_id);
    
    // 切换方向
    motor->direction = (motor->direction == DIRECTION_FORWARD) ? 
                       DIRECTION_REVERSE : DIRECTION_FORWARD;
    
    // 如果电机正在运行，立即更新方向GPIO
    if (motor->state == MOTOR_STATE_RUNNING) {
        update_motor_gpio(motor_id);
    }
}

void Motor_Start_All(void)
{
    Motor_Start(MOTOR_A);
    Motor_Start(MOTOR_B);
    Motor_Start(MOTOR_C);
    Motor_Start(MOTOR_D);
}

void Motor_Stop_All(void)
{
    Motor_Stop(MOTOR_A);
    Motor_Stop(MOTOR_B);
    Motor_Stop(MOTOR_C);
    Motor_Stop(MOTOR_D);
}

void Motor_SetSpeed_All(uint8_t speed_percent)
{
    Motor_SetSpeed(MOTOR_A, speed_percent);
    Motor_SetSpeed(MOTOR_B, speed_percent);
    Motor_SetSpeed(MOTOR_C, speed_percent);
    Motor_SetSpeed(MOTOR_D, speed_percent);
}

void Motor_Emergency_Stop(void)
{
    motor_system_locked = 1;  // 设置锁定标志
    // 立即停止所有PWM输出
    __HAL_TIM_SET_COMPARE(PWM_TIMER_HANDLE, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(PWM_TIMER_HANDLE, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(PWM_TIMER_HANDLE, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(PWM_TIMER_HANDLE, TIM_CHANNEL_4, 0);
    
    // 所有刹车通道拉低（触发刹车）
    HAL_GPIO_WritePin(A_Brake_GPIO_Port, A_Brake_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(B_Brake_GPIO_Port, B_Brake_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(C_Brake_GPIO_Port, C_Brake_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(D_Brake_GPIO_Port, D_Brake_Pin, GPIO_PIN_RESET);
    
    // 所有电源通道拉低（切断电源）
    HAL_GPIO_WritePin(POWER1_GPIO_Port, POWER1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(POWER2_GPIO_Port, POWER2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(POWER3_GPIO_Port, POWER3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(POWER4_GPIO_Port, POWER4_Pin, GPIO_PIN_RESET);
    
    // 更新所有电机状态
    motor_A.state = MOTOR_STATE_STOPPED;
    motor_B.state = MOTOR_STATE_STOPPED;
    motor_C.state = MOTOR_STATE_STOPPED;
    motor_D.state = MOTOR_STATE_STOPPED;
    motor_A.speed_percent = 0;
    motor_B.speed_percent = 0;
    motor_C.speed_percent = 0;
    motor_D.speed_percent = 0;
}

void Motor_Lock_All(void)
{
    Motor_Emergency_Stop();  // 调用已有的紧急停止函数（包含锁定标志设置）
}

void Motor_Unlock_All(void)
{
    motor_system_locked = 0;  // 清除锁定标志
    
    // 保持所有电机在停止状态（安全起见）
    Motor_Stop_All();
}

uint8_t Motor_Is_Locked(void)
{
    return motor_system_locked;
}

const char* Motor_Get_ID_String(Motor_ID_t motor_id)
{
    switch(motor_id) {
        case MOTOR_A: return "MOTOR_A";
        case MOTOR_B: return "MOTOR_B";
        case MOTOR_C: return "MOTOR_C";
        case MOTOR_D: return "MOTOR_D";
        default: return "UNKNOWN";
    }
}

const char* Motor_Get_Direction_String(Motor_Direction_t direction)
{
    return (direction == DIRECTION_FORWARD) ? "FORWARD" : "REVERSE";
}

