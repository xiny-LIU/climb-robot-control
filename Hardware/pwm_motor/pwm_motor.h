#ifndef __PWM_MOTOR_H
#define __PWM_MOTOR_H

#include "main.h"

// 电机编号定义
typedef enum {
    MOTOR_A = 0,
    MOTOR_B = 1,
    MOTOR_C = 2,
    MOTOR_D = 3
} Motor_ID_t;

// 电机方向定义
typedef enum {
    DIRECTION_FORWARD = 0,  // 正转 (Low电平)
    DIRECTION_REVERSE = 1   // 反转 (High电平)
} Motor_Direction_t;

// 电机状态定义
typedef enum {
    MOTOR_STATE_STOPPED = 0,
    MOTOR_STATE_RUNNING = 1
} Motor_State_t;

// 电机控制结构体
typedef struct {
    Motor_ID_t id;
    Motor_State_t state;
    Motor_Direction_t direction;
    uint8_t speed_percent;  // 0-100%
} Motor_Control_t;

// 全局电机控制实例
extern Motor_Control_t motor_A;
extern Motor_Control_t motor_B;
extern Motor_Control_t motor_C;
extern Motor_Control_t motor_D;

// 初始化函数
void Motor_Init(Motor_ID_t motor_id);
void Motor_Init_All(void);

// 基本控制函数
void Motor_Start(Motor_ID_t motor_id);
void Motor_Stop(Motor_ID_t motor_id);
void Motor_SoftStop(Motor_ID_t motor_id);
void Motor_SetSpeed(Motor_ID_t motor_id, uint8_t speed_percent);
void Motor_SetDirection(Motor_ID_t motor_id, Motor_Direction_t direction);
void Motor_Reverse(Motor_ID_t motor_id);

// 批量控制函数
void Motor_Start_All(void);
void Motor_Stop_All(void);
void Motor_SoftStop_All(void);
void Motor_SetSpeed_All(uint8_t speed_percent);
void Motor_Emergency_Stop(void);

// 辅助函数
const char* Motor_Get_ID_String(Motor_ID_t motor_id);
const char* Motor_Get_Direction_String(Motor_Direction_t direction);

// 电机锁定控制（用于PS2紧急停止）
void Motor_Lock_All(void);      // 锁定所有电机（断电+刹车）
void Motor_Unlock_All(void);    // 解锁所有电机
uint8_t Motor_Is_Locked(void);  // 查询锁定状态

#endif

