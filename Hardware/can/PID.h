#ifndef _PID_H
#define _PID_H

#include "stdint.h"

/* 电机状态枚举 */
typedef enum {
    MOTOR_STATUS_NORMAL = 0,    // 空载/正常运行
    MOTOR_STATUS_LOADED,        // 带载（抓取物体）
    MOTOR_STATUS_OBSTACLE,      // 遇到障碍/堵转
    MOTOR_STATUS_ERROR          // 过流/过热故障
} MotorStatus_t;

/**
 * @brief 初始化PID控制器（在系统启动时调用一次）
 */
void PID_Init(void);

/**
 * @brief 设置伸缩电机目标转子转速
 * @param motor_id: 电机编号，0=左臂伸缩电机，1=右臂伸缩电机
 * @param speed_rpm: 目标转子转速，单位 rpm。
 *                   该值应与 C620 CAN 反馈的 motor->speed_rpm 同单位
 */
void PID_SetTargetSpeed(uint8_t motor_id, int16_t speed_rpm);

/**
 * @brief 1ms周期调用，执行速度环PID并发送CAN指令
 * @note 必须在TIM中断中调用，保证硬实时性
 */
void PID_Loop_1ms(void);

/**
 * @brief 获取指定电机的实时电流值
 * @param motor_id: 电机编号（0或1）
 * @return 电流值（单位：mA，范围-20000~20000）
 */
int16_t PID_GetCurrent(uint8_t motor_id);

/**
 * @brief 获取电机运行状态（基于电流判断）
 * @param motor_id: 电机编号（0或1）
 * @return 电机状态枚举值
 */
MotorStatus_t PID_GetMotorStatus(uint8_t motor_id);

#endif
