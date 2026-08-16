#include "PID.h"
#include "CAN_receive.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>

/* ========== PID参数配置（根据你的电机调整） ========== */
#define SPEED_PID_KP        2.0f        // 速度环比例增益（响应强度）2 0.1 0.2
#define SPEED_PID_KI        0.1f        // 速度环积分增益（消除静差）
#define SPEED_PID_KD        0.2f        // 速度环微分增益（抑制超调）
#define SPEED_INTEGRAL_LIMIT 3000.0f    // 速度环积分限幅（单位：mA）
#define OUTPUT_CURRENT_LIMIT 10000      // 输出电流限幅（±10A）

/* 电流监测阈值（单位：mA） */
#define CURRENT_THRESHOLD_LOADED    3000    // >3A判定为带载
#define CURRENT_THRESHOLD_OBSTACLE  8000    // >8A判定为遇障
#define CURRENT_THRESHOLD_MAX       12000   // >12A判定为过流故障

/* ========== 私有结构体 ========== */
typedef struct {
    int16_t target_speed;       // 目标转速（RPM）
    int16_t last_speed;         // 上一次的转速（用于微分计算）
    float   integral;           // 速度环积分项
    float   last_error;         // 上一次的误差
    int16_t output_current;     // PID输出的电流值
    MotorStatus_t status;       // 电机状态
    int16_t actual_current;     // 电调反馈的实际电流
} MotorController_t;

static MotorController_t motor_ctrl[2];

/* ========== 公共函数 ========== */

void PID_Init(void)
{
    memset(motor_ctrl, 0, sizeof(motor_ctrl));
    // 初始化状态为NORMAL
    motor_ctrl[0].status = MOTOR_STATUS_NORMAL;
    motor_ctrl[1].status = MOTOR_STATUS_NORMAL;
}

void PID_SetTargetSpeed(uint8_t motor_id, int16_t speed_rpm)
{
    if (motor_id > 1) return;
    motor_ctrl[motor_id].target_speed = speed_rpm;
}

void PID_Loop_1ms(void)
{
    static uint16_t startup_timer = 0;
    int16_t output[2] = {0, 0};

    // ========== 上电后50ms软启动期（等待CAN数据稳定）==========
    if (startup_timer < 50) {
        startup_timer++;
        CAN_cmd_chassis(0, 0, 0, 0);  // 持续发送零电流
        return;
    }

    // ========== 正常PID循环 ==========
    for (uint8_t i = 0; i < 2; i++) {
        // 1. 读取当前转速和实际电流
        const motor_measure_t *motor = get_chassis_motor_measure_point(i);
        int16_t current_speed = motor->speed_rpm;          // 当前实际转速
        motor_ctrl[i].actual_current = motor->given_current; // 电调反馈的实际电流

        // 2. 计算速度误差：目标 - 实际
        float error = (float)motor_ctrl[i].target_speed - (float)current_speed;

        // 3. 积分项误差累积
        motor_ctrl[i].integral += error * SPEED_PID_KI;
        
        // 积分限幅
        if (motor_ctrl[i].integral > SPEED_INTEGRAL_LIMIT) 
            motor_ctrl[i].integral = SPEED_INTEGRAL_LIMIT;
        if (motor_ctrl[i].integral < -SPEED_INTEGRAL_LIMIT) 
            motor_ctrl[i].integral = -SPEED_INTEGRAL_LIMIT;

        // 微分项（速度变化率）
        float derivative = (error - motor_ctrl[i].last_error) * SPEED_PID_KD;
        motor_ctrl[i].last_error = error;

        // PID输出（电流值）
        float pid_output = error * SPEED_PID_KP + motor_ctrl[i].integral + derivative;
        
        // 输出限幅
        if (pid_output > OUTPUT_CURRENT_LIMIT) pid_output = OUTPUT_CURRENT_LIMIT;
        if (pid_output < -OUTPUT_CURRENT_LIMIT) pid_output = -OUTPUT_CURRENT_LIMIT;
        
        output[i] = (int16_t)pid_output;
        motor_ctrl[i].output_current = output[i];  // 记录输出值

        // 4. 更新电机状态（基于实际电流）
        int16_t abs_current = abs(motor_ctrl[i].actual_current);
        if (abs_current > CURRENT_THRESHOLD_MAX) {
            motor_ctrl[i].status = MOTOR_STATUS_ERROR;
        } else if (abs_current > CURRENT_THRESHOLD_OBSTACLE) {
            motor_ctrl[i].status = MOTOR_STATUS_OBSTACLE;
        } else if (abs_current > CURRENT_THRESHOLD_LOADED) {
            motor_ctrl[i].status = MOTOR_STATUS_LOADED;
        } else {
            motor_ctrl[i].status = MOTOR_STATUS_NORMAL;
        }
    }

    // 5. 发送CAN指令
    CAN_cmd_chassis(output[0], output[1], 0, 0);
}

int16_t PID_GetCurrent(uint8_t motor_id)
{
    if (motor_id > 1) return 0;
    return motor_ctrl[motor_id].actual_current;
}

MotorStatus_t PID_GetMotorStatus(uint8_t motor_id)
{
    if (motor_id > 1) return MOTOR_STATUS_ERROR;
    return motor_ctrl[motor_id].status;
}
