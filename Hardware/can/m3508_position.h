#ifndef __M3508_POSITION_H
#define __M3508_POSITION_H

#include "stdint.h"

/*
 * ============================================================
 * m3508_position.h
 *
 * 模块作用：
 * 1. 对左右两个 M3508 伸缩电机做“长度位置闭环”；
 * 2. 上层只需要输入目标伸缩长度，单位 mm；
 * 3. 本模块根据 encoder_counter 的累计转子角度计算当前伸缩长度；
 * 4. 本模块根据长度误差生成连续平滑的“目标转子转速”；
 * 5. 目标转子转速交给 PID_SetTargetSpeed()；
 * 6. PID.c 再根据目标转速与 C620 反馈 speed_rpm 计算电流；
 * 7. CAN_cmd_chassis() 最终发送的是电流指令，不是转速指令。
 * ============================================================
 */

/* ============================================================
 * 默认物理参数
 * ============================================================ */
/* M3508 官方减速比：i = 3591 / 187 ≈ 19.203（官方手册提供） */
#define M3508_POS_DEFAULT_REDUCTION_RATIO       (3591.0f / 187.0f)

/* 摩擦轮直径，单位 mm */
#define M3508_POS_PULLEY_DIAMETER_MM            38.0f

/* 默认伸缩长度范围，单位 mm */
#define M3508_POS_DEFAULT_D3_MIN_MM             560.0f
#define M3508_POS_DEFAULT_D3_MAX_MM             2560.0f

/************************************************
 * 全局变量
 ************************************************/
extern float g_m3508_pos_reduction_ratio;

extern volatile float g_m3508_pos_left_output_rpm;
extern volatile float g_m3508_pos_right_output_rpm;

extern volatile float g_m3508_pos_left_target_output_rpm;
extern volatile float g_m3508_pos_right_target_output_rpm;

/************************************************
 * 类型定义
 ************************************************/
/* @brief 伸缩电机侧别枚举 */
typedef enum {
    M3508_POS_LEFT = 0,
    M3508_POS_RIGHT = 1
} M3508_PositionSide_t;

/* @brief 伸缩位置环运行状态枚举 */
typedef enum {
    M3508_POS_OK = 0,
    M3508_POS_DISABLED,
    M3508_POS_MOTOR_LOADED,
    M3508_POS_MOTOR_OBSTACLE,
    M3508_POS_TARGET_LIMITED, // 目标值超出软件安全限位，已被自动截断
    M3508_POS_MOTOR_ERROR     // PID 底层报错，电机故障关断
} M3508_PositionStatus_t;

/* @brief 单个伸缩电机调试信息结构体 */
typedef struct {
    float current_length_mm;
    float target_length_mm;
    float error_mm;

    float output_rpm;
    float target_output_rpm;
    float rotor_rpm_set;

    M3508_PositionStatus_t status;
} M3508_PositionMotorDebug_t;

/* @brief 左右两个伸缩电机的整体调试状态快照 */
typedef struct {
    M3508_PositionMotorDebug_t left;
    M3508_PositionMotorDebug_t right;
} M3508_PositionDebug_t;

/************************************************
 * 初始化与使能
 ************************************************/
/* @brief 初始化 M3508 伸缩位置闭环模块 (开机自动同步当前长度，防暴走) */
void M3508_Position_Init(void);

/* @brief 使能或失能 M3508 伸缩位置闭环模块 */
void M3508_Position_Enable(uint8_t enable);

/* @brief 查询 M3508 伸缩位置闭环模块是否使能 */
uint8_t M3508_Position_IsEnabled(void);

/************************************************
 * 目标设置与周期更新
 ************************************************/
/* @brief 设置单侧伸缩目标长度（线程安全接口） */
void M3508_Position_SetTargetLength(M3508_PositionSide_t side, float target_length_mm);

/* @brief 同时设置左右伸缩目标长度 */
void M3508_Position_SetTargetLengthBoth(float left_target_mm, float right_target_mm);

/* Reset encoder zero and lock the position loop target to the new current length. */
void M3508_Position_ResetEncoderAndSyncTarget(M3508_PositionSide_t side);
void M3508_Position_ResetEncoderAndSyncTargetBoth(void);
void M3508_Position_SyncTargetToCurrent(M3508_PositionSide_t side);
void M3508_Position_SyncTargetToCurrentBoth(void);

/* @brief M3508 伸缩位置环周期更新函数 (建议在 5ms 或 10ms 定时中断中调用) */
void M3508_Position_Update_5ms(void);

/************************************************
 * 停止与状态查询
 ************************************************/
/* @brief 停止单侧伸缩电机 */
void M3508_Position_Stop(M3508_PositionSide_t side);

/* @brief 停止左右两个伸缩电机 */
void M3508_Position_StopAll(void);

/* @brief 判断单侧伸缩是否到达目标长度 */
uint8_t M3508_Position_IsTargetReached(M3508_PositionSide_t side);

/* @brief 判断左右伸缩是否全部到达目标长度 */
uint8_t M3508_Position_IsAllTargetReached(void);

/* @brief 安全获取 M3508 伸缩位置环调试信息结构体 (内置临界区保护，杜绝撕裂) */
void M3508_Position_GetDebugInfo(M3508_PositionDebug_t *out_debug);

/************************************************
 * 数据与动态参数配置接口
 ************************************************/
float M3508_Position_GetCurrentLength(M3508_PositionSide_t side);
float M3508_Position_GetTargetLength(M3508_PositionSide_t side);
float M3508_Position_GetOutputRPM(M3508_PositionSide_t side);
float M3508_Position_GetTargetOutputRPM(M3508_PositionSide_t side);

void M3508_Position_SetReductionRatio(float ratio);
float M3508_Position_GetReductionRatio(void);

void M3508_Position_SetBaseLength(M3508_PositionSide_t side, float base_length_mm);
void M3508_Position_SetBaseLengthBoth(float left_base_mm, float right_base_mm);

void M3508_Position_SetDirectionSign(M3508_PositionSide_t side, int8_t sign);
void M3508_Position_SetDirectionSignBoth(int8_t left_sign, int8_t right_sign);

void M3508_Position_SetLengthLimit(float min_mm, float max_mm);

#endif /* __M3508_POSITION_H */

