#ifndef __M3508_POSITION_H
#define __M3508_POSITION_H

#include "stdint.h"

/*
 * ============================================================
 * m3508_position.h
 *
 * 作用：
 * 1. 对左右两个 M3508 伸缩电机做长度位置闭环；
 * 2. 输入目标伸缩长度 mm；
 * 3. 内部根据 Encoder_Counter 累计角度计算当前长度；
 * 4. 根据长度误差生成转子目标转速；
 * 5. 调用 PID_SetTargetSpeed()，再由 PID.c 计算电流指令。
 *
 * 注意：
 * 本文件不直接调用 CAN_cmd_chassis()。
 * CAN_cmd_chassis() 仍由 PID_Loop_1ms() 内部调用。
 * ============================================================
 */


/* ============================================================
 * 默认物理参数
 * ============================================================ */

/*
 * M3508 官方减速比：
 *
 * i = 3591 / 187 ≈ 19.203
 *
 * 含义：
 * 电机转子转 i 圈，M3508 输出轴转 1 圈。
 */
#define M3508_POS_DEFAULT_REDUCTION_RATIO       (3591.0f / 187.0f)

/* 摩擦轮直径，单位 mm */
#define M3508_POS_PULLEY_DIAMETER_MM            38.0f

/* 默认伸缩长度范围，单位 mm */
#define M3508_POS_DEFAULT_D3_MIN_MM             550.0f
#define M3508_POS_DEFAULT_D3_MAX_MM             2000.0f


/************************************************
 * 全局变量
 ************************************************/

/*
 * 全局减速比变量。
 *
 * 默认值为 M3508_POS_DEFAULT_REDUCTION_RATIO。
 * 其他文件可直接读取，也可以通过 M3508_Position_SetReductionRatio() 修改。
 */
extern float g_m3508_pos_reduction_ratio;

/*
 * 当前 M3508 输出轴/摩擦轮转速，单位 rpm。
 *
 * 计算方式：
 * output_rpm = rotor_rpm / reduction_ratio
 */
extern volatile float g_m3508_pos_left_output_rpm;
extern volatile float g_m3508_pos_right_output_rpm;

/*
 * 目标 M3508 输出轴/摩擦轮转速，单位 rpm。
 *
 * 由长度位置环根据长度误差计算得到。
 */
extern volatile float g_m3508_pos_left_target_output_rpm;
extern volatile float g_m3508_pos_right_target_output_rpm;


/************************************************
 * 类型定义
 ************************************************/

typedef enum {
    M3508_POS_LEFT = 0,
    M3508_POS_RIGHT = 1
} M3508_PositionSide_t;

typedef enum {
    M3508_POS_OK = 0,
    M3508_POS_DISABLED,
    M3508_POS_TARGET_LIMITED,
    M3508_POS_MOTOR_ERROR
} M3508_PositionStatus_t;

typedef struct {
    float current_length_mm;
    float target_length_mm;
    float error_mm;

    float output_rpm;
    float target_output_rpm;
    float rotor_rpm_set;

    M3508_PositionStatus_t status;
} M3508_PositionMotorDebug_t;

typedef struct {
    M3508_PositionMotorDebug_t left;
    M3508_PositionMotorDebug_t right;
} M3508_PositionDebug_t;


/************************************************
 * 初始化与使能
 ************************************************/

void M3508_Position_Init(void);
void M3508_Position_Enable(uint8_t enable);
uint8_t M3508_Position_IsEnabled(void);


/************************************************
 * 目标设置与周期更新
 ************************************************/

/* 设置单侧伸缩目标长度，单位 mm */
void M3508_Position_SetTargetLength(M3508_PositionSide_t side, float target_length_mm);

/* 同时设置左右伸缩目标长度，单位 mm */
void M3508_Position_SetTargetLengthBoth(float left_target_mm, float right_target_mm);

/*
 * 5ms 外层位置环更新函数。
 *
 * 建议调用周期：
 * 5ms 或 10ms。
 *
 * 注意：
 * PID_Loop_1ms() 仍然需要在 1ms 中断中运行。
 */
void M3508_Position_Update_5ms(void);


/************************************************
 * 停止与状态
 ************************************************/

void M3508_Position_Stop(M3508_PositionSide_t side);
void M3508_Position_StopAll(void);

uint8_t M3508_Position_IsTargetReached(M3508_PositionSide_t side);
uint8_t M3508_Position_IsAllTargetReached(void);

M3508_PositionDebug_t M3508_Position_GetDebugInfo(void);


/************************************************
 * 当前值获取
 ************************************************/

float M3508_Position_GetCurrentLength(M3508_PositionSide_t side);
float M3508_Position_GetTargetLength(M3508_PositionSide_t side);

float M3508_Position_GetOutputRPM(M3508_PositionSide_t side);
float M3508_Position_GetTargetOutputRPM(M3508_PositionSide_t side);


/************************************************
 * 参数配置
 ************************************************/

/* 设置 M3508 减速比，默认 3591/187 */
void M3508_Position_SetReductionRatio(float ratio);
float M3508_Position_GetReductionRatio(void);

/*
 * 设置伸缩杆基准长度。
 *
 * 例如：
 * 如果机械臂收缩到最短长度后调用 Encoder_Counter_Reset()，
 * 则 base_length_mm 可设为 550.0f。
 */
void M3508_Position_SetBaseLength(M3508_PositionSide_t side, float base_length_mm);
void M3508_Position_SetBaseLengthBoth(float left_base_mm, float right_base_mm);

/*
 * 设置伸缩方向符号。
 *
 * 默认 +1：
 * 转子正方向转动时，伸缩长度 d3 增大。
 *
 * 如果实际相反，将 sign 设置为 -1。
 */
void M3508_Position_SetDirectionSign(M3508_PositionSide_t side, int8_t sign);
void M3508_Position_SetDirectionSignBoth(int8_t left_sign, int8_t right_sign);

/* 设置伸缩长度软限位 */
void M3508_Position_SetLengthLimit(float min_mm, float max_mm);

#endif
