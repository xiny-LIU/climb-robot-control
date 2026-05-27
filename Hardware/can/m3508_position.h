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
 * 4. 本模块根据长度误差生成“目标转子转速”；
 * 5. 目标转子转速交给 PID_SetTargetSpeed()；
 * 6. PID.c 再根据目标转速与 C620 反馈 speed_rpm 计算电流；
 * 7. CAN_cmd_chassis() 最终发送的是电流指令，不是转速指令。
 *
 * 注意：
 * 本模块不直接调用 CAN_cmd_chassis()。
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
 * 默认值：
 * M3508_POS_DEFAULT_REDUCTION_RATIO
 *
 * 作用：
 * 其他文件可以读取该变量，用于统一获得当前采用的 M3508 减速比。
 * 如果后续你通过实验标定发现实际传动比存在误差，也可以通过
 * M3508_Position_SetReductionRatio() 修改该值。
 */
extern float g_m3508_pos_reduction_ratio;

/*
 * 左右 M3508 当前输出轴/摩擦轮转速，单位 rpm。
 *
 * 计算关系：
 *
 * output_rpm = rotor_rpm / reduction_ratio
 *
 * 其中：
 * rotor_rpm 为 C620 反馈的转子转速；
 * reduction_ratio 为 M3508 减速比。
 */
extern volatile float g_m3508_pos_left_output_rpm;
extern volatile float g_m3508_pos_right_output_rpm;

/*
 * 左右 M3508 目标输出轴/摩擦轮转速，单位 rpm。
 *
 * 说明：
 * 该变量不是直接发送给 C620 的值。
 * 本模块会将它乘以减速比，转换为目标转子转速，
 * 再通过 PID_SetTargetSpeed() 交给速度环。
 */
extern volatile float g_m3508_pos_left_target_output_rpm;
extern volatile float g_m3508_pos_right_target_output_rpm;


/************************************************
 * 类型定义
 ************************************************/

/*
 * @brief 伸缩电机侧别枚举
 *
 * M3508_POS_LEFT  ：左臂伸缩电机，对应 PID motor_id = 0
 * M3508_POS_RIGHT ：右臂伸缩电机，对应 PID motor_id = 1
 */
typedef enum {
    M3508_POS_LEFT = 0,
    M3508_POS_RIGHT = 1
} M3508_PositionSide_t;

/*
 * @brief 伸缩位置环运行状态枚举
 *
 * M3508_POS_OK             ：运行正常
 * M3508_POS_DISABLED       ：模块未使能
 * M3508_POS_TARGET_LIMITED ：目标长度被软限位截断
 * M3508_POS_MOTOR_ERROR    ：PID 层检测到电机故障
 */
typedef enum {
    M3508_POS_OK = 0,
    M3508_POS_DISABLED,
    M3508_POS_TARGET_LIMITED,
    M3508_POS_MOTOR_ERROR
} M3508_PositionStatus_t;

/*
 * @brief 单个伸缩电机调试信息结构体
 *
 * current_length_mm ：当前伸缩长度，单位 mm
 * target_length_mm  ：目标伸缩长度，单位 mm
 * error_mm          ：长度误差，单位 mm
 * output_rpm        ：当前输出轴/摩擦轮转速，单位 rpm
 * target_output_rpm ：目标输出轴/摩擦轮转速，单位 rpm
 * rotor_rpm_set     ：传给 PID_SetTargetSpeed() 的目标转子转速，单位 rpm
 * status            ：当前状态
 */
typedef struct {
    float current_length_mm;
    float target_length_mm;
    float error_mm;

    float output_rpm;
    float target_output_rpm;
    float rotor_rpm_set;

    M3508_PositionStatus_t status;
} M3508_PositionMotorDebug_t;

/*
 * @brief 左右两个伸缩电机的调试信息
 */
typedef struct {
    M3508_PositionMotorDebug_t left;
    M3508_PositionMotorDebug_t right;
} M3508_PositionDebug_t;


/************************************************
 * 初始化与使能
 ************************************************/

/*
 * @brief 初始化 M3508 伸缩位置闭环模块
 *
 * 功能：
 * 1. 清空内部状态；
 * 2. 设置默认减速比；
 * 3. 设置左右伸缩电机 PID 编号；
 * 4. 设置默认基准长度；
 * 5. 设置默认目标长度；
 * 6. 停止左右伸缩电机。
 *
 * 调用时机：
 * 系统初始化阶段调用一次。
 *
 * 注意：
 * 本函数不会启动电机运动，初始化后模块默认处于 disabled 状态。
 */
void M3508_Position_Init(void);

/*
 * @brief 使能或失能 M3508 伸缩位置闭环模块
 *
 * @param enable
 *        1：使能位置闭环；
 *        0：失能位置闭环，并停止左右伸缩电机。
 *
 * 使用说明：
 * 只有使能后，M3508_Position_Update_5ms() 才会真正输出目标转速。
 */
void M3508_Position_Enable(uint8_t enable);

/*
 * @brief 查询 M3508 伸缩位置闭环模块是否使能
 *
 * @return
 *        1：已使能；
 *        0：未使能。
 */
uint8_t M3508_Position_IsEnabled(void);


/************************************************
 * 目标设置与周期更新
 ************************************************/

/*
 * @brief 设置单侧伸缩目标长度
 *
 * @param side
 *        M3508_POS_LEFT  ：左伸缩；
 *        M3508_POS_RIGHT ：右伸缩。
 *
 * @param target_length_mm
 *        目标伸缩长度，单位 mm。
 *
 * 说明：
 * 该函数只保存目标长度，不立即执行控制。
 * 真正的闭环计算在 M3508_Position_Update_5ms() 中进行。
 */
void M3508_Position_SetTargetLength(M3508_PositionSide_t side, float target_length_mm);

/*
 * @brief 同时设置左右伸缩目标长度
 *
 * @param left_target_mm
 *        左伸缩目标长度，单位 mm。
 *
 * @param right_target_mm
 *        右伸缩目标长度，单位 mm。
 */
void M3508_Position_SetTargetLengthBoth(float left_target_mm, float right_target_mm);

/*
 * @brief M3508 伸缩位置环周期更新函数
 *
 * 调用周期：
 * 建议 5ms 或 10ms 调用一次。
 *
 * 功能：
 * 1. 读取当前左右电机转子转速；
 * 2. 计算当前输出轴转速；
 * 3. 根据累计编码器计算当前伸缩长度；
 * 4. 根据长度误差计算目标伸缩线速度；
 * 5. 换算成目标输出轴转速；
 * 6. 再换算成目标转子转速；
 * 7. 调用 PID_SetTargetSpeed()。
 *
 * 注意：
 * PID_Loop_1ms() 仍然需要在 1ms 定时器中运行，
 * 因为真正的速度环和电流输出在 PID.c 中完成。
 */
void M3508_Position_Update_5ms(void);


/************************************************
 * 停止与状态
 ************************************************/

/*
 * @brief 停止单侧伸缩电机
 *
 * @param side
 *        M3508_POS_LEFT  ：停止左伸缩；
 *        M3508_POS_RIGHT ：停止右伸缩。
 *
 * 功能：
 * 将对应 PID motor_id 的目标速度设置为 0。
 */
void M3508_Position_Stop(M3508_PositionSide_t side);

/*
 * @brief 停止左右两个伸缩电机
 *
 * 功能：
 * 分别调用 M3508_Position_Stop(M3508_POS_LEFT)
 * 和 M3508_Position_Stop(M3508_POS_RIGHT)。
 */
void M3508_Position_StopAll(void);

/*
 * @brief 判断单侧伸缩是否到达目标长度
 *
 * @param side   需要判断的伸缩电机侧别。
 *
 * @return
 *        1：当前位置与目标位置误差小于死区；
 *        0：尚未到达。
 */
uint8_t M3508_Position_IsTargetReached(M3508_PositionSide_t side);

/*
 * @brief 判断左右伸缩是否全部到达目标长度
 *
 * @return
 *        1：左右都到达；
 *        0：至少有一侧未到达。
 */
uint8_t M3508_Position_IsAllTargetReached(void);

/*
 * @brief 获取 M3508 伸缩位置环调试信息
 *
 * @return
 *        返回包含左右伸缩当前长度、目标长度、误差、
 *        当前输出轴转速、目标输出轴转速等信息的结构体。
 */
M3508_PositionDebug_t M3508_Position_GetDebugInfo(void);


/************************************************
 * 当前值获取
 ************************************************/

/*
 * @brief 获取单侧当前伸缩长度
 *
 * @param side
 *        M3508_POS_LEFT  ：左伸缩；
 *        M3508_POS_RIGHT ：右伸缩。
 *
 * @return
 *        当前伸缩长度，单位 mm。
 */
float M3508_Position_GetCurrentLength(M3508_PositionSide_t side);

/*
 * @brief 获取单侧目标伸缩长度
 *
 * @param side    伸缩侧别。
 *
 * @return     目标伸缩长度，单位 mm。
 */
float M3508_Position_GetTargetLength(M3508_PositionSide_t side);

/*
 * @brief 获取单侧当前输出轴/摩擦轮转速
 *
 * @param side    伸缩侧别。
 *
 * @return       当前输出轴/摩擦轮转速，单位 rpm。
 */
float M3508_Position_GetOutputRPM(M3508_PositionSide_t side);

/*
 * @brief 获取单侧目标输出轴/摩擦轮转速
 *
 * @param side    伸缩侧别。
 *
 * @return    目标输出轴/摩擦轮转速，单位 rpm。
 */
float M3508_Position_GetTargetOutputRPM(M3508_PositionSide_t side);


/************************************************
 * 参数配置
 ************************************************/

/*
 * @brief 设置 M3508 减速比
 *
 * @param ratio     电机转子到 M3508 输出轴之间的减速比。
 *
 * 默认值：        3591 / 187
 *
 * 注意：
 * 如果传入 ratio < 1.0，则自动恢复默认减速比。
 */
void M3508_Position_SetReductionRatio(float ratio);

/*
 * @brief 获取当前使用的 M3508 减速比
 *
 * @return     当前减速比。
 */
float M3508_Position_GetReductionRatio(void);

/*
 * @brief 设置单侧伸缩基准长度
 *
 * @param side     伸缩侧别。
 *
 * @param base_length_mm    编码器清零时对应的实际伸缩长度，单位 mm。
 *
 * 使用场景：
 * 如果你将伸缩杆收缩到最短长度后调用 Encoder_Counter_Reset()，
 * 那么 base_length_mm 可以设置为 550.0f。
 */
void M3508_Position_SetBaseLength(M3508_PositionSide_t side, float base_length_mm);

/*
 * @brief 同时设置左右伸缩基准长度
 *
 * @param left_base_mm      左伸缩基准长度，单位 mm。
 *
 * @param right_base_mm    右伸缩基准长度，单位 mm。
 */
void M3508_Position_SetBaseLengthBoth(float left_base_mm, float right_base_mm);

/*
 * @brief 设置单侧伸缩方向符号
 *
 * @param side       伸缩侧别。
 *
 * @param sign
 *        +1：转子正方向转动时，d3 增大；
 *        -1：转子正方向转动时，d3 减小。
 *
 * 使用场景：
 * 如果发现目标长度增大时，伸缩杆反而缩短，则将该侧 sign 改为 -1。
 */
void M3508_Position_SetDirectionSign(M3508_PositionSide_t side, int8_t sign);

/*
 * @brief 同时设置左右伸缩方向符号
 *
 * @param left_sign       左伸缩方向符号。
 *
 * @param right_sign     右伸缩方向符号。
 */
void M3508_Position_SetDirectionSignBoth(int8_t left_sign, int8_t right_sign);

/*
 * @brief 设置伸缩长度软限位
 *
 * @param min_mm      最小允许伸缩长度，单位 mm。
 *
 * @param max_mm      最大允许伸缩长度，单位 mm。
 *
 * 注意：
 * 如果 max_mm <= min_mm，函数会直接返回，不修改限位。
 */
void M3508_Position_SetLengthLimit(float min_mm, float max_mm);

#endif
