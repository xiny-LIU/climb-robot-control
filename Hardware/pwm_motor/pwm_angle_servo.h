#ifndef __PWM_ANGLE_SERVO_H
#define __PWM_ANGLE_SERVO_H

#include "stdint.h"

/*
 * ============================================================
 * pwm_angle_servo.h
 *
 * 模块作用：
 * 1. 对 4 个 PWM 电机做角度位置闭环；
 * 2. 上层只需要输入目标角度，单位 deg；
 * 3. 本模块内部读取 PQY13 编码器角度；
 * 4. 根据角度误差设置 PWM 电机方向和速度；
 * 5. 到达目标附近后自动停止对应电机。
 *
 * 关节映射：
 * 左臂俯仰：MOTOR_A，ENC_1
 * 左臂偏航：MOTOR_B，ENC_2
 * 右臂俯仰：MOTOR_C，ENC_3
 * 右臂偏航：MOTOR_D，ENC_4
 * ============================================================
 */


/* 默认角度限位，单位 deg */
#define PWM_ANGLE_DEFAULT_YAW_MIN_DEG       -6.0f
#define PWM_ANGLE_DEFAULT_YAW_MAX_DEG       22.0f
#define PWM_ANGLE_DEFAULT_PITCH_MIN_DEG     -8.0f
#define PWM_ANGLE_DEFAULT_PITCH_MAX_DEG     30.0f

/* 标准单臂局部偏航限位，单位 deg */
#define PWM_ANGLE_STD_YAW_MIN_DEG           -6.0f
#define PWM_ANGLE_STD_YAW_MAX_DEG           22.0f

/* 左臂真实 DH 偏航角限位 */
#define PWM_ANGLE_LEFT_YAW_MIN_DEG          (PWM_ANGLE_STD_YAW_MIN_DEG)
#define PWM_ANGLE_LEFT_YAW_MAX_DEG          (PWM_ANGLE_STD_YAW_MAX_DEG)

/* 右臂真实 DH 偏航角限位：关于机身中面对称 */
#define PWM_ANGLE_RIGHT_YAW_MIN_DEG         (-PWM_ANGLE_STD_YAW_MAX_DEG)
#define PWM_ANGLE_RIGHT_YAW_MAX_DEG         (-PWM_ANGLE_STD_YAW_MIN_DEG)

/* 俯仰角左右一致，单位 deg */
#define PWM_ANGLE_DEFAULT_PITCH_MIN_DEG     -8.0f
#define PWM_ANGLE_DEFAULT_PITCH_MAX_DEG     30.0f

/*
 * @brief PWM 角度闭环关节枚举
 *
 * PWM_ANGLE_LEFT_PITCH  ：左臂俯仰
 * PWM_ANGLE_LEFT_YAW    ：左臂偏航
 * PWM_ANGLE_RIGHT_PITCH ：右臂俯仰
 * PWM_ANGLE_RIGHT_YAW   ：右臂偏航
 */
typedef enum {
    PWM_ANGLE_LEFT_PITCH = 0,
    PWM_ANGLE_LEFT_YAW,
    PWM_ANGLE_RIGHT_PITCH,
    PWM_ANGLE_RIGHT_YAW,
    PWM_ANGLE_JOINT_NUM
} PWM_AngleJoint_t;

/*
 * @brief PWM 角度闭环状态枚举
 *
 * PWM_ANGLE_OK             ：运行正常
 * PWM_ANGLE_DISABLED       ：模块未使能
 * PWM_ANGLE_TARGET_LIMITED ：目标角度被软限位截断
 */
typedef enum {
    PWM_ANGLE_OK = 0,
    PWM_ANGLE_DISABLED,
    PWM_ANGLE_TARGET_LIMITED
} PWM_AngleStatus_t;

/*
 * @brief 单个 PWM 角度关节调试信息
 *
 * current_deg   ：当前角度，单位 deg
 * target_deg    ：目标角度，单位 deg
 * error_deg     ：角度误差，单位 deg
 * speed_percent ：当前 PWM 速度百分比
 * status        ：当前状态
 */
typedef struct {
    float current_deg;
    float target_deg;
    float error_deg;
    uint8_t speed_percent;
    PWM_AngleStatus_t status;
} PWM_AngleJointDebug_t;

/*
 * @brief 4 个 PWM 角度关节的调试信息
 */
typedef struct {
    PWM_AngleJointDebug_t joint[PWM_ANGLE_JOINT_NUM];
} PWM_AngleDebug_t;


/************************************************
 * 初始化与使能
 ************************************************/

/*
 * @brief 初始化 PWM 角度闭环模块
 *
 * 功能：
 * 1. 设置 4 个 PWM 电机与 4 个编码器的默认映射；
 * 2. 设置默认角度限位；
 * 3. 清空调试信息；
 * 4. 停止所有 PWM 电机。
 *
 * 注意：
 * 初始化后模块默认处于 disabled 状态。
 */
void PWM_AngleServo_Init(void);

/*
 * @brief 使能或失能 PWM 角度闭环模块
 *
 * @param enable
 *        1：使能；
 *        0：失能，并停止所有 PWM 电机。
 */
void PWM_AngleServo_Enable(uint8_t enable);

/*
 * @brief 查询 PWM 角度闭环模块是否使能
 *
 * @return
 *        1：已使能；
 *        0：未使能。
 */
uint8_t PWM_AngleServo_IsEnabled(void);


/************************************************
 * 目标设置与周期更新
 ************************************************/

/*
 * @brief 设置单个 PWM 关节目标角度
 *
 * @param joint        需要控制的关节。
 *
 * @param target_deg       目标角度，单位 deg。
 *
 * 说明：
 * 该函数只保存目标角度，不立即执行控制。
 * 真正的闭环控制在 PWM_AngleServo_Update_5ms() 中执行。
 */
void PWM_AngleServo_SetTarget(PWM_AngleJoint_t joint, float target_deg);

/*
 * @brief 同时设置 4 个 PWM 关节目标角度
 *
 * @param left_pitch_deg      左俯仰目标角度，单位 deg。
 *
 * @param left_yaw_deg        左偏航目标角度，单位 deg。
 *
 * @param right_pitch_deg     右俯仰目标角度，单位 deg。
 *
 * @param right_yaw_deg       右偏航目标角度，单位 deg。
 */
void PWM_AngleServo_SetTargetAll(float left_pitch_deg,
                                 float left_yaw_deg,
                                 float right_pitch_deg,
                                 float right_yaw_deg);

/*
 * @brief PWM 角度闭环周期更新函数
 *
 * 调用周期：
 * 建议 5ms 调用一次。
 *
 * 调用前提：
 * 应保证 Update_All_Encoders() 已经刷新过 PQY13 编码器数据。
 *
 * 功能：
 * 1. 读取当前角度；
 * 2. 计算角度误差；
 * 3. 根据误差设置电机方向；
 * 4. 根据误差大小设置速度百分比；
 * 5. 到达死区后停止对应电机。
 */
void PWM_AngleServo_Update_5ms(void);


/************************************************
 * 停止与状态
 ************************************************/

/*
 * @brief 停止单个 PWM 关节电机
 *
 * @param joint   需要停止的关节。
 */
void PWM_AngleServo_Stop(PWM_AngleJoint_t joint);

/*
 * @brief 停止所有 PWM 关节电机
 *
 * 功能：
 * 停止 MOTOR_A、MOTOR_B、MOTOR_C、MOTOR_D。
 */
void PWM_AngleServo_StopAll(void);

/*
 * @brief 判断单个 PWM 关节是否到达目标角度
 *
 * @param joint  需要判断的关节。
 *
 * @return
 *        1：角度误差小于死区；
 *        0：尚未到达。
 */
uint8_t PWM_AngleServo_IsTargetReached(PWM_AngleJoint_t joint);

/*
 * @brief 判断所有 PWM 关节是否都到达目标角度
 *
 * @return
 *        1：全部到达；
 *        0：至少有一个未到达。
 */
uint8_t PWM_AngleServo_IsAllTargetReached(void);

/*
 * @brief 获取 PWM 角度闭环调试信息
 *
 * @param out_debug  指向用于存储调试信息的结构体指针
 */
void PWM_AngleServo_GetDebugInfo(PWM_AngleDebug_t *out_debug);


/************************************************
 * 当前值获取
 ************************************************/

/*
 * @brief 获取某个 PWM 关节当前角度
 *
 * @param joint        目标关节。
 *
 * @return        当前角度，单位 deg。
 *
 * 计算关系：
 * joint_angle = Normalize180(encoder_degree - zero_offset)
 */
float PWM_AngleServo_GetCurrentAngle(PWM_AngleJoint_t joint);

/*
 * @brief 获取某个 PWM 关节目标角度
 *
 * @param joint       目标关节。
 *
 * @return       目标角度，单位 deg。
 */
float PWM_AngleServo_GetTargetAngle(PWM_AngleJoint_t joint);


/************************************************
 * 参数配置
 ************************************************/

/*
 * @brief 设置某个 PWM 关节编码器零点偏置
 *
 * @param joint        目标关节。
 *
 * @param zero_offset_deg      机械零位时编码器读数，单位 deg。
 *
 * 计算关系：
 * joint_angle = Normalize180(encoder_degree - zero_offset_deg)
 */
void PWM_AngleServo_SetZeroOffset(PWM_AngleJoint_t joint, float zero_offset_deg);

/*
 * @brief 同时设置 4 个 PWM 关节编码器零点偏置
 *
 * @param left_pitch_offset    左俯仰机械零位编码器读数。
 *
 * @param left_yaw_offset      左偏航机械零位编码器读数。
 *
 * @param right_pitch_offset   右俯仰机械零位编码器读数。
 *
 * @param right_yaw_offset     右偏航机械零位编码器读数。
 */
void PWM_AngleServo_SetZeroOffsetAll(float left_pitch_offset,
                                     float left_yaw_offset,
                                     float right_pitch_offset,
                                     float right_yaw_offset);

/*
 * @brief 设置某个 PWM 关节方向符号
 *
 * @param joint
 *        目标关节。
 *
 * @param sign
 *        +1：error > 0 时，DIRECTION_FORWARD 使角度增大；
 *        -1：error > 0 时，DIRECTION_REVERSE 使角度增大。
 *
 * 使用场景：
 * 如果发现目标角度增大时，实际角度反而减小，则将 sign 改为 -1。
 */
void PWM_AngleServo_SetDirectionSign(PWM_AngleJoint_t joint, int8_t sign);

/*
 * @brief 同时设置 4 个 PWM 关节方向符号
 *
 * @param left_pitch_sign   左俯仰方向符号。
 *
 * @param left_yaw_sign     左偏航方向符号。
 *
 * @param right_pitch_sign  右俯仰方向符号。
 *
 * @param right_yaw_sign    右偏航方向符号。
 */
void PWM_AngleServo_SetDirectionSignAll(int8_t left_pitch_sign,
                                        int8_t left_yaw_sign,
                                        int8_t right_pitch_sign,
                                        int8_t right_yaw_sign);

/*
 * @brief 设置某个 PWM 关节角度软限位
 *
 * @param joint        目标关节。
 *
 * @param min_deg        最小允许角度，单位 deg。
 *
 * @param max_deg       最大允许角度，单位 deg。
 *
 * 注意：
 * 如果 max_deg <= min_deg，函数会直接返回，不修改限位。
 */
void PWM_AngleServo_SetLimit(PWM_AngleJoint_t joint, float min_deg, float max_deg);

/************************************************
 * 当前角度设为零点功能
 ************************************************/

/*
 * @brief 将指定关节的当前角度设为临时零点
 *
 * @param joint
 *        目标关节。
 *
 * 功能：
 * 1. 读取该关节当前绝对式编码器角度；
 * 2. 将该角度设置为 zero_offset；
 * 3. 将该关节当前目标角度设置为 0°；
 * 4. 停止对应电机，避免设零点后突然运动。
 *
 * 使用场景：
 * 绝对式编码器上电读数一般不为 0。
 * 测试时可以调用本函数，把当前姿态临时定义为 0°。
 */
void PWM_AngleServo_SetCurrentAsZero(PWM_AngleJoint_t joint);


/*
 * @brief 将 4 个 PWM 关节的当前角度全部设为临时零点
 *
 * 功能：
 * 分别对左俯仰、左偏航、右俯仰、右偏航执行
 * PWM_AngleServo_SetCurrentAsZero()。
 */
void PWM_AngleServo_SetAllCurrentAsZero(void);

#endif
