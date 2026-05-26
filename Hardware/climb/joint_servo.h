#ifndef __JOINT_SERVO_H
#define __JOINT_SERVO_H

#include "stdint.h"
#include "climb_control.h"

/*
 * ============================================================
 * joint_servo.h
 *
 * 作用：
 * 1. 将上层给出的 JointAngle_t 目标关节量转换为实际电机控制命令；
 * 2. 对 4 个 PWM 关节电机实现角度位置环；
 * 3. 对 2 个 M3508 伸缩电机实现长度位置环；
 * 4. 让 climb_control.c 不直接接触 Motor_SetSpeed / PID_SetTargetSpeed 等底层接口。
 *
 * 关节映射：
 * 左臂伸缩：PID motor_id = 0，对应 CAN_3508_M1_ID
 * 右臂伸缩：PID motor_id = 1，对应 CAN_3508_M2_ID
 *
 * 左臂俯仰：MOTOR_A，ENC_1
 * 左臂偏航：MOTOR_B，ENC_2
 * 右臂俯仰：MOTOR_C，ENC_3
 * 右臂偏航：MOTOR_D，ENC_4
 * ============================================================
 */


/* ============================================================
 * 全局参数与全局速度变量
 * ============================================================ */

/*
 * M3508 官方减速比：
 *
 * i = 3591 / 187 ≈ 19.203
 *
 * 含义：
 * 电机转子转 i 圈，M3508 输出轴转 1 圈。
 * 你的摩擦轮与 M3508 输出轴同步转动，因此：
 * 摩擦轮圈数 = 转子圈数 / i
 */
#define JOINT_SERVO_DEFAULT_M3508_REDUCTION_RATIO    (3591.0f / 187.0f)

/*
 * 全局减速比变量。
 *
 * 默认值为 3591/187。
 * 如果后续你通过实验标定发现实际传动比存在误差，可以在运行时修改该变量，
 * 或调用 JointServo_SetReductionRatio() 修改。
 */
extern float g_joint_servo_m3508_reduction_ratio;

/*
 * 当前 M3508 输出轴转速，单位 rpm。
 *
 * 由 C620 反馈的转子转速 speed_rpm 除以减速比得到：
 *
 * output_rpm = rotor_rpm / reduction_ratio
 *
 * 注意：
 * 这里是“电机输出轴/摩擦轮转速”，不是伸缩杆线速度。
 */
extern volatile float g_joint_servo_left_output_rpm;
extern volatile float g_joint_servo_right_output_rpm;

/*
 * 目标 M3508 输出轴转速，单位 rpm。
 *
 * 这是 joint_servo 根据伸缩长度误差计算出来的输出轴目标转速。
 * 发送给 PID_SetTargetSpeed() 前，会再乘以减速比变成转子目标转速。
 */
extern volatile float g_joint_servo_left_target_output_rpm;
extern volatile float g_joint_servo_right_target_output_rpm;


/* ============================================================
 * 类型定义
 * ============================================================ */

typedef enum {
    JOINT_SERVO_LEFT = 0,
    JOINT_SERVO_RIGHT = 1
} JointServoSide_t;

typedef enum {
    JOINT_SERVO_OK = 0,
    JOINT_SERVO_DISABLED,
    JOINT_SERVO_TARGET_LIMITED,
    JOINT_SERVO_MOTOR_ERROR
} JointServoStatus_t;

typedef struct {
    JointAngle_t current_left;
    JointAngle_t current_right;

    JointAngle_t target_left;
    JointAngle_t target_right;

    float err_left_yaw;
    float err_left_pitch;
    float err_left_d3;

    float err_right_yaw;
    float err_right_pitch;
    float err_right_d3;

    float left_output_rpm;
    float right_output_rpm;

    float left_target_output_rpm;
    float right_target_output_rpm;

    JointServoStatus_t status;
} JointServoDebug_t;


/* ============================================================
 * 对外接口
 * ============================================================ */

/* 初始化 joint_servo 内部状态 */
void JointServo_Init(void);

/*
 * 设置左右臂目标关节量。
 * 只保存目标，不立即执行。
 */
void JointServo_SetTarget(JointAngle_t left_target, JointAngle_t right_target);

/*
 * 立即设置目标并执行一次控制。
 * 后续可以用它替代 climb_control.c 中原来的 Execute_Joint_Commands()。
 */
void JointServo_ExecuteTarget(JointAngle_t left_target, JointAngle_t right_target);

/*
 * 5ms 周期控制函数。
 * 建议由 Climb_Control_Loop_5ms() 内部调用。
 */
void JointServo_Update_5ms(void);

/* 停止所有关节，但不锁死系统 */
void JointServo_StopAll(void);

/* 紧急停止：PWM 电机急停，伸缩电机目标速度置 0 */
void JointServo_EmergencyStop(void);

/* 使能/失能 joint_servo 控制 */
void JointServo_Enable(uint8_t enable);
uint8_t JointServo_IsEnabled(void);

/* 获取当前关节状态 */
JointAngle_t JointServo_GetCurrentLeft(void);
JointAngle_t JointServo_GetCurrentRight(void);

/* 判断左右臂是否都到达目标 */
uint8_t JointServo_IsTargetReached(void);

/* 获取调试信息 */
JointServoDebug_t JointServo_GetDebugInfo(void);


/* ============================================================
 * 标定与配置接口
 * ============================================================ */

/*
 * 设置 M3508 减速比。
 *
 * 参数：
 * ratio：电机转子到 M3508 输出轴之间的减速比。
 *
 * 默认值：
 * 3591 / 187
 */
void JointServo_SetReductionRatio(float ratio);
float JointServo_GetReductionRatio(void);

/*
 * 获取当前电机输出轴转速。
 *
 * 返回值：
 * 单位 rpm。
 */
float JointServo_GetLeftOutputRPM(void);
float JointServo_GetRightOutputRPM(void);

/*
 * 获取当前目标输出轴转速。
 *
 * 返回值：
 * 单位 rpm。
 */
float JointServo_GetLeftTargetOutputRPM(void);
float JointServo_GetRightTargetOutputRPM(void);

/*
 * 设置角度编码器零点偏置。
 *
 * 如果某个关节机械零位时编码器读数不是 0 度，则设置对应 offset。
 * 内部使用：
 *
 * joint_angle = Normalize180(encoder_degree - offset_degree)
 *
 * 参数：
 * left_pitch_offset_deg  ：左臂俯仰机械零位对应的编码器角度
 * left_yaw_offset_deg    ：左臂偏航机械零位对应的编码器角度
 * right_pitch_offset_deg ：右臂俯仰机械零位对应的编码器角度
 * right_yaw_offset_deg   ：右臂偏航机械零位对应的编码器角度
 */
void JointServo_SetAngleZeroOffset(float left_pitch_offset_deg,
                                   float left_yaw_offset_deg,
                                   float right_pitch_offset_deg,
                                   float right_yaw_offset_deg);

/*
 * 设置 PWM 电机方向符号。
 *
 * 默认 +1 表示：
 * error > 0 时，DIRECTION_FORWARD 使编码器角度增大。
 *
 * 如果发现某个关节方向反了，把对应 sign 设置为 -1。
 */
void JointServo_SetAngleDirectionSign(int8_t left_pitch_sign,
                                      int8_t left_yaw_sign,
                                      int8_t right_pitch_sign,
                                      int8_t right_yaw_sign);

/*
 * 设置伸缩杆方向符号。
 *
 * 默认 +1 表示：
 * C620 转子正方向转动时，d3 增大。
 *
 * 如果发现伸缩方向反了，把对应 sign 设置为 -1。
 */
void JointServo_SetExtensionDirectionSign(int8_t left_ext_sign,
                                          int8_t right_ext_sign);

/*
 * 设置伸缩杆基准长度。
 *
 * 如果你在机械臂收缩到最短 D3_MIN_LENGTH 后调用 Encoder_Counter_Reset()，
 * 那么这里可以使用默认的 D3_MIN_LENGTH。
 */
void JointServo_SetExtensionBaseLength(float left_base_mm,
                                       float right_base_mm);

#endif
