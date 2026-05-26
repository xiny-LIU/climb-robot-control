#ifndef __PWM_ANGLE_SERVO_H
#define __PWM_ANGLE_SERVO_H

#include "stdint.h"

/*
 * ============================================================
 * pwm_angle_servo.h
 *
 * 作用：
 * 1. 对 4 个 PWM 电机做角度位置闭环；
 * 2. 输入目标角度 deg；
 * 3. 内部读取 PQY13 编码器角度；
 * 4. 根据角度误差设置 PWM 电机方向和速度。
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


typedef enum {
    PWM_ANGLE_LEFT_PITCH = 0,
    PWM_ANGLE_LEFT_YAW,
    PWM_ANGLE_RIGHT_PITCH,
    PWM_ANGLE_RIGHT_YAW,
    PWM_ANGLE_JOINT_NUM
} PWM_AngleJoint_t;

typedef enum {
    PWM_ANGLE_OK = 0,
    PWM_ANGLE_DISABLED,
    PWM_ANGLE_TARGET_LIMITED
} PWM_AngleStatus_t;

typedef struct {
    float current_deg;
    float target_deg;
    float error_deg;
    uint8_t speed_percent;
    PWM_AngleStatus_t status;
} PWM_AngleJointDebug_t;

typedef struct {
    PWM_AngleJointDebug_t joint[PWM_ANGLE_JOINT_NUM];
} PWM_AngleDebug_t;


/************************************************
 * 初始化与使能
 ************************************************/

void PWM_AngleServo_Init(void);
void PWM_AngleServo_Enable(uint8_t enable);
uint8_t PWM_AngleServo_IsEnabled(void);


/************************************************
 * 目标设置与周期更新
 ************************************************/

void PWM_AngleServo_SetTarget(PWM_AngleJoint_t joint, float target_deg);

void PWM_AngleServo_SetTargetAll(float left_pitch_deg,
                                 float left_yaw_deg,
                                 float right_pitch_deg,
                                 float right_yaw_deg);

/*
 * 5ms 角度闭环更新函数。
 *
 * 调用前应保证 Update_All_Encoders() 已经刷新过编码器。
 */
void PWM_AngleServo_Update_5ms(void);


/************************************************
 * 停止与状态
 ************************************************/

void PWM_AngleServo_Stop(PWM_AngleJoint_t joint);
void PWM_AngleServo_StopAll(void);

uint8_t PWM_AngleServo_IsTargetReached(PWM_AngleJoint_t joint);
uint8_t PWM_AngleServo_IsAllTargetReached(void);

PWM_AngleDebug_t PWM_AngleServo_GetDebugInfo(void);


/************************************************
 * 当前值获取
 ************************************************/

float PWM_AngleServo_GetCurrentAngle(PWM_AngleJoint_t joint);
float PWM_AngleServo_GetTargetAngle(PWM_AngleJoint_t joint);


/************************************************
 * 参数配置
 ************************************************/

/*
 * 设置机械零点偏置。
 *
 * joint_angle = Normalize180(encoder_degree - zero_offset)
 */
void PWM_AngleServo_SetZeroOffset(PWM_AngleJoint_t joint, float zero_offset_deg);

void PWM_AngleServo_SetZeroOffsetAll(float left_pitch_offset,
                                     float left_yaw_offset,
                                     float right_pitch_offset,
                                     float right_yaw_offset);

/*
 * 设置方向符号。
 *
 * 默认 +1：
 * error > 0 时，DIRECTION_FORWARD 使角度增大。
 *
 * 如果方向相反，设置为 -1。
 */
void PWM_AngleServo_SetDirectionSign(PWM_AngleJoint_t joint, int8_t sign);

void PWM_AngleServo_SetDirectionSignAll(int8_t left_pitch_sign,
                                        int8_t left_yaw_sign,
                                        int8_t right_pitch_sign,
                                        int8_t right_yaw_sign);

/* 设置某个关节角度软限位 */
void PWM_AngleServo_SetLimit(PWM_AngleJoint_t joint, float min_deg, float max_deg);

#endif
