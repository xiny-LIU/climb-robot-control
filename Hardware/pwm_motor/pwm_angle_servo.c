#include "pwm_angle_servo.h"

#include "math.h"
#include "pwm_motor.h"
#include "spi4.h"


/* ============================================================
 * 角度位置环参数
 * ============================================================ */

/*
 * PWM 角度位置环：
 *
 * u_pwm = Kp * |error|
 *
 * 其中：
 * u_pwm：PWM 速度百分比；
 * Kp：比例系数；
 * error：角度误差，单位 deg。
 */
#define PWM_ANGLE_KP_PERCENT_PER_DEG     5.0f

/* 角度到位死区，单位 deg */
#define PWM_ANGLE_DEADBAND_DEG           0.8f

/* PWM 电机最小/最大速度百分比 */
#define PWM_ANGLE_MIN_SPEED_PERCENT      18u
#define PWM_ANGLE_MAX_SPEED_PERCENT      70u


/* ============================================================
 * 内部类型
 * ============================================================ */

typedef struct {
    Motor_ID_t motor_id;
    Encoder_ID_t encoder_id;

    float zero_offset_deg;
    int8_t direction_sign;

    float target_deg;
    float min_deg;
    float max_deg;
} PWM_AngleJointConfig_t;


/* ============================================================
 * 内部变量
 * ============================================================ */

static PWM_AngleJointConfig_t g_pwm_angle_joint[PWM_ANGLE_JOINT_NUM] = {
    /* 左臂俯仰：MOTOR_A + ENC_1 */
    {MOTOR_A, ENC_1, 0.0f, +1, 0.0f,
     PWM_ANGLE_DEFAULT_PITCH_MIN_DEG,
     PWM_ANGLE_DEFAULT_PITCH_MAX_DEG},

    /* 左臂偏航：MOTOR_B + ENC_2 */
    {MOTOR_B, ENC_2, 0.0f, +1, 0.0f,
     PWM_ANGLE_DEFAULT_YAW_MIN_DEG,
     PWM_ANGLE_DEFAULT_YAW_MAX_DEG},

    /* 右臂俯仰：MOTOR_C + ENC_3 */
    {MOTOR_C, ENC_3, 0.0f, +1, 0.0f,
     PWM_ANGLE_DEFAULT_PITCH_MIN_DEG,
     PWM_ANGLE_DEFAULT_PITCH_MAX_DEG},

    /* 右臂偏航：MOTOR_D + ENC_4 */
    {MOTOR_D, ENC_4, 0.0f, +1, 0.0f,
     PWM_ANGLE_DEFAULT_YAW_MIN_DEG,
     PWM_ANGLE_DEFAULT_YAW_MAX_DEG}
};

static uint8_t g_pwm_angle_enabled = 0;
static PWM_AngleDebug_t g_pwm_angle_debug;


/* ============================================================
 * 工具函数
 * ============================================================ */
/*
 * @brief 将浮点数限制在指定范围内
 *
 * @param value       原始输入值。
 *
 * @param min_value      最小允许值。
 *
 * @param max_value      最大允许值。
 *
 * @return        限幅后的结果。
 */
static float PWM_Angle_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}
/*
 * @brief 将 uint32 数值限制为 uint8 速度百分比
 *
 * @param value       原始输入值。
 *
 * @param min_value    最小允许值。
 *
 * @param max_value    最大允许值。
 *
 * @return        限幅后的 uint8_t 数值。
 *
 * 使用场景：
 * 将角度误差计算得到的速度百分比限制在 0~100 范围内。
 */
static uint8_t PWM_Angle_ClampU8(uint32_t value, uint32_t min_value, uint32_t max_value)
{
    if (value < min_value) return (uint8_t)min_value;
    if (value > max_value) return (uint8_t)max_value;
    return (uint8_t)value;
}
/*
 * @brief 将方向符号标准化为 +1 或 -1
 *
 * @param sign      输入方向符号。
 *
 * @return
 *        如果 sign < 0，返回 -1；
 *        否则返回 +1。
 */
static int8_t PWM_Angle_NormalizeSign(int8_t sign)
{
    return (sign < 0) ? -1 : +1;
}

/*
 * @brief 将角度归一化到 [-180, 180)
 *
 * @param angle_deg      原始角度，单位 deg。
 *
 * @return       归一化后的角度，单位 deg。
 *
 * 使用目的：避免 359° 和 1° 这种情况被误认为相差 358°。
 */
static float PWM_Angle_Normalize180(float angle_deg)
{
    while (angle_deg >= 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}
/*
 * @brief 判断 PWM 角度关节编号是否合法
 *
 * @param joint   关节编号。
 *
 * @return
 *        1：合法；
 *        0：非法。
 */
static uint8_t PWM_Angle_IsValidJoint(PWM_AngleJoint_t joint)
{
    return (joint < PWM_ANGLE_JOINT_NUM) ? 1u : 0u;
}

/*
 * @brief 计算目标角度与当前角度之间的最短误差
 *
 * @param target_deg      目标角度，单位 deg。
 *
 * @param current_deg     当前角度，单位 deg。
 *
 * @return        角度误差，范围 [-180, 180)，单位 deg。
 */
static float PWM_Angle_Error(float target_deg, float current_deg)
{
    return PWM_Angle_Normalize180(target_deg - current_deg);
}


/* ============================================================
 * 当前角度读取
 * ============================================================ */

float PWM_AngleServo_GetCurrentAngle(PWM_AngleJoint_t joint)
{
    if (!PWM_Angle_IsValidJoint(joint)) return 0.0f;

    Encoder_ID_t enc_id = g_pwm_angle_joint[joint].encoder_id;

    float enc_deg = encoder_data[enc_id].degree;

    float joint_deg =
        PWM_Angle_Normalize180(enc_deg -
                               g_pwm_angle_joint[joint].zero_offset_deg);

    return joint_deg;
}


/* ============================================================
 * 单关节角度闭环
 * ============================================================ */
/*
 * @brief 更新单个 PWM 关节角度闭环
 *
 * @param joint      需要更新的关节。
 *
 * 功能：
 * 1. 读取当前编码器角度；
 * 2. 计算目标角度与当前角度的误差；
 * 3. 如果误差小于死区，则停止电机；
 * 4. 如果误差大于死区，则根据误差方向设置电机方向；
 * 5. 根据误差大小设置 PWM 速度百分比。
 */
static void PWM_Angle_UpdateOne(PWM_AngleJoint_t joint)
{
    if (!PWM_Angle_IsValidJoint(joint)) return;

    PWM_AngleJointConfig_t *cfg = &g_pwm_angle_joint[joint];
    PWM_AngleJointDebug_t *dbg = &g_pwm_angle_debug.joint[joint];

    float current_deg = PWM_AngleServo_GetCurrentAngle(joint);

    float target_deg =
        PWM_Angle_ClampFloat(cfg->target_deg, cfg->min_deg, cfg->max_deg);

    cfg->target_deg = target_deg;

    float error_deg =
        PWM_Angle_Error(target_deg, current_deg);

    dbg->current_deg = current_deg;
    dbg->target_deg = target_deg;
    dbg->error_deg = error_deg;

    if (fabsf(error_deg) <= PWM_ANGLE_DEADBAND_DEG) {
        Motor_Stop(cfg->motor_id);
        dbg->speed_percent = 0;
        dbg->status = PWM_ANGLE_OK;
        return;
    }

    float speed_f =
        fabsf(error_deg) * PWM_ANGLE_KP_PERCENT_PER_DEG;

    uint8_t speed_percent =
        PWM_Angle_ClampU8((uint32_t)speed_f,
                          PWM_ANGLE_MIN_SPEED_PERCENT,
                          PWM_ANGLE_MAX_SPEED_PERCENT);

    /*
     * direction_sign = +1：
     * error > 0 时，DIRECTION_FORWARD 使角度增大。
     *
     * direction_sign = -1：
     * error > 0 时，DIRECTION_REVERSE 使角度增大。
     */
    Motor_Direction_t dir =
        ((error_deg * (float)cfg->direction_sign) > 0.0f) ?
        DIRECTION_FORWARD :
        DIRECTION_REVERSE;

    Motor_SetDirection(cfg->motor_id, dir);
    Motor_SetSpeed(cfg->motor_id, speed_percent);

    dbg->speed_percent = speed_percent;
    dbg->status = PWM_ANGLE_OK;
}


/* ============================================================
 * 对外接口
 * ============================================================ */

void PWM_AngleServo_Init(void)
{
    g_pwm_angle_enabled = 0;

    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].motor_id = MOTOR_A;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].encoder_id = ENC_1;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].zero_offset_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].direction_sign = +1;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].target_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].min_deg = PWM_ANGLE_DEFAULT_PITCH_MIN_DEG;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].max_deg = PWM_ANGLE_DEFAULT_PITCH_MAX_DEG;

    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].motor_id = MOTOR_B;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].encoder_id = ENC_2;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].zero_offset_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].direction_sign = +1;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].target_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].min_deg = PWM_ANGLE_DEFAULT_YAW_MIN_DEG;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].max_deg = PWM_ANGLE_DEFAULT_YAW_MAX_DEG;

    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].motor_id = MOTOR_C;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].encoder_id = ENC_3;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].zero_offset_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].direction_sign = +1;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].target_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].min_deg = PWM_ANGLE_DEFAULT_PITCH_MIN_DEG;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].max_deg = PWM_ANGLE_DEFAULT_PITCH_MAX_DEG;

    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].motor_id = MOTOR_D;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].encoder_id = ENC_4;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].zero_offset_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].direction_sign = +1;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].target_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].min_deg = PWM_ANGLE_DEFAULT_YAW_MIN_DEG;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].max_deg = PWM_ANGLE_DEFAULT_YAW_MAX_DEG;

    for (uint8_t i = 0; i < PWM_ANGLE_JOINT_NUM; i++) {
        g_pwm_angle_debug.joint[i].current_deg = 0.0f;
        g_pwm_angle_debug.joint[i].target_deg = 0.0f;
        g_pwm_angle_debug.joint[i].error_deg = 0.0f;
        g_pwm_angle_debug.joint[i].speed_percent = 0;
        g_pwm_angle_debug.joint[i].status = PWM_ANGLE_DISABLED;
    }

    PWM_AngleServo_StopAll();
}

void PWM_AngleServo_Enable(uint8_t enable)
{
    g_pwm_angle_enabled = enable ? 1u : 0u;

    if (!g_pwm_angle_enabled) {
        PWM_AngleServo_StopAll();

        for (uint8_t i = 0; i < PWM_ANGLE_JOINT_NUM; i++) {
            g_pwm_angle_debug.joint[i].status = PWM_ANGLE_DISABLED;
        }
    }
}

uint8_t PWM_AngleServo_IsEnabled(void)
{
    return g_pwm_angle_enabled;
}

void PWM_AngleServo_SetTarget(PWM_AngleJoint_t joint, float target_deg)
{
    if (!PWM_Angle_IsValidJoint(joint)) return;

    float limited =
        PWM_Angle_ClampFloat(target_deg,
                             g_pwm_angle_joint[joint].min_deg,
                             g_pwm_angle_joint[joint].max_deg);

    g_pwm_angle_joint[joint].target_deg = limited;

    g_pwm_angle_debug.joint[joint].target_deg = limited;

    if (limited != target_deg) {
        g_pwm_angle_debug.joint[joint].status = PWM_ANGLE_TARGET_LIMITED;
    }
}

void PWM_AngleServo_SetTargetAll(float left_pitch_deg,
                                 float left_yaw_deg,
                                 float right_pitch_deg,
                                 float right_yaw_deg)
{
    PWM_AngleServo_SetTarget(PWM_ANGLE_LEFT_PITCH, left_pitch_deg);
    PWM_AngleServo_SetTarget(PWM_ANGLE_LEFT_YAW, left_yaw_deg);
    PWM_AngleServo_SetTarget(PWM_ANGLE_RIGHT_PITCH, right_pitch_deg);
    PWM_AngleServo_SetTarget(PWM_ANGLE_RIGHT_YAW, right_yaw_deg);
}

void PWM_AngleServo_Update_5ms(void)
{
    if (!g_pwm_angle_enabled) {
        PWM_AngleServo_StopAll();

        for (uint8_t i = 0; i < PWM_ANGLE_JOINT_NUM; i++) {
            g_pwm_angle_debug.joint[i].status = PWM_ANGLE_DISABLED;
        }

        return;
    }

    PWM_Angle_UpdateOne(PWM_ANGLE_LEFT_PITCH);
    PWM_Angle_UpdateOne(PWM_ANGLE_LEFT_YAW);
    PWM_Angle_UpdateOne(PWM_ANGLE_RIGHT_PITCH);
    PWM_Angle_UpdateOne(PWM_ANGLE_RIGHT_YAW);
}

void PWM_AngleServo_Stop(PWM_AngleJoint_t joint)
{
    if (!PWM_Angle_IsValidJoint(joint)) return;

    Motor_Stop(g_pwm_angle_joint[joint].motor_id);
    g_pwm_angle_debug.joint[joint].speed_percent = 0;
}

void PWM_AngleServo_StopAll(void)
{
    Motor_Stop(MOTOR_A);
    Motor_Stop(MOTOR_B);
    Motor_Stop(MOTOR_C);
    Motor_Stop(MOTOR_D);

    for (uint8_t i = 0; i < PWM_ANGLE_JOINT_NUM; i++) {
        g_pwm_angle_debug.joint[i].speed_percent = 0;
    }
}

uint8_t PWM_AngleServo_IsTargetReached(PWM_AngleJoint_t joint)
{
    if (!PWM_Angle_IsValidJoint(joint)) return 0;

    float current = PWM_AngleServo_GetCurrentAngle(joint);
    float target = g_pwm_angle_joint[joint].target_deg;

    float error = fabsf(PWM_Angle_Error(target, current));

    return (error <= PWM_ANGLE_DEADBAND_DEG) ? 1u : 0u;
}

uint8_t PWM_AngleServo_IsAllTargetReached(void)
{
    for (uint8_t i = 0; i < PWM_ANGLE_JOINT_NUM; i++) {
        if (!PWM_AngleServo_IsTargetReached((PWM_AngleJoint_t)i)) {
            return 0;
        }
    }

    return 1;
}

PWM_AngleDebug_t PWM_AngleServo_GetDebugInfo(void)
{
    for (uint8_t i = 0; i < PWM_ANGLE_JOINT_NUM; i++) {
        PWM_AngleJoint_t joint = (PWM_AngleJoint_t)i;

        float current = PWM_AngleServo_GetCurrentAngle(joint);
        float target = g_pwm_angle_joint[i].target_deg;
        float error = PWM_Angle_Error(target, current);

        g_pwm_angle_debug.joint[i].current_deg = current;
        g_pwm_angle_debug.joint[i].target_deg = target;
        g_pwm_angle_debug.joint[i].error_deg = error;
    }

    return g_pwm_angle_debug;
}

float PWM_AngleServo_GetTargetAngle(PWM_AngleJoint_t joint)
{
    if (!PWM_Angle_IsValidJoint(joint)) return 0.0f;
    return g_pwm_angle_joint[joint].target_deg;
}

void PWM_AngleServo_SetZeroOffset(PWM_AngleJoint_t joint, float zero_offset_deg)
{
    if (!PWM_Angle_IsValidJoint(joint)) return;
    g_pwm_angle_joint[joint].zero_offset_deg = zero_offset_deg;
}

void PWM_AngleServo_SetZeroOffsetAll(float left_pitch_offset,
                                     float left_yaw_offset,
                                     float right_pitch_offset,
                                     float right_yaw_offset)
{
    PWM_AngleServo_SetZeroOffset(PWM_ANGLE_LEFT_PITCH, left_pitch_offset);
    PWM_AngleServo_SetZeroOffset(PWM_ANGLE_LEFT_YAW, left_yaw_offset);
    PWM_AngleServo_SetZeroOffset(PWM_ANGLE_RIGHT_PITCH, right_pitch_offset);
    PWM_AngleServo_SetZeroOffset(PWM_ANGLE_RIGHT_YAW, right_yaw_offset);
}

void PWM_AngleServo_SetDirectionSign(PWM_AngleJoint_t joint, int8_t sign)
{
    if (!PWM_Angle_IsValidJoint(joint)) return;
    g_pwm_angle_joint[joint].direction_sign = PWM_Angle_NormalizeSign(sign);
}

void PWM_AngleServo_SetDirectionSignAll(int8_t left_pitch_sign,
                                        int8_t left_yaw_sign,
                                        int8_t right_pitch_sign,
                                        int8_t right_yaw_sign)
{
    PWM_AngleServo_SetDirectionSign(PWM_ANGLE_LEFT_PITCH, left_pitch_sign);
    PWM_AngleServo_SetDirectionSign(PWM_ANGLE_LEFT_YAW, left_yaw_sign);
    PWM_AngleServo_SetDirectionSign(PWM_ANGLE_RIGHT_PITCH, right_pitch_sign);
    PWM_AngleServo_SetDirectionSign(PWM_ANGLE_RIGHT_YAW, right_yaw_sign);
}

void PWM_AngleServo_SetLimit(PWM_AngleJoint_t joint, float min_deg, float max_deg)
{
    if (!PWM_Angle_IsValidJoint(joint)) return;
    if (max_deg <= min_deg) return;

    g_pwm_angle_joint[joint].min_deg = min_deg;
    g_pwm_angle_joint[joint].max_deg = max_deg;
}
