#include "joint_servo.h"

#include "math.h"
#include "pwm_motor.h"
#include "spi4.h"
#include "PID.h"
#include "encoder_counter.h"
#include "CAN_receive.h"


/* ============================================================
 * 基本常量
 * ============================================================ */

#ifndef JS_PI
#define JS_PI 3.14159265358979323846f
#endif

/*
 * 摩擦轮直径，单位 mm。
 *
 * climb_control.h 中已有：
 * PULLEY_RADIUS = 19.0f
 */
#define JS_PULLEY_DIAMETER_MM       (2.0f * PULLEY_RADIUS)


/* ============================================================
 * 角度位置环参数
 * ============================================================ */

/*
 * PWM 角度位置环：
 *
 * u_pwm = Kp * |error|
 *
 * 其中：
 * u_pwm ：PWM 速度百分比，范围 0~100
 * Kp    ：角度误差到速度百分比的比例系数
 * error ：目标角度 - 当前角度，单位 deg
 */
#define JS_ANGLE_KP_PERCENT_PER_DEG     5.0f

/* 角度到位死区，单位 deg */
#define JS_ANGLE_DEADBAND_DEG           0.8f

/* PWM 电机最小/最大速度百分比 */
#define JS_PWM_MIN_SPEED_PERCENT        18u
#define JS_PWM_MAX_SPEED_PERCENT        70u


/* ============================================================
 * 伸缩长度位置环参数
 * ============================================================ */

/*
 * 伸缩长度位置环：
 *
 * v = Kp * e
 *
 * 其中：
 * v  ：期望伸缩线速度，单位 mm/s
 * Kp ：长度位置环比例系数，单位 1/s
 * e  ：目标长度 - 当前长度，单位 mm
 */
#define JS_EXT_KP_MM_S_PER_MM           1.2f

/* 伸缩长度到位死区，单位 mm */
#define JS_EXT_DEADBAND_MM              2.0f

/*
 * 伸缩电机最小/最大目标转子转速，单位 rpm。
 *
 * 注意：
 * 这里是 C620 控制侧的转子目标转速，不是 M3508 输出轴转速。
 */
#define JS_EXT_MIN_ROTOR_RPM            80
#define JS_EXT_MAX_ROTOR_RPM            2800


/* ============================================================
 * 全局变量定义
 * ============================================================ */

/*
 * 全局减速比变量。
 *
 * 默认使用 M3508 官方减速比 3591/187。
 * 如果实测有偏差，可以通过 JointServo_SetReductionRatio() 修改。
 */
float g_joint_servo_m3508_reduction_ratio =
    JOINT_SERVO_DEFAULT_M3508_REDUCTION_RATIO;

/*
 * 当前 M3508 输出轴转速，单位 rpm。
 *
 * output_rpm = C620反馈转子转速 / 减速比
 */
volatile float g_joint_servo_left_output_rpm = 0.0f;
volatile float g_joint_servo_right_output_rpm = 0.0f;

/*
 * 目标 M3508 输出轴转速，单位 rpm。
 *
 * 由伸缩长度位置环计算得到。
 */
volatile float g_joint_servo_left_target_output_rpm = 0.0f;
volatile float g_joint_servo_right_target_output_rpm = 0.0f;


/* ============================================================
 * 内部类型
 * ============================================================ */

typedef struct {
    Motor_ID_t motor_id;
    Encoder_ID_t encoder_id;
    float zero_offset_deg;
    int8_t direction_sign;
} AngleServoMap_t;

typedef struct {
    uint8_t pid_motor_id;
    float base_length_mm;
    int8_t direction_sign;
} ExtensionServoMap_t;


/* ============================================================
 * 内部变量
 * ============================================================ */

/*
 * 角度关节映射：
 * 0：左臂俯仰 MOTOR_A + ENC_1
 * 1：左臂偏航 MOTOR_B + ENC_2
 * 2：右臂俯仰 MOTOR_C + ENC_3
 * 3：右臂偏航 MOTOR_D + ENC_4
 */
static AngleServoMap_t g_angle_servo[4] = {
    {MOTOR_A, ENC_1, 0.0f, +1},
    {MOTOR_B, ENC_2, 0.0f, +1},
    {MOTOR_C, ENC_3, 0.0f, +1},
    {MOTOR_D, ENC_4, 0.0f, +1}
};

/*
 * 伸缩关节映射：
 * 0：左臂伸缩 PID motor_id = 0
 * 1：右臂伸缩 PID motor_id = 1
 */
static ExtensionServoMap_t g_ext_servo[2] = {
    {0, D3_MIN_LENGTH, +1},
    {1, D3_MIN_LENGTH, +1}
};

static JointAngle_t g_target_left = {0.0f, 0.0f, D3_MIN_LENGTH};
static JointAngle_t g_target_right = {0.0f, 0.0f, D3_MIN_LENGTH};

static JointServoDebug_t g_debug;
static uint8_t g_joint_servo_enabled = 0;


/* ============================================================
 * 内部数学工具函数
 * ============================================================ */

static float JS_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static int16_t JS_ClampInt16(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return (int16_t)min_value;
    if (value > max_value) return (int16_t)max_value;
    return (int16_t)value;
}

static uint8_t JS_ClampU8(uint32_t value, uint32_t min_value, uint32_t max_value)
{
    if (value < min_value) return (uint8_t)min_value;
    if (value > max_value) return (uint8_t)max_value;
    return (uint8_t)value;
}

/*
 * 将角度归一化到 [-180, 180)
 */
static float JS_NormalizeAngle180(float angle_deg)
{
    while (angle_deg >= 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

/*
 * 计算目标角度与当前角度之间的最短误差。
 *
 * e = target - current
 *
 * 其中：
 * e       ：角度误差，单位 deg
 * target  ：目标角度，单位 deg
 * current ：当前角度，单位 deg
 */
static float JS_AngleErrorDeg(float target_deg, float current_deg)
{
    return JS_NormalizeAngle180(target_deg - current_deg);
}

static int8_t JS_NormalizeSign(int8_t sign)
{
    return (sign < 0) ? -1 : +1;
}

/*
 * 获取当前有效减速比。
 *
 * 如果全局变量被误设为过小值，则自动回退到官方默认值。
 */
static float JS_GetValidReductionRatio(void)
{
    if (g_joint_servo_m3508_reduction_ratio < 1.0f) {
        return JOINT_SERVO_DEFAULT_M3508_REDUCTION_RATIO;
    }

    return g_joint_servo_m3508_reduction_ratio;
}


/* ============================================================
 * 输出轴转速更新
 * ============================================================ */

/*
 * 根据 C620 反馈转子转速，更新 M3508 输出轴转速全局变量。
 *
 * output_rpm = rotor_rpm / reduction_ratio
 *
 * 其中：
 * output_rpm       ：M3508 输出轴转速，单位 rpm；
 * rotor_rpm        ：C620 反馈的转子转速，单位 rpm；
 * reduction_ratio  ：M3508 减速比。
 */
static void JS_UpdateOutputSpeedGlobals(void)
{
    float ratio = JS_GetValidReductionRatio();

    const motor_measure_t *left_motor =
        get_chassis_motor_measure_point(g_ext_servo[0].pid_motor_id);

    const motor_measure_t *right_motor =
        get_chassis_motor_measure_point(g_ext_servo[1].pid_motor_id);

    g_joint_servo_left_output_rpm =
        (float)left_motor->speed_rpm / ratio;

    g_joint_servo_right_output_rpm =
        (float)right_motor->speed_rpm / ratio;
}


/* ============================================================
 * 当前状态读取
 * ============================================================ */

/*
 * 获取某个角度关节的当前机械角度。
 *
 * joint_angle = Normalize180(encoder_degree - zero_offset)
 *
 * 其中：
 * joint_angle    ：转换后的关节角，单位 deg；
 * encoder_degree ：PQY13 编码器读数，单位 deg；
 * zero_offset    ：机械零位对应的编码器角度，单位 deg。
 */
static float JS_GetAngleJointDeg(uint8_t angle_index)
{
    if (angle_index > 3) return 0.0f;

    float enc_deg =
        encoder_data[g_angle_servo[angle_index].encoder_id].degree;

    float joint_deg =
        JS_NormalizeAngle180(enc_deg -
                             g_angle_servo[angle_index].zero_offset_deg);

    return joint_deg;
}

/*
 * 获取伸缩杆当前长度，单位 mm。
 *
 * d3 = d3_base + sign * (rotor_turns / reduction_ratio) * pi * D
 *
 * 其中：
 * d3              ：当前伸缩杆长度，单位 mm；
 * d3_base         ：伸缩杆零点基准长度，单位 mm；
 * sign            ：伸缩方向符号，+1 或 -1；
 * rotor_turns     ：电机转子累计圈数；
 * reduction_ratio ：M3508 减速比；
 * D               ：摩擦轮直径，单位 mm。
 */
static float JS_GetExtensionLengthMm(uint8_t ext_index)
{
    if (ext_index > 1) return D3_MIN_LENGTH;

    /*
     * Encoder_Get_Total_Angle() 已经在 encoder_counter.c 中完成：
     * 1. 编码器回绕判断；
     * 2. 多圈累计；
     * 3. 得到转子侧累计编码器计数。
     */
    int32_t total_angle_count =
        Encoder_Get_Total_Angle(g_ext_servo[ext_index].pid_motor_id);

    /*
     * 转子累计圈数：
     *
     * rotor_turns = total_angle_count / 8192
     *
     * 其中：
     * rotor_turns       ：电机转子累计圈数；
     * total_angle_count ：累计编码器计数；
     * 8192              ：C620/M3508 一圈对应的编码器计数。
     */
    float rotor_turns = (float)total_angle_count / 8192.0f;

    /*
     * 输出轴/摩擦轮累计圈数：
     *
     * output_turns = rotor_turns / reduction_ratio
     *
     * 其中：
     * output_turns     ：M3508 输出轴或摩擦轮累计圈数；
     * reduction_ratio  ：M3508 减速比。
     */
    float ratio = JS_GetValidReductionRatio();
    float output_turns = rotor_turns / ratio;

    /*
     * 伸缩杆相对零点的长度变化：
     *
     * delta_mm = output_turns * pi * D
     *
     * 其中：
     * delta_mm     ：伸缩长度变化量，单位 mm；
     * output_turns ：输出轴/摩擦轮累计圈数；
     * D            ：摩擦轮直径，单位 mm。
     */
    float delta_mm = output_turns * JS_PI * JS_PULLEY_DIAMETER_MM;

    /*
     * 当前绝对伸缩长度：
     *
     * d3 = d3_base + sign * delta_mm
     *
     * 其中：
     * d3_base ：伸缩杆零点基准长度；
     * sign    ：伸缩方向符号；
     * delta_mm：相对零点的伸缩变化量。
     */
    float d3 = g_ext_servo[ext_index].base_length_mm +
               (float)g_ext_servo[ext_index].direction_sign * delta_mm;

    return d3;
}

JointAngle_t JointServo_GetCurrentLeft(void)
{
    JointAngle_t joint;

    joint.theta2 = JS_GetAngleJointDeg(0);   /* 左俯仰 */
    joint.theta1 = JS_GetAngleJointDeg(1);   /* 左偏航 */
    joint.d3     = JS_GetExtensionLengthMm(0);

    return joint;
}

JointAngle_t JointServo_GetCurrentRight(void)
{
    JointAngle_t joint;

    joint.theta2 = JS_GetAngleJointDeg(2);   /* 右俯仰 */
    joint.theta1 = JS_GetAngleJointDeg(3);   /* 右偏航 */
    joint.d3     = JS_GetExtensionLengthMm(1);

    return joint;
}


/* ============================================================
 * 目标限位
 * ============================================================ */

static JointAngle_t JS_LimitLeftTarget(JointAngle_t target)
{
    target.theta1 = JS_ClampFloat(target.theta1, YAW_MIN, YAW_MAX);
    target.theta2 = JS_ClampFloat(target.theta2, PITCH_MIN, PITCH_MAX);
    target.d3     = JS_ClampFloat(target.d3, D3_MIN_LENGTH, D3_MAX_LENGTH);
    return target;
}

static JointAngle_t JS_LimitRightTarget(JointAngle_t target)
{
    /*
     * 说明：
     * 这里暂时沿用 climb_control.h 中相同的 YAW_MIN/YAW_MAX。
     * 如果右臂偏航在你的机械定义里是镜像角度，应在 climb_control 的 IK
     * 或者 JointServo_SetAngleDirectionSign() 中统一处理符号。
     */
    target.theta1 = JS_ClampFloat(target.theta1, YAW_MIN, YAW_MAX);
    target.theta2 = JS_ClampFloat(target.theta2, PITCH_MIN, PITCH_MAX);
    target.d3     = JS_ClampFloat(target.d3, D3_MIN_LENGTH, D3_MAX_LENGTH);
    return target;
}


/* ============================================================
 * 单关节控制
 * ============================================================ */

/*
 * PWM 角度位置环。
 *
 * error = target - current
 * speed_percent = Kp * |error|
 *
 * 其中：
 * error         ：角度误差，单位 deg；
 * target_deg    ：目标角度，单位 deg；
 * current_deg   ：当前角度，单位 deg；
 * speed_percent ：PWM 速度百分比，范围 0~100。
 */
static void JS_ControlAngleJoint(uint8_t angle_index,
                                 float target_deg,
                                 float *err_out)
{
    if (angle_index > 3) return;

    float current_deg = JS_GetAngleJointDeg(angle_index);
    float error_deg = JS_AngleErrorDeg(target_deg, current_deg);

    if (err_out != 0) {
        *err_out = error_deg;
    }

    Motor_ID_t motor_id = g_angle_servo[angle_index].motor_id;

    if (fabsf(error_deg) <= JS_ANGLE_DEADBAND_DEG) {
        Motor_Stop(motor_id);
        return;
    }

    float speed_f =
        fabsf(error_deg) * JS_ANGLE_KP_PERCENT_PER_DEG;

    uint8_t speed_percent =
        JS_ClampU8((uint32_t)speed_f,
                   JS_PWM_MIN_SPEED_PERCENT,
                   JS_PWM_MAX_SPEED_PERCENT);

    /*
     * direction_sign = +1：
     * error > 0 时，DIRECTION_FORWARD 使角度增大。
     *
     * direction_sign = -1：
     * error > 0 时，DIRECTION_REVERSE 使角度增大。
     */
    int8_t sign = g_angle_servo[angle_index].direction_sign;

    Motor_Direction_t direction =
        ((error_deg * (float)sign) > 0.0f) ?
        DIRECTION_FORWARD :
        DIRECTION_REVERSE;

    Motor_SetDirection(motor_id, direction);
    Motor_SetSpeed(motor_id, speed_percent);
}

/*
 * 伸缩长度位置环。
 *
 * e_d = d3_target - d3_current
 * v_d = Kp * e_d
 *
 * 输出轴目标转速：
 *
 * output_rpm = v_d / (pi * D) * 60
 *
 * 转子目标转速：
 *
 * rotor_rpm = output_rpm * reduction_ratio
 *
 * 其中：
 * e_d             ：伸缩长度误差，单位 mm；
 * d3_target       ：目标伸缩长度，单位 mm；
 * d3_current      ：当前伸缩长度，单位 mm；
 * v_d             ：目标伸缩线速度，单位 mm/s；
 * D               ：摩擦轮直径，单位 mm；
 * output_rpm      ：M3508 输出轴/摩擦轮目标转速，单位 rpm；
 * rotor_rpm       ：发送给 PID_SetTargetSpeed() 的目标转子转速，单位 rpm；
 * reduction_ratio ：M3508 减速比。
 */
static void JS_ControlExtensionJoint(uint8_t ext_index,
                                     float target_mm,
                                     float *err_out)
{
    if (ext_index > 1) return;

    float current_mm = JS_GetExtensionLengthMm(ext_index);
    float error_mm = target_mm - current_mm;

    if (err_out != 0) {
        *err_out = error_mm;
    }

    uint8_t pid_id = g_ext_servo[ext_index].pid_motor_id;

    if (fabsf(error_mm) <= JS_EXT_DEADBAND_MM) {
        PID_SetTargetSpeed(pid_id, 0);

        if (ext_index == 0) {
            g_joint_servo_left_target_output_rpm = 0.0f;
        } else {
            g_joint_servo_right_target_output_rpm = 0.0f;
        }

        return;
    }

    /*
     * 长度位置环：
     *
     * v_mm_s = Kp * error_mm
     */
    float v_mm_s = JS_EXT_KP_MM_S_PER_MM * error_mm;

    /*
     * 输出轴/摩擦轮目标转速：
     *
     * output_rpm = v_mm_s / (pi * D) * 60
     */
    float output_rpm =
        v_mm_s / (JS_PI * JS_PULLEY_DIAMETER_MM) * 60.0f;

    /*
     * 根据伸缩方向符号，将“伸缩长度正方向”转换为“转子正方向”。
     *
     * target_output_rpm 是 M3508 输出轴/摩擦轮目标转速，单位 rpm。
     */
    float target_output_rpm =
        output_rpm * (float)g_ext_servo[ext_index].direction_sign;

    if (ext_index == 0) {
        g_joint_servo_left_target_output_rpm = target_output_rpm;
    } else {
        g_joint_servo_right_target_output_rpm = target_output_rpm;
    }

    /*
     * C620 的速度环目标是转子 rpm，因此需要乘以减速比。
     */
    float ratio = JS_GetValidReductionRatio();

    float rotor_rpm_f =
        target_output_rpm * ratio;

    int16_t rotor_rpm =
        JS_ClampInt16((int32_t)rotor_rpm_f,
                      -JS_EXT_MAX_ROTOR_RPM,
                      JS_EXT_MAX_ROTOR_RPM);

    /*
     * 小速度可能克服不了静摩擦，给最小转子转速。
     * 注意：只有误差超过死区时才使用最小转速。
     */
    if (rotor_rpm > 0 && rotor_rpm < JS_EXT_MIN_ROTOR_RPM) {
        rotor_rpm = JS_EXT_MIN_ROTOR_RPM;
    } else if (rotor_rpm < 0 && rotor_rpm > -JS_EXT_MIN_ROTOR_RPM) {
        rotor_rpm = -JS_EXT_MIN_ROTOR_RPM;
    }

    PID_SetTargetSpeed(pid_id, rotor_rpm);
}


/* ============================================================
 * 对外接口
 * ============================================================ */

void JointServo_Init(void)
{
    g_joint_servo_enabled = 0;

    g_angle_servo[0].motor_id = MOTOR_A;
    g_angle_servo[0].encoder_id = ENC_1;
    g_angle_servo[0].zero_offset_deg = 0.0f;
    g_angle_servo[0].direction_sign = +1;

    g_angle_servo[1].motor_id = MOTOR_B;
    g_angle_servo[1].encoder_id = ENC_2;
    g_angle_servo[1].zero_offset_deg = 0.0f;
    g_angle_servo[1].direction_sign = +1;

    g_angle_servo[2].motor_id = MOTOR_C;
    g_angle_servo[2].encoder_id = ENC_3;
    g_angle_servo[2].zero_offset_deg = 0.0f;
    g_angle_servo[2].direction_sign = +1;

    g_angle_servo[3].motor_id = MOTOR_D;
    g_angle_servo[3].encoder_id = ENC_4;
    g_angle_servo[3].zero_offset_deg = 0.0f;
    g_angle_servo[3].direction_sign = +1;

    g_ext_servo[0].pid_motor_id = 0;
    g_ext_servo[0].base_length_mm = D3_MIN_LENGTH;
    g_ext_servo[0].direction_sign = +1;

    g_ext_servo[1].pid_motor_id = 1;
    g_ext_servo[1].base_length_mm = D3_MIN_LENGTH;
    g_ext_servo[1].direction_sign = +1;

    /*
     * 如果全局减速比被外部错误修改为非法值，则恢复默认值。
     */
    if (g_joint_servo_m3508_reduction_ratio < 1.0f) {
        g_joint_servo_m3508_reduction_ratio =
            JOINT_SERVO_DEFAULT_M3508_REDUCTION_RATIO;
    }

    JS_UpdateOutputSpeedGlobals();

    g_target_left = JointServo_GetCurrentLeft();
    g_target_right = JointServo_GetCurrentRight();

    g_debug.current_left = g_target_left;
    g_debug.current_right = g_target_right;
    g_debug.target_left = g_target_left;
    g_debug.target_right = g_target_right;

    g_debug.err_left_yaw = 0.0f;
    g_debug.err_left_pitch = 0.0f;
    g_debug.err_left_d3 = 0.0f;
    g_debug.err_right_yaw = 0.0f;
    g_debug.err_right_pitch = 0.0f;
    g_debug.err_right_d3 = 0.0f;

    g_debug.left_output_rpm = g_joint_servo_left_output_rpm;
    g_debug.right_output_rpm = g_joint_servo_right_output_rpm;
    g_debug.left_target_output_rpm = 0.0f;
    g_debug.right_target_output_rpm = 0.0f;

    g_debug.status = JOINT_SERVO_DISABLED;

    JointServo_StopAll();
}

void JointServo_Enable(uint8_t enable)
{
    g_joint_servo_enabled = enable ? 1u : 0u;

    if (!g_joint_servo_enabled) {
        JointServo_StopAll();
        g_debug.status = JOINT_SERVO_DISABLED;
    }
}

uint8_t JointServo_IsEnabled(void)
{
    return g_joint_servo_enabled;
}

void JointServo_SetTarget(JointAngle_t left_target,
                          JointAngle_t right_target)
{
    JointAngle_t left_limited = JS_LimitLeftTarget(left_target);
    JointAngle_t right_limited = JS_LimitRightTarget(right_target);

    g_target_left = left_limited;
    g_target_right = right_limited;

    g_debug.target_left = g_target_left;
    g_debug.target_right = g_target_right;

    if (left_limited.theta1 != left_target.theta1 ||
        left_limited.theta2 != left_target.theta2 ||
        left_limited.d3     != left_target.d3     ||
        right_limited.theta1 != right_target.theta1 ||
        right_limited.theta2 != right_target.theta2 ||
        right_limited.d3     != right_target.d3) {
        g_debug.status = JOINT_SERVO_TARGET_LIMITED;
    } else {
        g_debug.status = JOINT_SERVO_OK;
    }
}

void JointServo_ExecuteTarget(JointAngle_t left_target,
                              JointAngle_t right_target)
{
    JointServo_SetTarget(left_target, right_target);
    JointServo_Update_5ms();
}

void JointServo_Update_5ms(void)
{
    JS_UpdateOutputSpeedGlobals();

    if (!g_joint_servo_enabled) {
        JointServo_StopAll();
        g_debug.status = JOINT_SERVO_DISABLED;
        return;
    }

    /* 当前值记录 */
    g_debug.current_left = JointServo_GetCurrentLeft();
    g_debug.current_right = JointServo_GetCurrentRight();

    /*
     * 左臂：
     * theta2 -> 左俯仰 MOTOR_A / ENC_1
     * theta1 -> 左偏航 MOTOR_B / ENC_2
     * d3     -> 左伸缩 PID motor 0
     */
    JS_ControlAngleJoint(0, g_target_left.theta2,
                         &g_debug.err_left_pitch);

    JS_ControlAngleJoint(1, g_target_left.theta1,
                         &g_debug.err_left_yaw);

    JS_ControlExtensionJoint(0, g_target_left.d3,
                             &g_debug.err_left_d3);

    /*
     * 右臂：
     * theta2 -> 右俯仰 MOTOR_C / ENC_3
     * theta1 -> 右偏航 MOTOR_D / ENC_4
     * d3     -> 右伸缩 PID motor 1
     */
    JS_ControlAngleJoint(2, g_target_right.theta2,
                         &g_debug.err_right_pitch);

    JS_ControlAngleJoint(3, g_target_right.theta1,
                         &g_debug.err_right_yaw);

    JS_ControlExtensionJoint(1, g_target_right.d3,
                             &g_debug.err_right_d3);

    /*
     * 更新调试速度信息。
     */
    g_debug.left_output_rpm = g_joint_servo_left_output_rpm;
    g_debug.right_output_rpm = g_joint_servo_right_output_rpm;
    g_debug.left_target_output_rpm = g_joint_servo_left_target_output_rpm;
    g_debug.right_target_output_rpm = g_joint_servo_right_target_output_rpm;

    /*
     * 电流/堵转状态检查。
     * 如果 PID.c 判定伸缩电机故障，立即停止伸缩速度目标。
     */
    if (PID_GetMotorStatus(0) == MOTOR_STATUS_ERROR ||
        PID_GetMotorStatus(1) == MOTOR_STATUS_ERROR) {
        PID_SetTargetSpeed(0, 0);
        PID_SetTargetSpeed(1, 0);

        g_joint_servo_left_target_output_rpm = 0.0f;
        g_joint_servo_right_target_output_rpm = 0.0f;

        g_debug.status = JOINT_SERVO_MOTOR_ERROR;
        return;
    }

    if (g_debug.status != JOINT_SERVO_TARGET_LIMITED) {
        g_debug.status = JOINT_SERVO_OK;
    }
}

void JointServo_StopAll(void)
{
    PID_SetTargetSpeed(0, 0);
    PID_SetTargetSpeed(1, 0);

    g_joint_servo_left_target_output_rpm = 0.0f;
    g_joint_servo_right_target_output_rpm = 0.0f;

    Motor_Stop(MOTOR_A);
    Motor_Stop(MOTOR_B);
    Motor_Stop(MOTOR_C);
    Motor_Stop(MOTOR_D);
}

void JointServo_EmergencyStop(void)
{
    PID_SetTargetSpeed(0, 0);
    PID_SetTargetSpeed(1, 0);

    g_joint_servo_left_target_output_rpm = 0.0f;
    g_joint_servo_right_target_output_rpm = 0.0f;

    Motor_Emergency_Stop();

    g_joint_servo_enabled = 0;
    g_debug.status = JOINT_SERVO_DISABLED;
}

uint8_t JointServo_IsTargetReached(void)
{
    JointAngle_t left_now = JointServo_GetCurrentLeft();
    JointAngle_t right_now = JointServo_GetCurrentRight();

    float e_lp =
        fabsf(JS_AngleErrorDeg(g_target_left.theta2,
                               left_now.theta2));

    float e_ly =
        fabsf(JS_AngleErrorDeg(g_target_left.theta1,
                               left_now.theta1));

    float e_ld =
        fabsf(g_target_left.d3 - left_now.d3);

    float e_rp =
        fabsf(JS_AngleErrorDeg(g_target_right.theta2,
                               right_now.theta2));

    float e_ry =
        fabsf(JS_AngleErrorDeg(g_target_right.theta1,
                               right_now.theta1));

    float e_rd =
        fabsf(g_target_right.d3 - right_now.d3);

    if (e_lp <= JS_ANGLE_DEADBAND_DEG &&
        e_ly <= JS_ANGLE_DEADBAND_DEG &&
        e_ld <= JS_EXT_DEADBAND_MM &&
        e_rp <= JS_ANGLE_DEADBAND_DEG &&
        e_ry <= JS_ANGLE_DEADBAND_DEG &&
        e_rd <= JS_EXT_DEADBAND_MM) {
        return 1;
    }

    return 0;
}

JointServoDebug_t JointServo_GetDebugInfo(void)
{
    JS_UpdateOutputSpeedGlobals();

    g_debug.current_left = JointServo_GetCurrentLeft();
    g_debug.current_right = JointServo_GetCurrentRight();

    g_debug.left_output_rpm = g_joint_servo_left_output_rpm;
    g_debug.right_output_rpm = g_joint_servo_right_output_rpm;
    g_debug.left_target_output_rpm = g_joint_servo_left_target_output_rpm;
    g_debug.right_target_output_rpm = g_joint_servo_right_target_output_rpm;

    return g_debug;
}


/* ============================================================
 * 全局减速比与输出轴转速接口
 * ============================================================ */

void JointServo_SetReductionRatio(float ratio)
{
    /*
     * M3508 官方值约为 19.203。
     * 这里做一个宽范围保护，避免误传 0 或负数。
     */
    if (ratio < 1.0f) {
        g_joint_servo_m3508_reduction_ratio =
            JOINT_SERVO_DEFAULT_M3508_REDUCTION_RATIO;
    } else {
        g_joint_servo_m3508_reduction_ratio = ratio;
    }
}

float JointServo_GetReductionRatio(void)
{
    return JS_GetValidReductionRatio();
}

float JointServo_GetLeftOutputRPM(void)
{
    JS_UpdateOutputSpeedGlobals();
    return g_joint_servo_left_output_rpm;
}

float JointServo_GetRightOutputRPM(void)
{
    JS_UpdateOutputSpeedGlobals();
    return g_joint_servo_right_output_rpm;
}

float JointServo_GetLeftTargetOutputRPM(void)
{
    return g_joint_servo_left_target_output_rpm;
}

float JointServo_GetRightTargetOutputRPM(void)
{
    return g_joint_servo_right_target_output_rpm;
}


/* ============================================================
 * 标定配置接口
 * ============================================================ */

void JointServo_SetAngleZeroOffset(float left_pitch_offset_deg,
                                   float left_yaw_offset_deg,
                                   float right_pitch_offset_deg,
                                   float right_yaw_offset_deg)
{
    g_angle_servo[0].zero_offset_deg = left_pitch_offset_deg;
    g_angle_servo[1].zero_offset_deg = left_yaw_offset_deg;
    g_angle_servo[2].zero_offset_deg = right_pitch_offset_deg;
    g_angle_servo[3].zero_offset_deg = right_yaw_offset_deg;
}

void JointServo_SetAngleDirectionSign(int8_t left_pitch_sign,
                                      int8_t left_yaw_sign,
                                      int8_t right_pitch_sign,
                                      int8_t right_yaw_sign)
{
    g_angle_servo[0].direction_sign =
        JS_NormalizeSign(left_pitch_sign);

    g_angle_servo[1].direction_sign =
        JS_NormalizeSign(left_yaw_sign);

    g_angle_servo[2].direction_sign =
        JS_NormalizeSign(right_pitch_sign);

    g_angle_servo[3].direction_sign =
        JS_NormalizeSign(right_yaw_sign);
}

void JointServo_SetExtensionDirectionSign(int8_t left_ext_sign,
                                          int8_t right_ext_sign)
{
    g_ext_servo[0].direction_sign =
        JS_NormalizeSign(left_ext_sign);

    g_ext_servo[1].direction_sign =
        JS_NormalizeSign(right_ext_sign);
}

void JointServo_SetExtensionBaseLength(float left_base_mm,
                                       float right_base_mm)
{
    g_ext_servo[0].base_length_mm =
        JS_ClampFloat(left_base_mm,
                      D3_MIN_LENGTH,
                      D3_MAX_LENGTH);

    g_ext_servo[1].base_length_mm =
        JS_ClampFloat(right_base_mm,
                      D3_MIN_LENGTH,
                      D3_MAX_LENGTH);
}
