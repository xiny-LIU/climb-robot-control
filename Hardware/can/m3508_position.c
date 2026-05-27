#include "m3508_position.h"

#include "math.h"
#include "PID.h"
#include "encoder_counter.h"
#include "CAN_receive.h"


/* ============================================================
 * 基本常量
 * ============================================================ */

#ifndef M3508_POS_PI
#define M3508_POS_PI 3.14159265358979323846f
#endif


/* ============================================================
 * 位置环参数
 * ============================================================ */

/*
 * 长度位置环：
 *
 * v = Kp * e
 *
 * 其中：
 * v：目标伸缩线速度，单位 mm/s；
 * Kp：长度位置环比例系数，单位 1/s；
 * e：伸缩长度误差，单位 mm。
 */
#define M3508_POS_KP_MM_S_PER_MM        1.2f

/* 伸缩长度到位死区，单位 mm */
#define M3508_POS_DEADBAND_MM           2.0f

/*
 * 速度环目标转子转速限幅，单位 rpm。
 *
 * 注意：
 * 这是传给 PID_SetTargetSpeed() 的目标转子转速，
 * 不是最终 CAN 发送值。
 * CAN 最终发送值仍然由 PID.c 算成电流指令。
 */
#define M3508_POS_MIN_ROTOR_RPM         80
#define M3508_POS_MAX_ROTOR_RPM         2800


/* ============================================================
 * 全局变量
 * ============================================================ */

float g_m3508_pos_reduction_ratio = M3508_POS_DEFAULT_REDUCTION_RATIO;

volatile float g_m3508_pos_left_output_rpm = 0.0f;
volatile float g_m3508_pos_right_output_rpm = 0.0f;

volatile float g_m3508_pos_left_target_output_rpm = 0.0f;
volatile float g_m3508_pos_right_target_output_rpm = 0.0f;


/* ============================================================
 * 内部类型
 * ============================================================ */

typedef struct {
    uint8_t pid_motor_id;       /* 0=左伸缩，1=右伸缩 */
    float base_length_mm;       /* 编码器清零时对应的伸缩杆长度 */
    float target_length_mm;     /* 目标伸缩长度 */
    int8_t direction_sign;      /* 伸缩方向符号，+1 或 -1 */
} M3508_PositionMotor_t;


/* ============================================================
 * 内部变量
 * ============================================================ */

static M3508_PositionMotor_t g_m3508_pos_motor[2] = {
    {0, M3508_POS_DEFAULT_D3_MIN_MM, M3508_POS_DEFAULT_D3_MIN_MM, +1},
    {1, M3508_POS_DEFAULT_D3_MIN_MM, M3508_POS_DEFAULT_D3_MIN_MM, +1}
};

static float g_length_min_mm = M3508_POS_DEFAULT_D3_MIN_MM;
static float g_length_max_mm = M3508_POS_DEFAULT_D3_MAX_MM;

static uint8_t g_m3508_pos_enabled = 0;

static M3508_PositionDebug_t g_m3508_pos_debug;


/* ============================================================
 * 工具函数
 * ============================================================ */
/*
 * @brief 将浮点数限制在指定范围内
 *
 * @param value      原始输入值。
 *
 * @param min_value   最小允许值。
 *
 * @param max_value   最大允许值。
 *
 * @return        限幅后的结果。
 */
static float M3508_Pos_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}
/*
 * @brief 将 int32 类型数值限制到 int16 范围内
 *
 * @param value      原始输入值。
 *
 * @param min_value  最小允许值。
 *
 * @param max_value  最大允许值。
 *
 * @return       限幅后的 int16_t 结果。
 *
 * 使用场景：
 * 用于将计算得到的目标转子转速限制在允许范围内。
 */
static int16_t M3508_Pos_ClampInt16(int32_t value, int32_t min_value, int32_t max_value)
{
    if (value < min_value) return (int16_t)min_value;
    if (value > max_value) return (int16_t)max_value;
    return (int16_t)value;
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
static int8_t M3508_Pos_NormalizeSign(int8_t sign)
{
    return (sign < 0) ? -1 : +1;
}
/*
 * @brief 获取当前有效减速比
 *
 * @return 当前有效减速比
 *
 * 说明：
 * 如果全局变量 g_m3508_pos_reduction_ratio 被错误设置为小于 1，
 * 则返回默认官方减速比 M3508_POS_DEFAULT_REDUCTION_RATIO。
 */
static float M3508_Pos_GetValidReductionRatio(void)
{
    if (g_m3508_pos_reduction_ratio < 1.0f) {
        return M3508_POS_DEFAULT_REDUCTION_RATIO;
    }

    return g_m3508_pos_reduction_ratio;
}
/*
 * @brief 将侧别枚举转换为数组下标
 *
 * @param side      M3508_POS_LEFT 或 M3508_POS_RIGHT。
 *
 * @return        左侧返回 0，右侧返回 1。
 */
static uint8_t M3508_Pos_SideToIndex(M3508_PositionSide_t side)
{
    return (side == M3508_POS_RIGHT) ? 1u : 0u;
}


/* ============================================================
 * 速度与长度换算
 * ============================================================ */

/*
 * @brief 更新左右 M3508 输出轴/摩擦轮当前转速
 *
 * 功能：
 * 1. 读取 C620 反馈的转子转速 speed_rpm；
 * 2. 除以 M3508 减速比；
 * 3. 得到输出轴/摩擦轮转速；
 * 4. 写入全局变量 g_m3508_pos_left_output_rpm 和
 *    g_m3508_pos_right_output_rpm。
 *
 * 公式：
 * output_rpm = rotor_rpm / reduction_ratio
 * 其中：
 * output_rpm：M3508 输出轴/摩擦轮转速，单位 rpm；
 * rotor_rpm：C620 反馈转子转速，单位 rpm；
 * reduction_ratio：M3508 减速比。
 */
static void M3508_Pos_UpdateOutputRPM(void)
{
    float ratio = M3508_Pos_GetValidReductionRatio();

    const motor_measure_t *left_motor =
        get_chassis_motor_measure_point(g_m3508_pos_motor[0].pid_motor_id);

    const motor_measure_t *right_motor =
        get_chassis_motor_measure_point(g_m3508_pos_motor[1].pid_motor_id);

    g_m3508_pos_left_output_rpm =
        (float)left_motor->speed_rpm / ratio;

    g_m3508_pos_right_output_rpm =
        (float)right_motor->speed_rpm / ratio;
}

/*
 * 获取当前伸缩长度。
 *
 * d3 = d3_base + sign * (rotor_turns / reduction_ratio) * pi * D
 *
 * 其中：
 * d3：当前伸缩长度，单位 mm；
 * d3_base：编码器清零时对应的基准长度，单位 mm；
 * sign：伸缩方向符号；
 * rotor_turns：电机转子累计圈数；
 * reduction_ratio：M3508 减速比；
 * D：摩擦轮直径，单位 mm。
 */
float M3508_Position_GetCurrentLength(M3508_PositionSide_t side)
{
    uint8_t index = M3508_Pos_SideToIndex(side);

    int32_t total_angle_count =
        Encoder_Get_Total_Angle(g_m3508_pos_motor[index].pid_motor_id);

    float rotor_turns = (float)total_angle_count / 8192.0f;
    float ratio = M3508_Pos_GetValidReductionRatio();
    float output_turns = rotor_turns / ratio;

    float delta_mm =
        output_turns * M3508_POS_PI * M3508_POS_PULLEY_DIAMETER_MM;

    float d3 =
        g_m3508_pos_motor[index].base_length_mm +
        (float)g_m3508_pos_motor[index].direction_sign * delta_mm;

    return d3;
}


/* ============================================================
 * 单侧位置闭环
 * ============================================================ */
/*
 * @brief 更新单侧 M3508 伸缩位置闭环
 *
 * @param index
 *        0：左伸缩；
 *        1：右伸缩。
 *
 * 功能：
 * 1. 读取当前伸缩长度；
 * 2. 计算长度误差；
 * 3. 长度误差转换为目标伸缩线速度；
 * 4. 线速度转换为输出轴目标转速；
 * 5. 输出轴目标转速转换为转子目标转速；
 * 6. 调用 PID_SetTargetSpeed()。
 */
static void M3508_Pos_UpdateOne(uint8_t index)
{
    if (index > 1) return;

    uint8_t pid_id = g_m3508_pos_motor[index].pid_motor_id;

    float current_mm =
        M3508_Position_GetCurrentLength((index == 0) ? M3508_POS_LEFT : M3508_POS_RIGHT);

    float target_mm =
        M3508_Pos_ClampFloat(g_m3508_pos_motor[index].target_length_mm,
                             g_length_min_mm,
                             g_length_max_mm);

    g_m3508_pos_motor[index].target_length_mm = target_mm;

    float error_mm = target_mm - current_mm;

    M3508_PositionMotorDebug_t *dbg =
        (index == 0) ? &g_m3508_pos_debug.left : &g_m3508_pos_debug.right;

    dbg->current_length_mm = current_mm;
    dbg->target_length_mm = target_mm;
    dbg->error_mm = error_mm;

    if (fabsf(error_mm) <= M3508_POS_DEADBAND_MM) {
        PID_SetTargetSpeed(pid_id, 0);

        dbg->target_output_rpm = 0.0f;
        dbg->rotor_rpm_set = 0.0f;
        dbg->status = M3508_POS_OK;

        if (index == 0) {
            g_m3508_pos_left_target_output_rpm = 0.0f;
        } else {
            g_m3508_pos_right_target_output_rpm = 0.0f;
        }

        return;
    }

    /*
     * 外层长度位置环：
     *
     * v_mm_s = Kp * error_mm
     */
    float v_mm_s = M3508_POS_KP_MM_S_PER_MM * error_mm;

    /*
     * 伸缩线速度 -> 输出轴/摩擦轮目标转速：
     *
     * output_rpm = v_mm_s / (pi * D) * 60
     */
    float output_rpm =
        v_mm_s / (M3508_POS_PI * M3508_POS_PULLEY_DIAMETER_MM) * 60.0f;

    /*
     * 根据机构方向，将伸缩正方向转成电机正方向。
     */
    float target_output_rpm =
        output_rpm * (float)g_m3508_pos_motor[index].direction_sign;

    if (index == 0) {
        g_m3508_pos_left_target_output_rpm = target_output_rpm;
    } else {
        g_m3508_pos_right_target_output_rpm = target_output_rpm;
    }

    /*
     * 输出轴目标转速 -> 转子目标转速：
     *
     * rotor_rpm_set = target_output_rpm * reduction_ratio
     *
     * 该值传给 PID_SetTargetSpeed()。
     * PID.c 再根据转速误差计算电流指令。
     */
    float ratio = M3508_Pos_GetValidReductionRatio();
    float rotor_rpm_set_f = target_output_rpm * ratio;

    int16_t rotor_rpm_set =
        M3508_Pos_ClampInt16((int32_t)rotor_rpm_set_f,
                             -M3508_POS_MAX_ROTOR_RPM,
                             M3508_POS_MAX_ROTOR_RPM);

    if (rotor_rpm_set > 0 && rotor_rpm_set < M3508_POS_MIN_ROTOR_RPM) {
        rotor_rpm_set = M3508_POS_MIN_ROTOR_RPM;
    } else if (rotor_rpm_set < 0 && rotor_rpm_set > -M3508_POS_MIN_ROTOR_RPM) {
        rotor_rpm_set = -M3508_POS_MIN_ROTOR_RPM;
    }

    PID_SetTargetSpeed(pid_id, rotor_rpm_set);

    dbg->target_output_rpm = target_output_rpm;
    dbg->rotor_rpm_set = (float)rotor_rpm_set;
    dbg->status = M3508_POS_OK;
}


/* ============================================================
 * 对外接口
 * ============================================================ */

void M3508_Position_Init(void)
{
    g_m3508_pos_enabled = 0;

    g_m3508_pos_reduction_ratio = M3508_POS_DEFAULT_REDUCTION_RATIO;

    g_m3508_pos_motor[0].pid_motor_id = 0;
    g_m3508_pos_motor[0].base_length_mm = M3508_POS_DEFAULT_D3_MIN_MM;
    g_m3508_pos_motor[0].target_length_mm = M3508_POS_DEFAULT_D3_MIN_MM;
    g_m3508_pos_motor[0].direction_sign = +1;

    g_m3508_pos_motor[1].pid_motor_id = 1;
    g_m3508_pos_motor[1].base_length_mm = M3508_POS_DEFAULT_D3_MIN_MM;
    g_m3508_pos_motor[1].target_length_mm = M3508_POS_DEFAULT_D3_MIN_MM;
    g_m3508_pos_motor[1].direction_sign = +1;

    g_length_min_mm = M3508_POS_DEFAULT_D3_MIN_MM;
    g_length_max_mm = M3508_POS_DEFAULT_D3_MAX_MM;

    M3508_Pos_UpdateOutputRPM();

    g_m3508_pos_debug.left.status = M3508_POS_DISABLED;
    g_m3508_pos_debug.right.status = M3508_POS_DISABLED;

    M3508_Position_StopAll();
}

void M3508_Position_Enable(uint8_t enable)
{
    g_m3508_pos_enabled = enable ? 1u : 0u;

    if (!g_m3508_pos_enabled) {
        M3508_Position_StopAll();
        g_m3508_pos_debug.left.status = M3508_POS_DISABLED;
        g_m3508_pos_debug.right.status = M3508_POS_DISABLED;
    }
}

uint8_t M3508_Position_IsEnabled(void)
{
    return g_m3508_pos_enabled;
}

void M3508_Position_SetTargetLength(M3508_PositionSide_t side, float target_length_mm)
{
    uint8_t index = M3508_Pos_SideToIndex(side);

    float limited =
        M3508_Pos_ClampFloat(target_length_mm, g_length_min_mm, g_length_max_mm);

    g_m3508_pos_motor[index].target_length_mm = limited;

    M3508_PositionMotorDebug_t *dbg =
        (index == 0) ? &g_m3508_pos_debug.left : &g_m3508_pos_debug.right;

    if (limited != target_length_mm) {
        dbg->status = M3508_POS_TARGET_LIMITED;
    }
}

void M3508_Position_SetTargetLengthBoth(float left_target_mm, float right_target_mm)
{
    M3508_Position_SetTargetLength(M3508_POS_LEFT, left_target_mm);
    M3508_Position_SetTargetLength(M3508_POS_RIGHT, right_target_mm);
}

void M3508_Position_Update_5ms(void)
{
    M3508_Pos_UpdateOutputRPM();

    g_m3508_pos_debug.left.output_rpm = g_m3508_pos_left_output_rpm;
    g_m3508_pos_debug.right.output_rpm = g_m3508_pos_right_output_rpm;

    if (!g_m3508_pos_enabled) {
        M3508_Position_StopAll();
        g_m3508_pos_debug.left.status = M3508_POS_DISABLED;
        g_m3508_pos_debug.right.status = M3508_POS_DISABLED;
        return;
    }

    if (PID_GetMotorStatus(0) == MOTOR_STATUS_ERROR) {
        M3508_Position_Stop(M3508_POS_LEFT);
        g_m3508_pos_debug.left.status = M3508_POS_MOTOR_ERROR;
    } else {
        M3508_Pos_UpdateOne(0);
    }

    if (PID_GetMotorStatus(1) == MOTOR_STATUS_ERROR) {
        M3508_Position_Stop(M3508_POS_RIGHT);
        g_m3508_pos_debug.right.status = M3508_POS_MOTOR_ERROR;
    } else {
        M3508_Pos_UpdateOne(1);
    }

    g_m3508_pos_debug.left.output_rpm = g_m3508_pos_left_output_rpm;
    g_m3508_pos_debug.right.output_rpm = g_m3508_pos_right_output_rpm;
    g_m3508_pos_debug.left.target_output_rpm = g_m3508_pos_left_target_output_rpm;
    g_m3508_pos_debug.right.target_output_rpm = g_m3508_pos_right_target_output_rpm;
}

void M3508_Position_Stop(M3508_PositionSide_t side)
{
    uint8_t index = M3508_Pos_SideToIndex(side);
    uint8_t pid_id = g_m3508_pos_motor[index].pid_motor_id;

    PID_SetTargetSpeed(pid_id, 0);

    if (index == 0) {
        g_m3508_pos_left_target_output_rpm = 0.0f;
        g_m3508_pos_debug.left.target_output_rpm = 0.0f;
        g_m3508_pos_debug.left.rotor_rpm_set = 0.0f;
    } else {
        g_m3508_pos_right_target_output_rpm = 0.0f;
        g_m3508_pos_debug.right.target_output_rpm = 0.0f;
        g_m3508_pos_debug.right.rotor_rpm_set = 0.0f;
    }
}

void M3508_Position_StopAll(void)
{
    M3508_Position_Stop(M3508_POS_LEFT);
    M3508_Position_Stop(M3508_POS_RIGHT);
}

uint8_t M3508_Position_IsTargetReached(M3508_PositionSide_t side)
{
    uint8_t index = M3508_Pos_SideToIndex(side);

    float current =
        M3508_Position_GetCurrentLength(side);

    float target =
        g_m3508_pos_motor[index].target_length_mm;

    return (fabsf(target - current) <= M3508_POS_DEADBAND_MM) ? 1u : 0u;
}

uint8_t M3508_Position_IsAllTargetReached(void)
{
    return (M3508_Position_IsTargetReached(M3508_POS_LEFT) &&
            M3508_Position_IsTargetReached(M3508_POS_RIGHT)) ? 1u : 0u;
}

M3508_PositionDebug_t M3508_Position_GetDebugInfo(void)
{
    M3508_Pos_UpdateOutputRPM();

    g_m3508_pos_debug.left.current_length_mm =
        M3508_Position_GetCurrentLength(M3508_POS_LEFT);

    g_m3508_pos_debug.right.current_length_mm =
        M3508_Position_GetCurrentLength(M3508_POS_RIGHT);

    g_m3508_pos_debug.left.target_length_mm =
        g_m3508_pos_motor[0].target_length_mm;

    g_m3508_pos_debug.right.target_length_mm =
        g_m3508_pos_motor[1].target_length_mm;

    g_m3508_pos_debug.left.output_rpm =
        g_m3508_pos_left_output_rpm;

    g_m3508_pos_debug.right.output_rpm =
        g_m3508_pos_right_output_rpm;

    g_m3508_pos_debug.left.target_output_rpm =
        g_m3508_pos_left_target_output_rpm;

    g_m3508_pos_debug.right.target_output_rpm =
        g_m3508_pos_right_target_output_rpm;

    return g_m3508_pos_debug;
}

float M3508_Position_GetTargetLength(M3508_PositionSide_t side)
{
    uint8_t index = M3508_Pos_SideToIndex(side);
    return g_m3508_pos_motor[index].target_length_mm;
}

float M3508_Position_GetOutputRPM(M3508_PositionSide_t side)
{
    M3508_Pos_UpdateOutputRPM();
    return (side == M3508_POS_RIGHT) ?
           g_m3508_pos_right_output_rpm :
           g_m3508_pos_left_output_rpm;
}

float M3508_Position_GetTargetOutputRPM(M3508_PositionSide_t side)
{
    return (side == M3508_POS_RIGHT) ?
           g_m3508_pos_right_target_output_rpm :
           g_m3508_pos_left_target_output_rpm;
}

void M3508_Position_SetReductionRatio(float ratio)
{
    if (ratio < 1.0f) {
        g_m3508_pos_reduction_ratio = M3508_POS_DEFAULT_REDUCTION_RATIO;
    } else {
        g_m3508_pos_reduction_ratio = ratio;
    }
}

float M3508_Position_GetReductionRatio(void)
{
    return M3508_Pos_GetValidReductionRatio();
}

void M3508_Position_SetBaseLength(M3508_PositionSide_t side, float base_length_mm)
{
    uint8_t index = M3508_Pos_SideToIndex(side);

    g_m3508_pos_motor[index].base_length_mm =
        M3508_Pos_ClampFloat(base_length_mm, g_length_min_mm, g_length_max_mm);
}

void M3508_Position_SetBaseLengthBoth(float left_base_mm, float right_base_mm)
{
    M3508_Position_SetBaseLength(M3508_POS_LEFT, left_base_mm);
    M3508_Position_SetBaseLength(M3508_POS_RIGHT, right_base_mm);
}

void M3508_Position_SetDirectionSign(M3508_PositionSide_t side, int8_t sign)
{
    uint8_t index = M3508_Pos_SideToIndex(side);
    g_m3508_pos_motor[index].direction_sign = M3508_Pos_NormalizeSign(sign);
}

void M3508_Position_SetDirectionSignBoth(int8_t left_sign, int8_t right_sign)
{
    M3508_Position_SetDirectionSign(M3508_POS_LEFT, left_sign);
    M3508_Position_SetDirectionSign(M3508_POS_RIGHT, right_sign);
}

void M3508_Position_SetLengthLimit(float min_mm, float max_mm)
{
    if (max_mm <= min_mm) return;

    g_length_min_mm = min_mm;
    g_length_max_mm = max_mm;
}
