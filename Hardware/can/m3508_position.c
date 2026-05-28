#include "m3508_position.h"
#include "math.h"
#include "PID.h"
#include "encoder_counter.h"
#include "CAN_receive.h"
#include "stm32f4xx.h"

/* * 提示：__disable_irq() 和 __get_PRIMASK() 是 ARM CMSIS 的标准内核指令。
 * 在 STM32 标配环境下（HAL库或标准库），它们通常已经在核心头文件中声明。
 * 如果编译器报错找不到对应的内联函数，请取消注释下方这行：
 * #include "cmsis_compiler.h" 
 */

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
 * 长度位置环：v = Kp * e
 * v  ：目标伸缩线速度，单位 mm/s；
 * Kp ：长度位置环比例系数，单位 1/s；
 * e  ：伸缩长度误差，单位 mm。
 */
#define M3508_POS_KP_MM_S_PER_MM        1.2f

/* 伸缩长度到位死区，单位 mm */
#define M3508_POS_DEADBAND_MM           2.0f

/* 速度环目标转子转速限幅，单位 rpm */
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
 * 内部类型与变量
 * ============================================================ */
typedef struct {
    uint8_t pid_motor_id;       /* 0=左伸缩，1=右伸缩 */
    float base_length_mm;       /* 编码器清零时对应的伸缩杆长度 */
    float target_length_mm;     /* 原始目标伸缩长度（跨线程共享） */
    int8_t direction_sign;      /* 伸缩方向符号，+1 或 -1 */
} M3508_PositionMotor_t;

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
static float M3508_Pos_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static int8_t M3508_Pos_NormalizeSign(int8_t sign)
{
    return (sign < 0) ? -1 : +1;
}

static float M3508_Pos_GetValidReductionRatio(void)
{
    if (g_m3508_pos_reduction_ratio < 1.0f) {
        return M3508_POS_DEFAULT_REDUCTION_RATIO;
    }
    return g_m3508_pos_reduction_ratio;
}

static uint8_t M3508_Pos_SideToIndex(M3508_PositionSide_t side)
{
    return (side == M3508_POS_RIGHT) ? 1u : 0u;
 }

/* ============================================================
 * 速度与长度换算
 * ============================================================ */
static void M3508_Pos_UpdateOutputRPM(void)
{
    float ratio = M3508_Pos_GetValidReductionRatio();

    const motor_measure_t *left_motor = get_chassis_motor_measure_point(g_m3508_pos_motor[0].pid_motor_id);
    const motor_measure_t *right_motor = get_chassis_motor_measure_point(g_m3508_pos_motor[1].pid_motor_id);

    g_m3508_pos_left_output_rpm = (float)left_motor->speed_rpm / ratio;
    g_m3508_pos_right_output_rpm = (float)right_motor->speed_rpm / ratio;
}

float M3508_Position_GetCurrentLength(M3508_PositionSide_t side)
{
    uint8_t index = M3508_Pos_SideToIndex(side);

    int32_t total_angle_count = Encoder_Get_Total_Angle(g_m3508_pos_motor[index].pid_motor_id);

    float rotor_turns = (float)total_angle_count / 8192.0f;
    float ratio = M3508_Pos_GetValidReductionRatio();
    float output_turns = rotor_turns / ratio;

    float delta_mm = output_turns * M3508_POS_PI * M3508_POS_PULLEY_DIAMETER_MM;
    float d3 = g_m3508_pos_motor[index].base_length_mm + (float)g_m3508_pos_motor[index].direction_sign * delta_mm;

    return d3;
}

/* ============================================================
 * 单侧位置闭环核心控制（传入本地快照值）
 * ============================================================ */
static void M3508_Pos_UpdateOne(uint8_t index, float local_target_mm)
{
    if (index > 1) return;

    uint8_t pid_id = g_m3508_pos_motor[index].pid_motor_id;
    M3508_PositionMotorDebug_t *dbg = (index == 0) ? &g_m3508_pos_debug.left : &g_m3508_pos_debug.right;

    // 1. 获取当前实际机械长度
    float current_mm = M3508_Position_GetCurrentLength((index == 0) ? M3508_POS_LEFT : M3508_POS_RIGHT);
    
    // 2. 软件安全限位截断，并合理判定与记录限位状态（解决状态覆盖问题）
    float clamped_target_mm = M3508_Pos_ClampFloat(local_target_mm, g_length_min_mm, g_length_max_mm);
    if (clamped_target_mm != local_target_mm) {
        dbg->status = M3508_POS_TARGET_LIMITED;
    } else {
        dbg->status = M3508_POS_OK;
    }

    // 3. 计算实际控制误差
    float error_mm = clamped_target_mm - current_mm;

    dbg->current_length_mm = current_mm;
    dbg->target_length_mm = clamped_target_mm;
    dbg->error_mm = error_mm;
    
    // 4. 死区连续性映射（消除突变，彻底解决边界高频振荡）
    float control_error = 0.0f;
    if (error_mm > M3508_POS_DEADBAND_MM) {
        control_error = error_mm - M3508_POS_DEADBAND_MM;
    } else if (error_mm < -M3508_POS_DEADBAND_MM) {
        control_error = error_mm + M3508_POS_DEADBAND_MM;
    } else {
        control_error = 0.0f; // 彻底进入死区，完全静止
    }

    if (control_error == 0.0f) {
        PID_SetTargetSpeed(pid_id, 0);
        dbg->target_output_rpm = 0.0f;
        dbg->rotor_rpm_set = 0.0f;
        
        if (index == 0) g_m3508_pos_left_target_output_rpm = 0.0f;
        else g_m3508_pos_right_target_output_rpm = 0.0f;
        return;
    }

    // 5. 外层位置环 P 控制 (计算目标线速度 mm/s)
    float v_mm_s = M3508_POS_KP_MM_S_PER_MM * control_error;

    // 6. 线速度 -> 输出轴目标转速 RPM
    float output_rpm = v_mm_s / (M3508_POS_PI * M3508_POS_PULLEY_DIAMETER_MM) * 60.0f;
    float target_output_rpm = output_rpm * (float)g_m3508_pos_motor[index].direction_sign;

    float ratio = M3508_Pos_GetValidReductionRatio();

    float rotor_rpm_set_f = target_output_rpm * ratio;

    rotor_rpm_set_f =
        M3508_Pos_ClampFloat(rotor_rpm_set_f,
                             -M3508_POS_MAX_ROTOR_RPM,
                             M3508_POS_MAX_ROTOR_RPM);

    int16_t rotor_rpm_set = (int16_t)rotor_rpm_set_f;

    /*
     * 反算“实际执行的目标输出轴转速”。
     * 这样 debug 和实际控制量一致。
     */
    float actual_target_output_rpm =
        (float)rotor_rpm_set / ratio;

    if (index == 0) {
        g_m3508_pos_left_target_output_rpm = actual_target_output_rpm;
    } else {
        g_m3508_pos_right_target_output_rpm = actual_target_output_rpm;
    }

    PID_SetTargetSpeed(pid_id, rotor_rpm_set);

    dbg->target_output_rpm = actual_target_output_rpm;
    dbg->rotor_rpm_set = (float)rotor_rpm_set;
}

/* ============================================================
 * 对外控制接口
 * ============================================================ */
void M3508_Position_Init(void)
{
    g_m3508_pos_enabled = 0;
    g_m3508_pos_reduction_ratio = M3508_POS_DEFAULT_REDUCTION_RATIO;

    g_m3508_pos_motor[0].pid_motor_id = 0;
    g_m3508_pos_motor[0].base_length_mm = M3508_POS_DEFAULT_D3_MIN_MM;
    g_m3508_pos_motor[0].direction_sign = +1;

    g_m3508_pos_motor[1].pid_motor_id = 1;
    g_m3508_pos_motor[1].base_length_mm = M3508_POS_DEFAULT_D3_MIN_MM;
    g_m3508_pos_motor[1].direction_sign = +1;

    g_length_min_mm = M3508_POS_DEFAULT_D3_MIN_MM;
    g_length_max_mm = M3508_POS_DEFAULT_D3_MAX_MM;

    /*
     * 在 pid_id、base_length、direction_sign 都设置好之后，
     * 再同步当前长度为目标长度。
     */
    g_m3508_pos_motor[0].target_length_mm =
        M3508_Position_GetCurrentLength(M3508_POS_LEFT);

    g_m3508_pos_motor[1].target_length_mm =
        M3508_Position_GetCurrentLength(M3508_POS_RIGHT);

    M3508_Pos_UpdateOutputRPM();

    g_m3508_pos_debug.left.current_length_mm =
        g_m3508_pos_motor[0].target_length_mm;
    g_m3508_pos_debug.left.target_length_mm =
        g_m3508_pos_motor[0].target_length_mm;
    g_m3508_pos_debug.left.error_mm = 0.0f;

    g_m3508_pos_debug.right.current_length_mm =
        g_m3508_pos_motor[1].target_length_mm;
    g_m3508_pos_debug.right.target_length_mm =
        g_m3508_pos_motor[1].target_length_mm;
    g_m3508_pos_debug.right.error_mm = 0.0f;

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

    /*
     * 建议在输入端也做限位。
     * 这样 g_m3508_pos_motor[index].target_length_mm 永远保存安全目标。
     */
    float limited_target =
        M3508_Pos_ClampFloat(target_length_mm, g_length_min_mm, g_length_max_mm);

    uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();

    g_m3508_pos_motor[index].target_length_mm = limited_target;

    if (index == 0) {
        g_m3508_pos_debug.left.target_length_mm = limited_target;
        if (limited_target != target_length_mm) {
            g_m3508_pos_debug.left.status = M3508_POS_TARGET_LIMITED;
        }
    } else {
        g_m3508_pos_debug.right.target_length_mm = limited_target;
        if (limited_target != target_length_mm) {
            g_m3508_pos_debug.right.status = M3508_POS_TARGET_LIMITED;
        }
    }

    __set_PRIMASK(primask_bit);
}

void M3508_Position_SetTargetLengthBoth(float left_target_mm, float right_target_mm)
{
    M3508_Position_SetTargetLength(M3508_POS_LEFT, left_target_mm);
    M3508_Position_SetTargetLength(M3508_POS_RIGHT, right_target_mm);
}

/* ============================================================
 * 5ms 周期闭环更新线程（核心异步线程安全改造）
 * ============================================================ */
void M3508_Position_Update_5ms(void)
{
    M3508_Pos_UpdateOutputRPM();

    if (!g_m3508_pos_enabled) {
        M3508_Position_StopAll();
        g_m3508_pos_debug.left.status = M3508_POS_DISABLED;
        g_m3508_pos_debug.right.status = M3508_POS_DISABLED;
        return;
    }

    float local_left_target = 0.0f;
    float local_right_target = 0.0f;

    /* --------------------------------------------------------
     * 【临界区保护】为高层跨线程输入的 target_length_mm 拍照存本地快照
     * -------------------------------------------------------- */
    uint32_t primask_bit = __get_PRIMASK(); 
    __disable_irq();                        
    
    local_left_target  = g_m3508_pos_motor[0].target_length_mm;
    local_right_target = g_m3508_pos_motor[1].target_length_mm;
    
    __set_PRIMASK(primask_bit);             
    /* -------------------------------------------------------- */

    uint8_t left_pid_id  = g_m3508_pos_motor[0].pid_motor_id;
    uint8_t right_pid_id = g_m3508_pos_motor[1].pid_motor_id;

    // 左电机控制分配
    if (PID_GetMotorStatus(left_pid_id) == MOTOR_STATUS_ERROR) {
        M3508_Position_Stop(M3508_POS_LEFT);
        g_m3508_pos_debug.left.status = M3508_POS_MOTOR_ERROR;
    } else {
        M3508_Pos_UpdateOne(0, local_left_target); 
    }

    // 右电机控制分配
    if (PID_GetMotorStatus(right_pid_id) == MOTOR_STATUS_ERROR) {
        M3508_Position_Stop(M3508_POS_RIGHT);
        g_m3508_pos_debug.right.status = M3508_POS_MOTOR_ERROR;
    } else {
        M3508_Pos_UpdateOne(1, local_right_target); 
    }

    // 统一同步一轮全局反馈状态
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
    float current = M3508_Position_GetCurrentLength(side);
    
    float raw_target;

    uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();
    raw_target = g_m3508_pos_motor[index].target_length_mm;
    __set_PRIMASK(primask_bit);

    float target =
        M3508_Pos_ClampFloat(raw_target, g_length_min_mm, g_length_max_mm);

    return (fabsf(target - current) <= M3508_POS_DEADBAND_MM) ? 1u : 0u;
}

uint8_t M3508_Position_IsAllTargetReached(void)
{
    return (M3508_Position_IsTargetReached(M3508_POS_LEFT) &&
            M3508_Position_IsTargetReached(M3508_POS_RIGHT)) ? 1u : 0u;
}

/* ============================================================
 * 线程安全的数据导出接口（彻底消除 Torn Read 隐患）
 * ============================================================ */
void M3508_Position_GetDebugInfo(M3508_PositionDebug_t *out_debug)
{
    if (out_debug == NULL) return;

    uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();

    *out_debug = g_m3508_pos_debug;

    __set_PRIMASK(primask_bit);
}

float M3508_Position_GetTargetLength(M3508_PositionSide_t side)
{
    uint8_t index = M3508_Pos_SideToIndex(side);
    float val;
    
    uint32_t primask_bit = __get_PRIMASK();
    __disable_irq();
    val = g_m3508_pos_motor[index].target_length_mm;
    __set_PRIMASK(primask_bit);
    
    return val;
}

float M3508_Position_GetOutputRPM(M3508_PositionSide_t side)
{
    return (side == M3508_POS_RIGHT) ? g_m3508_pos_right_output_rpm : g_m3508_pos_left_output_rpm;
}

float M3508_Position_GetTargetOutputRPM(M3508_PositionSide_t side)
{
    return (side == M3508_POS_RIGHT) ? g_m3508_pos_right_target_output_rpm : g_m3508_pos_left_target_output_rpm;
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
    g_m3508_pos_motor[index].base_length_mm = M3508_Pos_ClampFloat(base_length_mm, g_length_min_mm, g_length_max_mm);
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

