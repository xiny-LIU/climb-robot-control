#include "pwm_angle_servo.h"

#include <math.h>
#include <stddef.h>
#include "string.h"
#include <stdio.h>
#include <stdlib.h>

#include "pwm_motor.h"
#include "spi4.h"

/* ============================================================
 * 角度位置环参数
 * ============================================================ */
/*
 * PWM 角度位置环：
 * u_pwm = Kp * |error|
 *
 * 其中：
 * u_pwm：PWM 速度百分比（0u ~ 100u）；
 * Kp   ：比例系数，决定了误差多大时电机该用多快的速度追赶；
 * error：角度误差，单位 deg（度）。
 */
#define PWM_ANGLE_KP_PERCENT_PER_DEG     3.0f// 比例系数 Kp。意思是：每差 1 度，电机速度就增加 5%

/* 角度到位死区，单位 deg */
#define PWM_ANGLE_DEADBAND_DEG           1.0f// 死区。意思是：只要实际角度和目标角度相差小于 0.8 度，就认为到了，电机停转

/* PWM 电机最小/最大速度百分比 */
#define PWM_ANGLE_MIN_SPEED_PERCENT      8u// 最小速度 18%。低于这个速度，电机可能因为摩擦力根本带不动
#define PWM_ANGLE_MAX_SPEED_PERCENT      50u// 最大速度 70%。超过这个速度，机械臂可能因为太快而失控或撞坏

/* ============================================================
 * 目标角度平滑参数
 * ============================================================ */

/*
 * PWM_AngleServo_Update_5ms() 每 5ms 调用一次，
 * 所以控制周期为 0.005s。
 */
#define PWM_ANGLE_UPDATE_PERIOD_SEC          0.005f

/*
 * 内部目标角度 command_deg 的最大变化速度，单位 deg/s。
 * 这个值不是电机真实速度，而是“目标角度变化速度”。
 */
#define PWM_ANGLE_TARGET_SLEW_RATE_DEG_S     8.0f

/* ============================================================
 * 内部类型
 * ============================================================ */

typedef struct {
    Motor_ID_t motor_id;       // 电机ID（告诉系统这个关节由哪个电机驱动，如 MOTOR_A）
    Encoder_ID_t encoder_id;   // 编码器ID（告诉系统这个关节的数据由哪个传感器读取，如 ENC_1）

    float zero_offset_deg;     // 机械零点偏置（组装时有误差，用来校准的机械零位角度）
    
     /*
     * 编码器方向符号：
     * +1：编码器角度增加方向 = DH 关节角正方向
     * -1：编码器角度增加方向与 DH 关节角正方向相反
     */
    int8_t encoder_sign;

    /*
     * 电机方向符号：
     * +1：DIRECTION_FORWARD 会让 DH 关节角增大
     * -1：DIRECTION_REVERSE 会让 DH 关节角增大
     */
    int8_t direction_sign;

    float target_deg;          // 当前这个关节的目标角度（用户发出的指令）
    float command_deg;         // 内部平滑目标角度，位置环实际追踪它
    float min_deg;             // 该关节允许的最小安全角度（软限位）
    float max_deg;             // 该关节允许的最大安全角度（软限位）
} PWM_AngleJointConfig_t;


/* ============================================================
 * 内部变量
 * ============================================================ */

// 创建一个数组，里面包含 4 个关节的硬件绑定和默认安全角度限制
static PWM_AngleJointConfig_t g_pwm_angle_joint[PWM_ANGLE_JOINT_NUM] = {
    /* 1. 左臂俯仰关节：绑定 MOTOR_A 和 ENC_1 */
    {MOTOR_A, ENC_1, 0.0f, +1, +1, 0.0f, 0.0f,
     PWM_ANGLE_DEFAULT_PITCH_MIN_DEG,
     PWM_ANGLE_DEFAULT_PITCH_MAX_DEG},

    /* 2. 左臂偏航关节：绑定 MOTOR_B 和 ENC_2 */
    {MOTOR_B, ENC_2, 0.0f, +1, +1, 0.0f, 0.0f,
     PWM_ANGLE_LEFT_YAW_MIN_DEG,
     PWM_ANGLE_LEFT_YAW_MAX_DEG},

    /* 3. 右臂俯仰关节：绑定 MOTOR_C 和 ENC_3 */
    {MOTOR_C, ENC_3, 0.0f, +1, +1, 0.0f, 0.0f,
     PWM_ANGLE_DEFAULT_PITCH_MIN_DEG,
     PWM_ANGLE_DEFAULT_PITCH_MAX_DEG},

    /* 4. 右臂偏航关节：绑定 MOTOR_D 和 ENC_4 */
    {MOTOR_D, ENC_4, 0.0f, +1, +1, 0.0f, 0.0f,
     PWM_ANGLE_RIGHT_YAW_MIN_DEG,
     PWM_ANGLE_RIGHT_YAW_MAX_DEG}
};

static uint8_t g_pwm_angle_enabled = 0; // 全局总开关：1代表启动闭环控制，0代表彻底关闭
static PWM_AngleDebug_t g_pwm_angle_debug; // 全局调试变量，用来存在运行过程中的实时状态，方便后台查看


/* ============================================================
 * 工具函数
 * ============================================================ */

/*
 * @brief 将浮点数限制在指定范围内（数字限幅）
 */
static float PWM_Angle_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value; // 如果比最小值还小，就强制等于最小值
    if (value > max_value) return max_value; // 如果比最大值还大，就强制等于最大值
    return value;                            // 在正常范围内，原样返回
}

/*
 * @brief 将数值限制为 uint8 速度百分比（0 ~ 100）
 */
static uint8_t PWM_Angle_ClampU8(uint32_t value, uint32_t min_value, uint32_t max_value)
{
    if (value < min_value) return (uint8_t)min_value;
    if (value > max_value) return (uint8_t)max_value;
    return (uint8_t)value;
}

/*
 * @brief 将方向符号标准化为 +1 或 -1（防止用户输入乱七八糟的数字）
 */
static int8_t PWM_Angle_NormalizeSign(int8_t sign)
{
    return (sign < 0) ? -1 : +1; // 只要是负数就变 -1，只要是正数或0就变 +1
}

/*
 * @brief 将角度归一化到 [-180, 180) 之间
 * * 核心目的：解决圆周运动的“边界跳变”问题。
 * 比如当前在 179 度，目标是 -179 度，它们实际只相差 2 度。
 * 如果直接做减法：-179 - 179 = -358 度，电机就会绕一大圈去追，这就傻了。
 * 经过这个函数处理后，-358 度会自动变成 +2 度，电机会选择最近的路径转过去。
 */
static float PWM_Angle_Normalize180(float angle_deg)
{
    while (angle_deg >= 180.0f) angle_deg -= 360.0f; // 大于180度就减去一圈
    while (angle_deg < -180.0f) angle_deg += 360.0f; // 小于-180度就加上一圈
    return angle_deg;
}

/*
 * @brief 判断关节编号是否合法（防止数组越界死机）
 */
static uint8_t PWM_Angle_IsValidJoint(PWM_AngleJoint_t joint)
{
    return (joint < PWM_ANGLE_JOINT_NUM) ? 1u : 0u; // 编号必须在 0、1、2、3 之中
}

/*
 * @brief 计算目标角度与当前角度之间的【最短距离】误差
 */
static float PWM_Angle_Error(float target_deg, float current_deg)
{
    // 两数直接相减，然后调用归一化工具，确保得到的是 [-180, 180) 之间的最短捷径
    return PWM_Angle_Normalize180(target_deg - current_deg);
}

/*
 * @brief 让当前内部目标角度 current_cmd_deg 以固定最大步长靠近 final_target_deg
 *
 * 作用：
 * final_target_deg 可以突然变化，但 current_cmd_deg 不会突然跳变。
 */
static float PWM_Angle_ApproachAngle(float current_cmd_deg,
                                     float final_target_deg,
                                     float max_step_deg)
{
    float err_deg = PWM_Angle_Error(final_target_deg, current_cmd_deg);

    if (max_step_deg <= 0.0f) {
        return current_cmd_deg;
    }

    if (fabsf(err_deg) <= max_step_deg) {
        return final_target_deg;
    }

    if (err_deg > 0.0f) {
        return PWM_Angle_Normalize180(current_cmd_deg + max_step_deg);
    } else {
        return PWM_Angle_Normalize180(current_cmd_deg - max_step_deg);
    }
}

/* ============================================================
 * 当前角度读取
 * ============================================================ */

float PWM_AngleServo_GetCurrentAngle(PWM_AngleJoint_t joint)
{
    // 1. 安全检查，如果关节编号不对，直接返回0度
    if (!PWM_Angle_IsValidJoint(joint)) return 0.0f;

    // 2. 找到这个关节对应的传感器（编码器）ID
    Encoder_ID_t enc_id = g_pwm_angle_joint[joint].encoder_id;

    // 3. 从全局硬件硬件数组 `encoder_data` 中取出这个传感器的原始度数
    float enc_deg = encoder_data[enc_id].degree;

    // 4. 重点：实际角度 = 传感器读数 - 零点偏置

    // 先计算编码器相对机械零点的角度变化。
    float delta_deg =
        PWM_Angle_Normalize180(enc_deg - g_pwm_angle_joint[joint].zero_offset_deg);
    /*
     * 再根据 encoder_sign 转换成 DH 法定义下的关节角。
     *
     * encoder_sign = +1：
     *     编码器角度增加方向与 DH 正方向一致；
     *
     * encoder_sign = -1：
     *     编码器角度增加方向与 DH 正方向相反。
     */
    float joint_deg =
        PWM_Angle_Normalize180((float)g_pwm_angle_joint[joint].encoder_sign * delta_deg);

    return joint_deg; // 返回计算后的真实关节角度
}


/* ============================================================
 * 单关节角度闭环（核心计算）
 * ============================================================ */
static void PWM_Angle_UpdateOne(PWM_AngleJoint_t joint)
{
    // 1. 安全检查：确认传入的关节编号是否合法
    // 如果 joint 超出 PWM_ANGLE_JOINT_NUM 范围，说明输入非法，直接退出
    if (!PWM_Angle_IsValidJoint(joint)) return;

    // 2. 创建指针 shortcut，方便后续缩写代码
    // cfg 指向该关节的配置参数，比如电机编号、编码器编号、零点偏置、方向符号、目标角度和限位
    // dbg 指向该关节的调试数据，比如当前角度、目标角度、误差、速度百分比和状态
    PWM_AngleJointConfig_t *cfg = &g_pwm_angle_joint[joint];
    PWM_AngleJointDebug_t *dbg = &g_pwm_angle_debug.joint[joint];

    // 3. 获取当前实际角度
    // 内部会读取对应 PQY13 编码器角度，并减去 zero_offset_deg，最后归一化到 [-180, 180)
    float current_deg = PWM_AngleServo_GetCurrentAngle(joint);

    // 4. 获取目标角度，并在执行端再次做限位保护
    // 虽然 PWM_AngleServo_SetTarget() 已经做过一次限位，
    // 但如果后续调用 PWM_AngleServo_SetLimit() 修改了软限位，
    // 原来的目标角度可能已经不在新的限位范围内。
    // 所以这里再次 Clamp 一次，保证真正执行的目标角度一定安全。
    float target_deg = cfg->target_deg;

    // 5. 将限位后的目标角度写回配置结构体
    // 这样 cfg->target_deg 始终保存当前真正执行的安全目标角度
    cfg->target_deg = target_deg;
    
    /*
     * 根据目标变化率计算每个 5ms 周期允许 command_deg 变化的最大角度。
     */
    float max_step_deg =
        PWM_ANGLE_TARGET_SLEW_RATE_DEG_S * PWM_ANGLE_UPDATE_PERIOD_SEC;

    /*
     * command_deg 慢慢靠近 target_deg。
     * 注意：位置环实际追踪 command_deg，而不是直接追踪 target_deg。
     */
    cfg->command_deg =
        PWM_Angle_ApproachAngle(cfg->command_deg, target_deg, max_step_deg);
    
//打印步长    
//        float old_command_deg = cfg->command_deg;

//        cfg->command_deg =
//            PWM_Angle_ApproachAngle(cfg->command_deg, target_deg, max_step_deg);

//        if (joint == PWM_ANGLE_LEFT_PITCH)
//        {
//            printf("[UPD] target=%.3f, old_cmd=%.3f, new_cmd=%.3f, max_step=%.4f, cmd_err=%.3f\r\n",
//                   target_deg,
//                   old_command_deg,
//                   cfg->command_deg,
//                   max_step_deg,
//                   PWM_Angle_Error(target_deg, old_command_deg));
//        }
    
    // 6. 计算当前角度与目标角度之间的误差
    // error_deg = Normalize180(target_deg - current_deg)
    // 这样可以避免 359° 和 1° 被误判为相差 358° 的问题
    float error_deg = PWM_Angle_Error(cfg->command_deg, current_deg);

//取消步长的测试
//    cfg->command_deg = target_deg;

//    float error_deg = PWM_Angle_Error(target_deg, current_deg);

    // 7. 将实时数据登记到“调试看板”，方便串口打印或后续检查
    dbg->current_deg = current_deg;
    dbg->target_deg = target_deg;
    dbg->command_deg = cfg->command_deg;
    dbg->error_deg = error_deg;

    // 8.【死区控制】如果误差绝对值小于设定死区，说明已经到位
    // 此时停止对应电机，并把速度百分比记录为 0
    if (fabsf(error_deg) <= PWM_ANGLE_DEADBAND_DEG) {
        Motor_Stop(cfg->motor_id);// 让对应电机停止输出
        dbg->speed_percent = 0;

        // 9. 更新状态
        // 注意：如果之前目标角度被限位，状态是 PWM_ANGLE_TARGET_LIMITED，
        // 这里不要直接覆盖成 PWM_ANGLE_OK，否则调试时看不到“目标被限位”的信息。
        // 只有在当前状态不是 TARGET_LIMITED 时，才标记为 OK。
        if (dbg->status != PWM_ANGLE_TARGET_LIMITED) {
            dbg->status = PWM_ANGLE_OK;
        }

        return;
    }

    // 10.【计算速度】如果还没到位，则根据角度误差计算速度百分比
    // 计算公式：
    // speed_f = |error_deg| * PWM_ANGLE_KP_PERCENT_PER_DEG
    //
    // 其中：
    // speed_f 是未限幅前的 PWM 速度百分比；
    // error_deg 是角度误差，单位 deg；
    // PWM_ANGLE_KP_PERCENT_PER_DEG 是比例系数，表示每 1° 误差对应多少速度百分比。
    float speed_f = fabsf(error_deg) * PWM_ANGLE_KP_PERCENT_PER_DEG;

    // 11.【限速保护】把算出来的速度限制在最小速度和最大速度之间
    // 最小速度用于克服静摩擦，防止误差存在但电机转不动；
    // 最大速度用于防止速度过大导致冲过目标或机械冲击。
    uint8_t speed_percent =
        PWM_Angle_ClampU8((uint32_t)speed_f,
                          PWM_ANGLE_MIN_SPEED_PERCENT,
                          PWM_ANGLE_MAX_SPEED_PERCENT);

    // 12.【判断方向】
    // error_deg 表示目标角度相对于当前角度的方向；
    // cfg->direction_sign 表示实际电机安装方向与DH关节角增加方向之间的关系。
    //
    // 如果 direction_sign = +1：
    //     error_deg > 0 时，DIRECTION_FORWARD 应该让角度增大；
    //
    // 如果 direction_sign = -1：
    //     error_deg > 0 时，DIRECTION_REVERSE 才能让角度增大。
    //
    // 所以使用 error_deg * direction_sign 判断最终应该正转还是反转。
    Motor_Direction_t dir =
        ((error_deg * (float)cfg->direction_sign) > 0.0f) ?
        DIRECTION_FORWARD :
        DIRECTION_REVERSE;

    // 13.【执行动作】向底层硬件发送方向和速度指令
    // Motor_SetDirection() 设置电机方向；
    // Motor_SetSpeed() 设置 PWM 速度百分比，并启动对应电机。
    Motor_SetDirection(cfg->motor_id, dir);
    Motor_SetSpeed(cfg->motor_id, speed_percent);

    // 14. 更新调试数据中的速度百分比
    dbg->speed_percent = speed_percent;

    // 15. 更新状态
    // 同样，为了保留“目标角度曾经被限位”的调试信息，
    // 这里不要无条件写 PWM_ANGLE_OK。
    if (dbg->status != PWM_ANGLE_TARGET_LIMITED) {
        dbg->status = PWM_ANGLE_OK;
    }
}


/* ============================================================
 * 对外接口
 * ============================================================ */

// 1. 初始化模块
void PWM_AngleServo_Init(void)
{
    g_pwm_angle_enabled = 0;

    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].motor_id = MOTOR_A;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].encoder_id = ENC_1;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].zero_offset_deg = 20.0f;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].encoder_sign = +1;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].direction_sign = -1;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].target_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].command_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].min_deg = PWM_ANGLE_DEFAULT_PITCH_MIN_DEG;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_PITCH].max_deg = PWM_ANGLE_DEFAULT_PITCH_MAX_DEG;

    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].motor_id = MOTOR_B;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].encoder_id = ENC_2;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].zero_offset_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].encoder_sign = +1;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].direction_sign = +1;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].target_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].command_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].min_deg = PWM_ANGLE_LEFT_YAW_MIN_DEG;
    g_pwm_angle_joint[PWM_ANGLE_LEFT_YAW].max_deg = PWM_ANGLE_LEFT_YAW_MAX_DEG;

    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].motor_id = MOTOR_C;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].encoder_id = ENC_3;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].zero_offset_deg = 10.0f;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].encoder_sign = -1;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].direction_sign = +1;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].target_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].command_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].min_deg = PWM_ANGLE_DEFAULT_PITCH_MIN_DEG;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_PITCH].max_deg = PWM_ANGLE_DEFAULT_PITCH_MAX_DEG;

    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].motor_id = MOTOR_D;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].encoder_id = ENC_4;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].zero_offset_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].encoder_sign = +1;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].direction_sign = +1;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].target_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].command_deg = 0.0f;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].min_deg = PWM_ANGLE_RIGHT_YAW_MIN_DEG;
    g_pwm_angle_joint[PWM_ANGLE_RIGHT_YAW].max_deg = PWM_ANGLE_RIGHT_YAW_MAX_DEG;

// 清空所有调试数据
    for (uint8_t i = 0; i < PWM_ANGLE_JOINT_NUM; i++) {
        g_pwm_angle_debug.joint[i].current_deg = 0.0f;
        g_pwm_angle_debug.joint[i].target_deg = 0.0f;
        g_pwm_angle_debug.joint[i].command_deg = 0.0f;
        g_pwm_angle_debug.joint[i].error_deg = 0.0f;
        g_pwm_angle_debug.joint[i].speed_percent = 0;
        g_pwm_angle_debug.joint[i].status = PWM_ANGLE_DISABLED;
    }
// 安全起见，上电初始化最后一步：强制让 4 个电机全部停止
    PWM_AngleServo_StopAll();
}

/**
 * @brief 2.锁死：将当前所有机械臂关节的实际位置，直接设为目标角度（就地锁死）
 * @note  调用前提：必须确保底层编码器数据已经至少通过 SPI 成功刷新过一次！
 */
void PWM_AngleServo_LockCurrentPosition(void)
{
    for (uint8_t i = 0; i < PWM_ANGLE_JOINT_NUM; i++) {
        PWM_AngleJoint_t joint = (PWM_AngleJoint_t)i;
        PWM_AngleJointConfig_t *cfg = &g_pwm_angle_joint[joint];
        PWM_AngleJointDebug_t *dbg = &g_pwm_angle_debug.joint[joint];

        float current_pos = PWM_AngleServo_GetCurrentAngle(joint);

        /*
         * 锁死函数的核心原则：
         * 当前在哪里，就锁在哪里。
         * 不在这里做软限位裁剪。
         */
        cfg->target_deg = current_pos;
        cfg->command_deg = current_pos;

        dbg->current_deg = current_pos;
        dbg->target_deg = current_pos;
        dbg->command_deg = current_pos;
        dbg->error_deg = 0.0f;
        dbg->speed_percent = 0;

        dbg->status = g_pwm_angle_enabled ? PWM_ANGLE_OK : PWM_ANGLE_DISABLED;

        Motor_Stop(cfg->motor_id);
    }
}
//3. 总开关使能接口
void PWM_AngleServo_Enable(uint8_t enable)
{
    g_pwm_angle_enabled = enable ? 1u : 0u;// 更新全局开关状态

    if (!g_pwm_angle_enabled) {
        // 如果用户下达了“关闭总开关”的指令（enable = 0）
        PWM_AngleServo_StopAll(); // 必须立刻切断所有电机的动力，确保绝对安全

        for (uint8_t i = 0; i < PWM_ANGLE_JOINT_NUM; i++) {
            g_pwm_angle_debug.joint[i].status = PWM_ANGLE_DISABLED;// 标记状态为未使能
        }
    }
}

uint8_t PWM_AngleServo_IsEnabled(void)
{
    return g_pwm_angle_enabled;
}

//4. 输入目标角度（外界大脑指挥机械臂的窗口）
void PWM_AngleServo_SetTarget(PWM_AngleJoint_t joint, float target_deg)
{
    if (!PWM_Angle_IsValidJoint(joint)) return;

    // 【安全软限位】用户输入的角度绝对不能超过硬件安全范围！
    // 比如：用户手滑输入了 100 度，通过 ClampFloat 会被强行斩断并锁死在 max_deg（例如 30 度）
    float limited =
        PWM_Angle_ClampFloat(target_deg,
                             g_pwm_angle_joint[joint].min_deg,
                             g_pwm_angle_joint[joint].max_deg);

    // 将安全合法的角度存入关节档案
    g_pwm_angle_joint[joint].target_deg = limited;
    g_pwm_angle_debug.joint[joint].target_deg = limited;

    // 如果发现用户的输入被我们强制截断了，就在状态里亮起黄灯告警
    if (limited != target_deg) {
        g_pwm_angle_debug.joint[joint].status = PWM_ANGLE_TARGET_LIMITED; // 目标被限位截断
    } else {
        g_pwm_angle_debug.joint[joint].status = PWM_ANGLE_OK; // 正常
    }
    
    //打印当前command_deg
//    printf("[SET] joint=%d, input=%.3f, limited=%.3f, target=%.3f, command=%.3f\r\n",
//       joint,
//       target_deg,
//       limited,
//       g_pwm_angle_joint[joint].target_deg,
//       g_pwm_angle_joint[joint].command_deg);
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
//5. 5ms 核心定时更新（整个系统的“心脏跳动”）
void PWM_AngleServo_Update_5ms(void)
{
    // 检查总开关，如果总开关是关闭的，什么都不做，直接停机退出
    if (!g_pwm_angle_enabled) {
        PWM_AngleServo_StopAll();

        for (uint8_t i = 0; i < PWM_ANGLE_JOINT_NUM; i++) {
            g_pwm_angle_debug.joint[i].status = PWM_ANGLE_DISABLED;
        }
        return;
    }
// 如果开关开着，每隔 5毫秒 就会依次把 4 个关节的控制算法全部刷新一遍
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
    float command = g_pwm_angle_joint[joint].command_deg;

    float err_current = fabsf(PWM_Angle_Error(target, current));
    float err_command = fabsf(PWM_Angle_Error(target, command));

    return ((err_current <= PWM_ANGLE_DEADBAND_DEG) &&
            (err_command <= PWM_ANGLE_DEADBAND_DEG)) ? 1u : 0u;
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
/* * @brief 获取系统的调试信息看板（修复后的安全版本）
 */
/* 【修复】改用指针传递，避免大结构体复制引发的栈溢出 */
void PWM_AngleServo_GetDebugInfo(PWM_AngleDebug_t *out_debug)
{
    if (out_debug == NULL) return;// 防呆设计：如果传进来的是空指针，直接拒绝服务，防止死机
    // 在被调用的瞬间，再对 4 个关节的最新传感器数据进行一次抓取和更新
    for (uint8_t i = 0; i < PWM_ANGLE_JOINT_NUM; i++) {
        PWM_AngleJoint_t joint = (PWM_AngleJoint_t)i;

    float current = PWM_AngleServo_GetCurrentAngle(joint);
    float target = g_pwm_angle_joint[i].target_deg;
    float command = g_pwm_angle_joint[i].command_deg;
    float error = PWM_Angle_Error(command, current);

    g_pwm_angle_debug.joint[i].current_deg = current;
    g_pwm_angle_debug.joint[i].target_deg = target;
    g_pwm_angle_debug.joint[i].command_deg = command;
    g_pwm_angle_debug.joint[i].error_deg = error;
    }
    /* * 核心操作：*out_debug = g_pwm_angle_debug;
     * 意思是：通过外面传进来的“地址盒子”，把写好的调试看板数据，直接递到外面的变量里。
     * 避免了在内存中产生巨大的临时复制，运行效率极高！
     */
    /* 将全局的 debug 数据拷贝到外部指定的结构体中 */
    *out_debug = g_pwm_angle_debug;
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
void PWM_AngleServo_SetEncoderSign(PWM_AngleJoint_t joint, int8_t sign)
{
    if (!PWM_Angle_IsValidJoint(joint)) return;
    g_pwm_angle_joint[joint].encoder_sign = PWM_Angle_NormalizeSign(sign);
}
void PWM_AngleServo_SetEncoderSignAll(int8_t left_pitch_sign,
                                      int8_t left_yaw_sign,
                                      int8_t right_pitch_sign,
                                      int8_t right_yaw_sign)
{
    PWM_AngleServo_SetEncoderSign(PWM_ANGLE_LEFT_PITCH, left_pitch_sign);
    PWM_AngleServo_SetEncoderSign(PWM_ANGLE_LEFT_YAW, left_yaw_sign);
    PWM_AngleServo_SetEncoderSign(PWM_ANGLE_RIGHT_PITCH, right_pitch_sign);
    PWM_AngleServo_SetEncoderSign(PWM_ANGLE_RIGHT_YAW, right_yaw_sign);
}
void PWM_AngleServo_SetLimit(PWM_AngleJoint_t joint, float min_deg, float max_deg)
{
    if (!PWM_Angle_IsValidJoint(joint)) return;
    if (max_deg <= min_deg) return;

    g_pwm_angle_joint[joint].min_deg = min_deg;
    g_pwm_angle_joint[joint].max_deg = max_deg;

    float limited_target =
        PWM_Angle_ClampFloat(g_pwm_angle_joint[joint].target_deg,
                             min_deg,
                             max_deg);

    float limited_command =
        PWM_Angle_ClampFloat(g_pwm_angle_joint[joint].command_deg,
                             min_deg,
                             max_deg);

    if (limited_target != g_pwm_angle_joint[joint].target_deg) {
        g_pwm_angle_debug.joint[joint].status = PWM_ANGLE_TARGET_LIMITED;
    }

    g_pwm_angle_joint[joint].target_deg = limited_target;
    g_pwm_angle_joint[joint].command_deg = limited_command;

    g_pwm_angle_debug.joint[joint].target_deg = limited_target;
    g_pwm_angle_debug.joint[joint].command_deg = limited_command;
}

/************************************************
 * 当前角度设为零点功能
 ************************************************/

void PWM_AngleServo_SetCurrentAsZero(PWM_AngleJoint_t joint)
{
    // 1. 安全检查：防止关节编号非法导致数组越界
    if (!PWM_Angle_IsValidJoint(joint)) return;

    // 2. 找到该关节对应的编码器 ID
    Encoder_ID_t enc_id = g_pwm_angle_joint[joint].encoder_id;

    // 3. 读取当前绝对式编码器角度
    // 绝对式编码器上电后读数通常不是 0°，
    // 这里直接把当前读数作为新的 zero_offset。
    float current_encoder_deg = encoder_data[enc_id].degree;

    // 4. 将当前编码器角度设置为该关节的零点偏置
    // 之后 PWM_AngleServo_GetCurrentAngle(joint) 的计算结果就是：
    // Normalize180(encoder_degree - current_encoder_deg)
    // 所以当前姿态会被定义为 0°。
    g_pwm_angle_joint[joint].zero_offset_deg = current_encoder_deg;

    // 5. 将目标角度也设置为 0°
    // 这样可以防止设完零点后，电机因为旧目标角度突然运动。
    g_pwm_angle_joint[joint].target_deg = 0.0f;
    g_pwm_angle_joint[joint].command_deg = 0.0f;

    // 6. 更新调试信息
    g_pwm_angle_debug.joint[joint].current_deg = 0.0f;
    g_pwm_angle_debug.joint[joint].target_deg = 0.0f;
    g_pwm_angle_debug.joint[joint].command_deg = 0.0f;
    g_pwm_angle_debug.joint[joint].error_deg = 0.0f;
    g_pwm_angle_debug.joint[joint].speed_percent = 0;

    // 7. 根据模块当前使能状态设置状态标志
    if (g_pwm_angle_enabled) {
        g_pwm_angle_debug.joint[joint].status = PWM_ANGLE_OK;
    } else {
        g_pwm_angle_debug.joint[joint].status = PWM_ANGLE_DISABLED;
    }

    // 8. 安全起见，设零点后先停止该关节电机
    Motor_Stop(g_pwm_angle_joint[joint].motor_id);
}


void PWM_AngleServo_SetAllCurrentAsZero(void)
{
    // 将左俯仰当前位置设为 0°
    PWM_AngleServo_SetCurrentAsZero(PWM_ANGLE_LEFT_PITCH);

    // 将左偏航当前位置设为 0°
    PWM_AngleServo_SetCurrentAsZero(PWM_ANGLE_LEFT_YAW);

    // 将右俯仰当前位置设为 0°
    PWM_AngleServo_SetCurrentAsZero(PWM_ANGLE_RIGHT_PITCH);

    // 将右偏航当前位置设为 0°
    PWM_AngleServo_SetCurrentAsZero(PWM_ANGLE_RIGHT_YAW);
}









