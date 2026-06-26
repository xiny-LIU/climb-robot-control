#include "climb_control.h"

#include "math.h"
#include "analysis_data.h"
#include "m3508_position.h"
#include "pwm_angle_servo.h"
#include "usart6.h"

#define CLIMB_PI                 3.14159265358979323846f
#define CLIMB_EPS                1.0e-5f
#define CLIMB_BODY_BASE_DX_MM    0.0f
#define CLIMB_DEFAULT_STEP_MM    150.0f
#define CLIMB_BIAS_Y_MM          25.0f
#define CLIMB_PULL_RATE_MM_TICK  0.35f
#define CLIMB_SINGULAR_EPS      1.0e-3f

/* DH 参数以正逆运动学模型为准，左右 L4 不再按旧 climb_control 写法取值。 */
static const float L1 = 25.21f;
static const float L2 = 8.65f;
static const float L3 = 5.66f;
static const float L4_LEFT = 0.59f;
static const float L4_RIGHT = 6.21f;

Climb_Robot_t robot;

typedef struct {
    float m[3][3];
} Climb_Matrix3_t;

/**
 * @brief 角度转弧度。
 * @param deg 角度值，单位 deg。
 * @return 弧度值，单位 rad。
 */
static float Deg2Rad(float deg)
{
    return deg * CLIMB_PI / 180.0f;
}

/**
 * @brief 弧度转角度。
 * @param rad 弧度值，单位 rad。
 * @return 角度值，单位 deg。
 */
static float Rad2Deg(float rad)
{
    return rad * 180.0f / CLIMB_PI;
}

/**
 * @brief 浮点限幅工具。
 * @param value 输入值。
 * @param min_value 下限。
 * @param max_value 上限。
 * @return 限幅后的值。
 */
static float Clamp_Float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

/**
 * @brief 三维向量加法。
 */
static Point3D_t Point_Add(Point3D_t a, Point3D_t b)
{
    Point3D_t out;
    out.x = a.x + b.x;
    out.y = a.y + b.y;
    out.z = a.z + b.z;
    return out;
}

/**
 * @brief 三维向量减法。
 */
static Point3D_t Point_Sub(Point3D_t a, Point3D_t b)
{
    Point3D_t out;
    out.x = a.x - b.x;
    out.y = a.y - b.y;
    out.z = a.z - b.z;
    return out;
}

/**
 * @brief 按左右臂选择对应的 DH 参数 L4。
 * @param side 左右臂侧别。
 * @return L4，单位 mm。
 */
static float Climb_GetL4(Climb_Side_t side)
{
    return (side == CLIMB_SIDE_RIGHT) ? L4_RIGHT : L4_LEFT;
}

/**
 * @brief 获取单臂基座原点在机身坐标系 {B} 下的安装偏置。
 * @param side 左右臂侧别。
 * @return 基座偏置向量，单位 mm。
 */
static Point3D_t Climb_GetBaseOffsetB(Climb_Side_t side)
{
    Point3D_t p;

    /*
     * 机身坐标系 {B} 原点位于两支撑轮连线中点。
     * 左右臂基座与机身平行安装，只体现 X 共同偏置和 Y 方向左右半距。
     */
    p.x = CLIMB_BODY_BASE_DX_MM;
    p.y = (side == CLIMB_SIDE_LEFT) ? (BODY_L_W * 0.5f) : (-BODY_L_W * 0.5f);
    p.z = 0.0f;
    return p;
}

/**
 * @brief 获取左右臂状态结构体指针。
 * @param side 左右臂侧别。
 * @return 对应侧状态结构体指针。
 */
static Climb_ArmState_t *Climb_GetArm(Climb_Side_t side)
{
    return (side == CLIMB_SIDE_RIGHT) ? &robot.right : &robot.left;
}

/**
 * @brief 根据当前 IMU 欧拉角构造机身到世界的旋转矩阵 R_WB。
 * @return 3x3 旋转矩阵。
 */
static Climb_Matrix3_t Climb_GetRotationWB(void)
{
    Climb_Matrix3_t r;
    float roll = Deg2Rad(robot.roll_deg);
    float pitch = Deg2Rad(robot.pitch_deg);
    float yaw = Deg2Rad(robot.yaw_deg);

    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    /*
     * Z-Y-X 欧拉角旋转矩阵。
     * 作用方向为：把机身坐标系 {B} 下的向量转换到世界坐标系 {W}。
     */
    r.m[0][0] = cy * cp;
    r.m[0][1] = cy * sp * sr - sy * cr;
    r.m[0][2] = cy * sp * cr + sy * sr;

    r.m[1][0] = sy * cp;
    r.m[1][1] = sy * sp * sr + cy * cr;
    r.m[1][2] = sy * sp * cr - cy * sr;

    r.m[2][0] = -sp;
    r.m[2][1] = cp * sr;
    r.m[2][2] = cp * cr;

    return r;
}

/**
 * @brief 矩阵乘向量：out = R * p。
 * @param r 3x3 矩阵。
 * @param p 三维向量。
 * @return 乘积向量。
 */
static Point3D_t Climb_MulMatVec(Climb_Matrix3_t r, Point3D_t p)
{
    Point3D_t out;
    out.x = r.m[0][0] * p.x + r.m[0][1] * p.y + r.m[0][2] * p.z;
    out.y = r.m[1][0] * p.x + r.m[1][1] * p.y + r.m[1][2] * p.z;
    out.z = r.m[2][0] * p.x + r.m[2][1] * p.y + r.m[2][2] * p.z;
    return out;
}

/**
 * @brief 转置矩阵乘向量：out = R^T * p。
 * @param r 3x3 矩阵。
 * @param p 三维向量。
 * @return 乘积向量。
 */
static Point3D_t Climb_MulMatTVec(Climb_Matrix3_t r, Point3D_t p)
{
    Point3D_t out;
    out.x = r.m[0][0] * p.x + r.m[1][0] * p.y + r.m[2][0] * p.z;
    out.y = r.m[0][1] * p.x + r.m[1][1] * p.y + r.m[2][1] * p.z;
    out.z = r.m[0][2] * p.x + r.m[1][2] * p.y + r.m[2][2] * p.z;
    return out;
}

/**
 * @brief 从 IMU 输出缓存同步当前机身姿态。
 * @note 姿态角单位为 deg；若 IMU 指针无效，则姿态清零。
 */
static void Climb_UpdateIMUAttitude(void)
{
    protocol_info_t *imu = IMU_GetOutputInfo();

    if (imu == 0) {
        robot.roll_deg = 0.0f;
        robot.pitch_deg = 0.0f;
        robot.yaw_deg = 0.0f;
        return;
    }

    robot.roll_deg = imu->roll;
    robot.pitch_deg = imu->pitch;
    robot.yaw_deg = imu->yaw;
}

Point3D_t Kinematics_FK(JointAngle_t joint, float l4)
{
    Point3D_t p;
    float t1 = Deg2Rad(joint.theta1);
    float t2 = Deg2Rad(joint.theta2);
    float d3 = joint.d3;

    /* 单臂局部 FK，输出爪端在该侧局部基座坐标系 {0_s} 下的位置。 */
    p.x = cosf(t1) * (l4 * cosf(t2) - d3 * sinf(t2) + L2) - L3 * sinf(t1);
    p.y = sinf(t1) * (l4 * cosf(t2) - d3 * sinf(t2) + L2) + L3 * cosf(t1);
    p.z = -l4 * sinf(t2) - d3 * cosf(t2) + L1;

    return p;
}

/**
 * @brief 按左右侧封装 FK，自动选择该侧 L4。
 * @param joint 单臂关节量。
 * @param side 左右臂侧别。
 * @return 爪端局部坐标。
 */
static Point3D_t Climb_FK(JointAngle_t joint, Climb_Side_t side)
{
    return Kinematics_FK(joint, Climb_GetL4(side));
}

/**
 * @brief 计算当前构型的可操作性粗判据。
 * @param joint 单臂关节量。
 * @param side 左右臂侧别。
 * @return |det(J)| 的简化结果，越小越接近奇异位形。
 */
static float Climb_Manipulability(JointAngle_t joint, Climb_Side_t side)
{
    float t2 = Deg2Rad(joint.theta2);
    float d3 = joint.d3;
    float l4 = Climb_GetL4(side);
    float u = l4 * cosf(t2) - d3 * sinf(t2) + L2;

    /* det(J) = -d3 * U，这里只取绝对值作为远离奇异位形的粗判据。 */
    return fabsf(d3 * u);
}

/**
 * @brief 使用闭环模块公开的限位常量做 IK 结果预检查。
 * @param joint 待检查的关节量。
 * @param side 左右臂侧别。
 * @return 1=大致在闭环允许范围内，0=超出范围。
 * @note 这里不替代闭环模块的最终限位，只用于状态机判断。
 */
static uint8_t Climb_JointInsideApproxLimit(JointAngle_t joint, Climb_Side_t side)
{
    float yaw_min = (side == CLIMB_SIDE_RIGHT) ?
        PWM_ANGLE_RIGHT_YAW_MIN_DEG :
        PWM_ANGLE_LEFT_YAW_MIN_DEG;
    float yaw_max = (side == CLIMB_SIDE_RIGHT) ?
        PWM_ANGLE_RIGHT_YAW_MAX_DEG :
        PWM_ANGLE_LEFT_YAW_MAX_DEG;

    if (joint.d3 < M3508_POS_DEFAULT_D3_MIN_MM ||
        joint.d3 > M3508_POS_DEFAULT_D3_MAX_MM) {
        return 0u;
    }

    if (joint.theta1 < yaw_min || joint.theta1 > yaw_max) {
        return 0u;
    }

    if (joint.theta2 < PWM_ANGLE_DEFAULT_PITCH_MIN_DEG ||
        joint.theta2 > PWM_ANGLE_DEFAULT_PITCH_MAX_DEG) {
        return 0u;
    }

    return 1u;
}

/**
 * @brief 单臂局部逆运动学求解。
 * @param p0 爪端在单臂局部基座坐标系 {0_s} 下的目标点。
 * @param side 左右臂侧别，用于选择 L4 与左右限位。
 * @param out_joint 输出关节目标。
 * @return IK 状态；LIMITED 表示数学可解但超出当前闭环限位。
 */
Climb_IKStatus_t Climb_Kinematics_IK(Point3D_t p0,
                                      Climb_Side_t side,
                                      JointAngle_t *out_joint)
{
    float l4;
    float rho2;
    float u2;
    float u;
    float m;
    float n;
    float d32;
    float d3;
    float denom;
    float sin_t2;
    float cos_t2;
    JointAngle_t joint;
    Climb_IKStatus_t status = CLIMB_IK_OK;

    if (out_joint == 0) {
        return CLIMB_IK_UNREACHABLE;
    }

    l4 = Climb_GetL4(side);
    rho2 = p0.x * p0.x + p0.y * p0.y;
    u2 = rho2 - L3 * L3;
    if (u2 < 0.0f) {
        return CLIMB_IK_UNREACHABLE;
    }

    u = sqrtf(u2);

    /*
     * 采用正运动学模型中的正根分支。
     * 若实机需要另一构型分支，应在这里扩展分支选择策略。
     */
    joint.theta1 = Rad2Deg(atan2f(p0.y, p0.x) - atan2f(L3, u));

    m = u - L2;
    n = p0.z - L1;
    d32 = m * m + n * n - l4 * l4;
    if (d32 < 0.0f) {
        return CLIMB_IK_UNREACHABLE;
    }

    d3 = sqrtf(d32);
    denom = l4 * l4 + d3 * d3;
    if (denom < CLIMB_EPS) {
        return CLIMB_IK_UNREACHABLE;
    }

    sin_t2 = (-d3 * m - l4 * n) / denom;
    cos_t2 = (l4 * m - d3 * n) / denom;

    /* 抑制浮点舍入导致的 1.000000x 输入 atan2 前异常传播。 */
    sin_t2 = Clamp_Float(sin_t2, -1.0f, 1.0f);
    cos_t2 = Clamp_Float(cos_t2, -1.0f, 1.0f);

    joint.theta2 = Rad2Deg(atan2f(sin_t2, cos_t2));
    joint.d3 = d3;

    if (!Climb_JointInsideApproxLimit(joint, side)) {
        status = CLIMB_IK_LIMITED;
    }

    if (Climb_Manipulability(joint, side) < CLIMB_SINGULAR_EPS) {
        status = CLIMB_IK_SINGULAR;
    }

    *out_joint = joint;
    return status;
}

/**
 * @brief 读取某侧当前关节量。
 * @param side 左右臂侧别。
 * @return 当前 yaw、pitch、d3。
 * @note 角度来自 PWM 角度闭环，伸缩长度来自 M3508 位置闭环。
 */
static JointAngle_t Climb_ReadJoint(Climb_Side_t side)
{
    JointAngle_t q;

    /*
     * 当前关节量统一从两个闭环模块读取。
     * climb_control 不再直接读原始编码器来推断电机目标。
     */
    if (side == CLIMB_SIDE_RIGHT) {
        q.theta1 = PWM_AngleServo_GetCurrentAngle(PWM_ANGLE_RIGHT_YAW);
        q.theta2 = PWM_AngleServo_GetCurrentAngle(PWM_ANGLE_RIGHT_PITCH);
        q.d3 = M3508_Position_GetCurrentLength(M3508_POS_RIGHT);
    } else {
        q.theta1 = PWM_AngleServo_GetCurrentAngle(PWM_ANGLE_LEFT_YAW);
        q.theta2 = PWM_AngleServo_GetCurrentAngle(PWM_ANGLE_LEFT_PITCH);
        q.d3 = M3508_Position_GetCurrentLength(M3508_POS_LEFT);
    }

    return q;
}

/**
 * @brief 根据当前机身位姿与当前关节量计算爪端世界坐标。
 * @param side 左右臂侧别。
 * @return 爪端在世界坐标系 {W} 下的位置。
 */
static Point3D_t Climb_CalcClawWorld(Climb_Side_t side)
{
    Climb_Matrix3_t r = Climb_GetRotationWB();
    Climb_ArmState_t *arm = Climb_GetArm(side);
    Point3D_t base_b = Climb_GetBaseOffsetB(side);
    Point3D_t claw_0 = Climb_FK(arm->current_joint, side);
    Point3D_t claw_b = Point_Add(base_b, claw_0);

    /* P_claw_W = P_body_W + R_WB * (P_base_B + P_claw_0) */
    return Point_Add(robot.body_pos_w, Climb_MulMatVec(r, claw_b));
}

/**
 * @brief 将世界坐标目标转换为某侧单臂局部基座坐标。
 * @param side 左右臂侧别。
 * @param world_target 世界坐标系 {W} 下的目标点。
 * @return 局部基座坐标系 {0_s} 下的目标点。
 */
static Point3D_t Climb_WorldToLocal(Climb_Side_t side, Point3D_t world_target)
{
    Climb_Matrix3_t r = Climb_GetRotationWB();
    Point3D_t base_b = Climb_GetBaseOffsetB(side);
    Point3D_t delta_w = Point_Sub(world_target, robot.body_pos_w);
    Point3D_t delta_b = Climb_MulMatTVec(r, delta_w);

    /* 把世界目标点反变换到单臂局部基座坐标系，供 IK 使用。 */
    return Point_Sub(delta_b, base_b);
}

/**
 * @brief 用已锁定锚点反推机身世界坐标。
 * @param side 用作约束源的附着侧。
 * @return 由该侧独立估计的机身世界坐标。
 */
static Point3D_t Climb_EstimateBodyFromAnchor(Climb_Side_t side)
{
    Climb_Matrix3_t r = Climb_GetRotationWB();
    Climb_ArmState_t *arm = Climb_GetArm(side);
    Point3D_t base_b = Climb_GetBaseOffsetB(side);
    Point3D_t claw_0 = Climb_FK(arm->current_joint, side);
    Point3D_t offset_w = Climb_MulMatVec(r, Point_Add(base_b, claw_0));
    Point3D_t body = Point_Sub(arm->anchor_w, offset_w);

    /* 车轮贴墙约束：机身世界 Z 固定为车轮半径。 */
    body.z = WHEEL_RADIUS;
    return body;
}

/**
 * @brief 同步旧版锚点字段。
 * @note 兼容外部旧代码可能访问 robot.anchor_left_w/right_w 的情况。
 */
static void Climb_SyncAnchorAliases(void)
{
    robot.anchor_left_w = robot.left.anchor_w;
    robot.anchor_right_w = robot.right.anchor_w;
}

/**
 * @brief 刷新左右两侧当前关节量缓存。
 */
static void Climb_UpdateCurrentJoints(void)
{
    robot.left.current_joint = Climb_ReadJoint(CLIMB_SIDE_LEFT);
    robot.right.current_joint = Climb_ReadJoint(CLIMB_SIDE_RIGHT);
}

/**
 * @brief 锁存某侧爪端为世界坐标锚点。
 * @param side 需要锁存的侧别。
 * @note 通常在确认抓墙瞬间调用。
 */
static void Climb_LockAnchor(Climb_Side_t side)
{
    Climb_ArmState_t *arm = Climb_GetArm(side);

    /* 抓墙瞬间锁存世界坐标锚点，后续通过该固定点反推机身位姿。 */
    arm->current_joint = Climb_ReadJoint(side);
    arm->anchor_w = Climb_CalcClawWorld(side);
    arm->attached = 1u;
    Climb_SyncAnchorAliases();
}

/**
 * @brief 更新运动学里程计。
 * @note 根据 attached 标志决定使用双锚点融合、左锚点或右锚点反推机身位置。
 */
void Update_Odometry(void)
{
    Point3D_t body_left;
    Point3D_t body_right;

    Climb_UpdateIMUAttitude();
    Climb_UpdateCurrentJoints();

    /*
     * 双臂附着时左右锚点各算一次机身位置，再做等权融合。
     * 单臂附着时使用唯一承重侧作为里程计基准。
     */
    if (robot.left.attached && robot.right.attached) {
        body_left = Climb_EstimateBodyFromAnchor(CLIMB_SIDE_LEFT);
        body_right = Climb_EstimateBodyFromAnchor(CLIMB_SIDE_RIGHT);

        robot.body_pos_w.x = 0.5f * (body_left.x + body_right.x);
        robot.body_pos_w.y = 0.5f * (body_left.y + body_right.y);
        robot.body_pos_w.z = WHEEL_RADIUS;
    } else if (robot.left.attached) {
        robot.body_pos_w = Climb_EstimateBodyFromAnchor(CLIMB_SIDE_LEFT);
    } else if (robot.right.attached) {
        robot.body_pos_w = Climb_EstimateBodyFromAnchor(CLIMB_SIDE_RIGHT);
    } else {
        robot.body_pos_w.z = WHEEL_RADIUS;
    }

    robot.current_dist = robot.body_pos_w.x - robot.start_body_x;
}

/**
 * @brief 统一下发左右臂关节目标。
 * @param left 左臂目标关节量。
 * @param right 右臂目标关节量。
 * @note 这是 climb_control 中唯一真正写入闭环目标的底层封装。
 */
static void Climb_SetJointTarget(JointAngle_t left, JointAngle_t right)
{
    robot.left.target_joint = left;
    robot.right.target_joint = right;

    /*
     * 目标只下发到闭环模块。
     * 角度闭环负责 PWM 方向/速度，M3508 位置闭环负责伸缩长度到位。
     */
    PWM_AngleServo_SetTargetAll(left.theta2,
                                left.theta1,
                                right.theta2,
                                right.theta1);

    M3508_Position_SetTargetLengthBoth(left.d3, right.d3);
}

/**
 * @brief 兼容旧接口的关节目标下发函数。
 * @param left 左臂目标关节量。
 * @param right 右臂目标关节量。
 * @note 内部已改为调用闭环目标接口，不再直接控制电机。
 */
void Execute_Joint_Commands(JointAngle_t left, JointAngle_t right)
{
    Climb_SetJointTarget(left, right);
}

/**
 * @brief 判断左右 PWM 角度闭环和 M3508 长度闭环是否全部到位。
 * @return 1=全部到位，0=至少一个关节未到位。
 */
static uint8_t Climb_AllJointReached(void)
{
    return (PWM_AngleServo_IsAllTargetReached() &&
            M3508_Position_IsAllTargetReached()) ? 1u : 0u;
}

/**
 * @brief 在给定机身位姿下，保持某侧锚点不动并反算该侧关节量。
 * @param side 左右臂侧别。
 * @param anchor_w 该侧固定锚点的世界坐标。
 * @param out_joint 输出求解得到的关节目标。
 * @return 1=可下发目标，0=不可达或奇异。
 */
static uint8_t Climb_SolveJointForAnchor(Climb_Side_t side,
                                         Point3D_t anchor_w,
                                         JointAngle_t *out_joint)
{
    Point3D_t target_0 = Climb_WorldToLocal(side, anchor_w);
    Climb_IKStatus_t status = Climb_Kinematics_IK(target_0, side, out_joint);
    Climb_ArmState_t *arm = Climb_GetArm(side);

    arm->ik_status = status;
    return (status == CLIMB_IK_OK || status == CLIMB_IK_LIMITED) ? 1u : 0u;
}

/**
 * @brief 将两个闭环目标同步到当前位置，实现“就地锁住”。
 * @note 释放/切换状态前使用，避免旧目标残留。
 */
static void Climb_HoldCurrentTargets(void)
{
    JointAngle_t left = Climb_ReadJoint(CLIMB_SIDE_LEFT);
    JointAngle_t right = Climb_ReadJoint(CLIMB_SIDE_RIGHT);

    M3508_Position_SyncTargetToCurrentBoth();
    PWM_AngleServo_LockCurrentPosition();
    Climb_SetJointTarget(left, right);
}

/**
 * @brief 确保 PWM 角度闭环与 M3508 位置闭环已使能。
 */
static void Climb_EnableClosedLoops(void)
{
    if (!M3508_Position_IsEnabled()) {
        M3508_Position_Enable(1u);
    }

    if (!PWM_AngleServo_IsEnabled()) {
        PWM_AngleServo_Enable(1u);
    }
}

/**
 * @brief 给定目标机身位姿，反算所有已附着臂的关节目标。
 * @param target_body 目标机身世界坐标。
 * @note 该函数会临时写 robot.body_pos_w 用于坐标反算；IK 失败会恢复旧值。
 */
static void Climb_CommandBodyPose(Point3D_t target_body)
{
    Point3D_t old_body = robot.body_pos_w;
    JointAngle_t left = robot.left.current_joint;
    JointAngle_t right = robot.right.current_joint;
    uint8_t left_ok = 1u;
    uint8_t right_ok = 1u;

    /*
     * 给定一个期望机身位姿，保持已附着锚点不动，
     * 反求各附着臂应到达的关节目标。
     */
    robot.body_pos_w = target_body;
    robot.body_pos_w.z = WHEEL_RADIUS;

    if (robot.left.attached) {
        left_ok = Climb_SolveJointForAnchor(CLIMB_SIDE_LEFT,
                                            robot.left.anchor_w,
                                            &left);
    }

    if (robot.right.attached) {
        right_ok = Climb_SolveJointForAnchor(CLIMB_SIDE_RIGHT,
                                             robot.right.anchor_w,
                                             &right);
    }

    if (left_ok && right_ok) {
        Climb_SetJointTarget(left, right);
    } else {
        /* IK 失败时恢复估计位姿，避免状态量被不可达目标污染。 */
        robot.body_pos_w = old_body;
    }
}

/**
 * @brief 生成自由摆动臂的下一落点。
 * @param side 当前自由摆动侧。
 * @return 爪端世界坐标目标。
 * @note 当前为最小占位规划：沿 X 前移 step_length，Y 保持对应侧横向位置。
 */
static Point3D_t Climb_MakeSwingTarget(Climb_Side_t side)
{
    Point3D_t target;

    target.x = robot.body_pos_w.x + robot.step_length;
    target.y = robot.body_pos_w.y +
        ((side == CLIMB_SIDE_LEFT) ? (BODY_L_W * 0.5f) : (-BODY_L_W * 0.5f));
    target.z = 0.0f;

    /* 当前只给出最小可运行落点：沿 X 前移一步，Z 落在墙面。 */
    return target;
}

/**
 * @brief 将自由摆动臂移动到下一落点。
 * @param side 当前自由摆动侧。
 * @note 另一侧目标保持最近一次承重目标不变。
 */
static void Climb_MoveFreeArmToTarget(Climb_Side_t side)
{
    Point3D_t target_w = Climb_MakeSwingTarget(side);
    Point3D_t target_0 = Climb_WorldToLocal(side, target_w);
    JointAngle_t target_joint;
    Climb_IKStatus_t status;

    status = Climb_Kinematics_IK(target_0, side, &target_joint);
    Climb_GetArm(side)->ik_status = status;

    if (status == CLIMB_IK_OK || status == CLIMB_IK_LIMITED) {
        if (side == CLIMB_SIDE_LEFT) {
            Climb_SetJointTarget(target_joint, robot.right.target_joint);
        } else {
            Climb_SetJointTarget(robot.left.target_joint, target_joint);
        }
    }
}

/**
 * @brief 重心预偏置子程序。
 * @param to_right 1=向右臂偏置，0=向左臂偏置。
 * @note 当前使用固定横向偏置，后续可替换为力反馈或电流反馈策略。
 */
void Subroutine_CoG_PreBias(uint8_t to_right)
{
    Point3D_t target_body = robot.body_pos_w;
    float mid_y = 0.5f * (robot.left.anchor_w.y + robot.right.anchor_w.y);

    /* 预偏置先用固定横向偏移占位，后续可替换为力/电流反馈策略。 */
    target_body.y = mid_y + (to_right ? -CLIMB_BIAS_Y_MM : CLIMB_BIAS_Y_MM);
    Climb_CommandBodyPose(target_body);
}

/**
 * @brief 初始化 climb_control 全局状态。
 * @note 不会自动启动攀爬；启动请调用 Climb_Start()。
 */
void Climb_Control_Init(void)
{
    robot.state = CLIMB_IDLE;

    robot.body_pos_w.x = 0.0f;
    robot.body_pos_w.y = 0.0f;
    robot.body_pos_w.z = WHEEL_RADIUS;

    robot.roll_deg = 0.0f;
    robot.pitch_deg = 0.0f;
    robot.yaw_deg = 0.0f;

    robot.left.attached = 0u;
    robot.right.attached = 0u;
    robot.left.ik_status = CLIMB_IK_OK;
    robot.right.ik_status = CLIMB_IK_OK;

    robot.target_dist = 0.0f;
    robot.current_dist = 0.0f;
    robot.step_length = CLIMB_DEFAULT_STEP_MM;
    robot.start_body_x = 0.0f;

    Climb_UpdateIMUAttitude();
    Climb_UpdateCurrentJoints();
    robot.left.target_joint = robot.left.current_joint;
    robot.right.target_joint = robot.right.current_joint;

    Climb_SyncAnchorAliases();
}

/**
 * @brief INIT 状态处理：同步当前位置、使能闭环、锁存初始双锚点。
 */
static void Climb_InitPositionState(void)
{
    Climb_UpdateIMUAttitude();
    Climb_UpdateCurrentJoints();

    robot.body_pos_w.z = WHEEL_RADIUS;
    robot.start_body_x = robot.body_pos_w.x;
    robot.current_dist = 0.0f;

    Climb_HoldCurrentTargets();
    Climb_EnableClosedLoops();

    Climb_LockAnchor(CLIMB_SIDE_LEFT);
    Climb_LockAnchor(CLIMB_SIDE_RIGHT);

    robot.state = CLIMB_PRE_BIAS_TO_RIGHT;
}

/**
 * @brief 攀爬控制主状态机，建议 5ms 调用一次。
 * @note 本函数只做运动学规划和目标下发，底层闭环更新仍由各闭环模块负责。
 */
void Climb_Control_Loop_5ms(void)
{
    if (robot.state != CLIMB_IDLE &&
        robot.state != CLIMB_INIT_POSITION &&
        robot.state != CLIMB_EMERGENCY_STOP) {
        Update_Odometry();
    } else {
        Climb_UpdateIMUAttitude();
        Climb_UpdateCurrentJoints();
    }

    switch (robot.state) {
    case CLIMB_IDLE:
        break;

    case CLIMB_INIT_POSITION:
        Climb_InitPositionState();
        break;

    case CLIMB_PRE_BIAS_TO_RIGHT:
        Subroutine_CoG_PreBias(1u);
        if (Climb_AllJointReached()) {
            robot.state = CLIMB_RELEASE_LEFT;
        }
        break;

    case CLIMB_RELEASE_LEFT:
        robot.left.attached = 0u;
        Climb_HoldCurrentTargets();
        robot.state = CLIMB_MOVE_LEFT_ARM;
        break;

    case CLIMB_MOVE_LEFT_ARM:
        Climb_MoveFreeArmToTarget(CLIMB_SIDE_LEFT);
        if (Climb_AllJointReached()) {
            robot.state = CLIMB_GRAB_LEFT;
        }
        break;

    case CLIMB_GRAB_LEFT:
        Climb_LockAnchor(CLIMB_SIDE_LEFT);
        robot.state = CLIMB_PULL_UP;
        break;

    case CLIMB_PULL_UP: {
        Point3D_t target_body = robot.body_pos_w;
        float remain = robot.target_dist - robot.current_dist;
        float step = CLIMB_PULL_RATE_MM_TICK;

        if (remain <= 0.0f) {
            if (Climb_AllJointReached()) {
                robot.state = CLIMB_PRE_BIAS_TO_LEFT;
            }
            break;
        }

        if (step > remain) {
            step = remain;
        }

        target_body.x += step;
        Climb_CommandBodyPose(target_body);
        break;
    }

    case CLIMB_PRE_BIAS_TO_LEFT:
        Subroutine_CoG_PreBias(0u);
        if (Climb_AllJointReached()) {
            robot.state = CLIMB_RELEASE_RIGHT;
        }
        break;

    case CLIMB_RELEASE_RIGHT:
        robot.right.attached = 0u;
        Climb_HoldCurrentTargets();
        robot.state = CLIMB_MOVE_RIGHT_ARM;
        break;

    case CLIMB_MOVE_RIGHT_ARM:
        Climb_MoveFreeArmToTarget(CLIMB_SIDE_RIGHT);
        if (Climb_AllJointReached()) {
            robot.state = CLIMB_GRAB_RIGHT;
        }
        break;

    case CLIMB_GRAB_RIGHT:
        Climb_LockAnchor(CLIMB_SIDE_RIGHT);
        if (robot.current_dist >= robot.target_dist) {
            robot.state = CLIMB_IDLE;
        } else {
            robot.state = CLIMB_PRE_BIAS_TO_RIGHT;
        }
        break;

    case CLIMB_EMERGENCY_STOP:
        M3508_Position_Enable(0u);
        PWM_AngleServo_Enable(0u);
        M3508_Position_StopAll();
        PWM_AngleServo_StopAll();
        break;

    default:
        robot.state = CLIMB_EMERGENCY_STOP;
        break;
    }
}

/**
 * @brief 启动攀爬任务。
 * @param distance 目标攀爬距离，单位 mm；小于 0 时按 0 处理。
 */
void Climb_Start(float distance)
{
    robot.target_dist = (distance > 0.0f) ? distance : 0.0f;
    robot.current_dist = 0.0f;
    robot.state = CLIMB_INIT_POSITION;
}

/**
 * @brief 停止攀爬状态机并停止闭环输出。
 */
void Climb_Stop(void)
{
    robot.state = CLIMB_IDLE;
    M3508_Position_StopAll();
    PWM_AngleServo_StopAll();
}

/**
 * @brief 获取当前状态机状态。
 * @return 当前 Climb_State_t。
 */
Climb_State_t Climb_GetState(void)
{
    return robot.state;
}
