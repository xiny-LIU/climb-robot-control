#include "climb_control.h"
#include "math.h"
#include "PID.h"
#include "pwm_motor.h"
#include "spi4.h"
#include "usart6.h"
#include "encoder_counter.h"
#include "analysis_data.h"

#define PI 3.14159265f

/* DH 参数 (来自讨论记录0506) */
static const float L1 = 25.21f;
static const float L2 = 8.65f;
static const float L3 = 5.66f;
static const float L4_LEFT = 6.21f;
static const float L4_RIGHT = 0.59f;

/* 安装偏置 (肩关节相对于机身中心) */
static const float SHOULDER_DX = 50.0f;  // 假设值，需实测
static const float SHOULDER_DZ = 20.0f;  // 假设值，需实测

Climb_Robot_t robot;

/* 内部数学辅助函数 */
static float Deg2Rad(float deg) { return deg * PI / 180.0f; }
static float Rad2Deg(float rad) { return rad * 180.0f / PI; }

/**
 * @brief 单臂局部正运动学 (FK)
 * @return 锚点相对于手臂基座坐标系 {0} 的位置
 */
Point3D_t Kinematics_FK(JointAngle_t joint, float l4) {
    Point3D_t p;
    float t1 = Deg2Rad(joint.theta1);
    float t2 = Deg2Rad(joint.theta2);
    float d3 = joint.d3;

    p.x = cosf(t1) * (l4 * cosf(t2) - d3 * sinf(t2) + L2) - L3 * sinf(t1);
    p.y = sinf(t1) * (l4 * cosf(t2) - d3 * sinf(t2) + L2) + L3 * cosf(t1);
    p.z = -l4 * sinf(t2) - d3 * cosf(t2) + L1;
    return p;
}

/**
 * @brief 计算理论俯仰角 γ(t) (策略2: 安全间隙法)
 */
float Calculate_Target_Gamma(void) {
    float C = SAFE_HEIGHT - WHEEL_RADIUS;
    float A = SHOULDER_DZ;
    float B = SHOULDER_DX;
    // γ(t) = arcsin((H_safe - R)/sqrt(dx^2+dz^2)) - arctan(dz/-dx)
    float gamma = asinf(C / sqrtf(B * B + A * A)) - atan2f(A, -B);
    return Rad2Deg(gamma);
}

/**
 * @brief 更新运动里程计 (视角反转)
 */
void Update_Odometry(void) {
    protocol_info_t* imu = IMU_GetOutputInfo();
    float gamma = Deg2Rad(imu->pitch); // 机身当前实际俯仰角
    
    // 获取当前抓墙手臂的关节数据 (假设左臂抓墙)
    JointAngle_t left_joint;
    left_joint.theta1 = encoder_data[ENC_2].degree; // 左偏航
    left_joint.theta2 = encoder_data[ENC_1].degree; // 左俯仰
    left_joint.d3 = (float)Encoder_Get_Total_Angle(0) / 8192.0f * PI * 38.0f; // 假设d0=0

    Point3D_t p_local = Kinematics_FK(left_joint, L4_LEFT);
    
    // 反算机身世界坐标 (简化版)
    // XB = Anchor_X - (cos(gamma)*px + sin(gamma)*pz)
    robot.body_pos_w.x = robot.anchor_left_w.x - (cosf(gamma) * p_local.x + sinf(gamma) * p_local.z);
    robot.body_pos_w.y = robot.anchor_left_w.y - p_local.y;
}

/**
 * @brief 电机指令发送包装
 */
void Execute_Joint_Commands(JointAngle_t left, JointAngle_t right) {
    // 1. 伸缩电机 (PID速度环或位置环，此处简化为设置目标)
    // 实际应根据当前位置与目标的差值计算速度
    PID_SetTargetSpeed(0, (int16_t)left.d3);  // 逻辑需根据PID.h调整
    PID_SetTargetSpeed(1, (int16_t)right.d3);

    // 2. 俯仰/偏航电机 (PWM控制)
    // 这里需要一个简单的关节角度闭环：Angle_Error -> PWM_Speed
    float err_l_p = left.theta2 - encoder_data[ENC_1].degree;
    Motor_SetSpeed(MOTOR_A, (uint8_t)fabsf(err_l_p * 5.0f)); 
    Motor_SetDirection(MOTOR_A, err_l_p > 0 ? DIRECTION_FORWARD : DIRECTION_REVERSE);

    float err_l_y = left.theta1 - encoder_data[ENC_2].degree;
    Motor_SetSpeed(MOTOR_B, (uint8_t)fabsf(err_l_y * 5.0f));
    Motor_SetDirection(MOTOR_B, err_l_y > 0 ? DIRECTION_FORWARD : DIRECTION_REVERSE);
    
    // 右臂同理...
}

/**
 * @brief 重心预偏置子程序 (受控弧线下坠)
 */
void Subroutine_CoG_PreBias(uint8_t to_right) {
    if (to_right) {
        // 目标：YB = Y_anchor_right, d3R = D3_MIN
        // 逆解过程...
        JointAngle_t target_r = {0, 0, D3_MIN_LENGTH}; 
        JointAngle_t release_l = {0, 0, 0}; // 左臂放松
        Execute_Joint_Commands(release_l, target_r);
    }
}

/**
 * @brief 攀爬控制初始化
 */
void Climb_Control_Init(void) {
    robot.state = CLIMB_IDLE;
    robot.body_pos_w.x = 0;
    robot.body_pos_w.y = 0;
}

/**
 * @brief 主控制循环 (由 user_tim.c 的 5ms 任务调用)
 */
void Climb_Control_Loop_5ms(void) {
    Update_Odometry();

    switch (robot.state) {
        case CLIMB_IDLE:
            Motor_Stop_All();
            break;

        case CLIMB_PULL_UP: {
            float target_gamma = Calculate_Target_Gamma();
            // 协同计算各关节目标值并发送指令
            // 此处省略具体的 IK 实时解算过程代码
            break;
        }

        case CLIMB_PRE_BIAS_TO_RIGHT:
            Subroutine_CoG_PreBias(1);
            if (fabsf(robot.body_pos_w.y - robot.anchor_right_w.y) < 5.0f) {
                robot.state = CLIMB_RELEASE_LEFT;
            }
            break;

        case CLIMB_EMERGENCY_STOP:
            Motor_Emergency_Stop();
            break;

        default:
            break;
    }
}

/**
 * @brief 启动攀爬指令
 */
void Climb_Start(float distance) {
    robot.target_dist = distance;
    robot.state = CLIMB_INIT_POSITION;
}
