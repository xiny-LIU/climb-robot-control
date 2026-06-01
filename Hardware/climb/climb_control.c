#include "climb_control.h"
#include "math.h"
#include "PID.h"
#include "pwm_motor.h"
#include "spi4.h"
#include "usart6.h"
#include "encoder_counter.h"
#include "analysis_data.h"

#define PI 3.14159265f

/* DH 参数 */
static const float L1 = 25.21f;
static const float L2 = 8.65f;
static const float L3 = 5.66f;
static const float L4_LEFT = 6.21f;
static const float L4_RIGHT = 0.59f;

/* 全局机器人状态结构体实例 */
Climb_Robot_t robot;

/* 内部数学辅助函数 */
static float Deg2Rad(float deg) { return deg * PI / 180.0f; }
static float Rad2Deg(float rad) { return rad * 180.0f / PI; }

/**
 * @brief [详细解释] 这是一个限位工具函数，防止算出来的值超出物理极限
 */
static float Clamp_Float(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief 单臂局部正运动学 (FK)
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
 * @brief 更新运动里程计 (视角反转)
 */
void Update_Odometry(void) {
    protocol_info_t* imu = IMU_GetOutputInfo();
    float gamma = Deg2Rad(imu->pitch); // 机身当前实际俯仰角
    
    // 获取当前抓墙手臂的关节数据 (假设左臂抓墙)
    JointAngle_t left_joint;
    left_joint.theta1 = encoder_data[ENC_2].degree; // 左偏航
    left_joint.theta2 = encoder_data[ENC_1].degree; // 左俯仰
    left_joint.d3 = (float)Encoder_Get_Total_Angle(0) / 8192.0f * PI * 38.0f; 

    Point3D_t p_local = Kinematics_FK(left_joint, L4_LEFT);
    
    robot.body_pos_w.x = robot.anchor_left_w.x - (cosf(gamma) * p_local.x + sinf(gamma) * p_local.z);
    robot.body_pos_w.y = robot.anchor_left_w.y - p_local.y;
}

/**
 * @brief [详细解释] 电机指令发送包装。不管上面怎么算，在这里必须经过限位检查才能发给电机。
 */
void Execute_Joint_Commands(JointAngle_t left, JointAngle_t right) {
    
    // 1. 严格的安全软限位 (Clamp拦截)
    left.theta1 = Clamp_Float(left.theta1, YAW_MIN, YAW_MAX);
    left.theta2 = Clamp_Float(left.theta2, PITCH_MIN, PITCH_MAX);
    left.d3     = Clamp_Float(left.d3, D3_MIN_LENGTH, D3_MAX_LENGTH);
    
    right.theta1 = Clamp_Float(right.theta1, YAW_MIN, YAW_MAX);
    right.theta2 = Clamp_Float(right.theta2, PITCH_MIN, PITCH_MAX);
    right.d3     = Clamp_Float(right.d3, D3_MIN_LENGTH, D3_MAX_LENGTH);

    // 2. 发送伸缩电机指令 (这里需要你后续补充位置环逻辑，暂用速度环代指)
    PID_SetTargetSpeed(0, (int16_t)left.d3);  
    PID_SetTargetSpeed(1, (int16_t)right.d3);

    // 3. 发送俯仰/偏航电机(PWM)指令
    float err_l_p = left.theta2 - encoder_data[ENC_1].degree;
    Motor_SetSpeed(MOTOR_A, (uint8_t)Clamp_Float(fabsf(err_l_p * 5.0f), 0, 100)); 
    Motor_SetDirection(MOTOR_A, err_l_p > 0 ? DIRECTION_FORWARD : DIRECTION_REVERSE);

    float err_l_y = left.theta1 - encoder_data[ENC_2].degree;
    Motor_SetSpeed(MOTOR_B, (uint8_t)Clamp_Float(fabsf(err_l_y * 5.0f), 0, 100));
    Motor_SetDirection(MOTOR_B, err_l_y > 0 ? DIRECTION_FORWARD : DIRECTION_REVERSE);
    
    float err_r_p = right.theta2 - encoder_data[ENC_3].degree;
    Motor_SetSpeed(MOTOR_C, (uint8_t)Clamp_Float(fabsf(err_r_p * 5.0f), 0, 100)); 
    Motor_SetDirection(MOTOR_C, err_r_p > 0 ? DIRECTION_FORWARD : DIRECTION_REVERSE);

    float err_r_y = right.theta1 - encoder_data[ENC_4].degree;
    Motor_SetSpeed(MOTOR_D, (uint8_t)Clamp_Float(fabsf(err_r_y * 5.0f), 0, 100));
    Motor_SetDirection(MOTOR_D, err_r_y > 0 ? DIRECTION_FORWARD : DIRECTION_REVERSE);
}

/**
 * @brief 重心预偏置子程序
 */
void Subroutine_CoG_PreBias(uint8_t to_right) {
    if (to_right) {
        JointAngle_t target_r = {0, 0, D3_MIN_LENGTH}; 
        JointAngle_t release_l = {0, 0, 0}; 
        Execute_Joint_Commands(release_l, target_r);
    }
}

/**
 * @brief 初始化
 */
void Climb_Control_Init(void) {
    robot.state = CLIMB_IDLE;
    robot.body_pos_w.x = 0;
    robot.body_pos_w.y = 0;
}

/**
 * @brief [详细解释] 这就是所谓的“主状态机”！
 * 它被外面的定时器每5ms调用一次，里面的 switch(robot.state) 根据当前的状态去执行对应的代码块。
 */
void Climb_Control_Loop_5ms(void) {
    Update_Odometry();

    switch (robot.state) {
        
        case CLIMB_IDLE:
            Motor_Stop_All();
            break;

        case CLIMB_PULL_UP: {
            // [详细解释] 删除了复杂的 Calculate_Target_Gamma 函数！
            // 因为车轮很大，离墙很远，这里直接“传 0”，强行让机身在数学解算中保持垂直！
            float target_gamma = 0.0f; 
            
            // 后续我们会根据这个 target_gamma = 0 去逆向算出 theta1, theta2, d3
            // 然后调用 Execute_Joint_Commands(算出左臂角度, 算出右臂角度);
            break;
        }

        case CLIMB_PRE_BIAS_TO_RIGHT:
            Subroutine_CoG_PreBias(1);
            if (fabsf(robot.body_pos_w.y - robot.anchor_right_w.y) < 5.0f) {
                // 如果重心移到了右边，就把状态改为“释放左臂”，下一次循环就会去执行释放动作
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
 * @brief 启动指令
 */
void Climb_Start(float distance) {
    robot.target_dist = distance;
    robot.state = CLIMB_INIT_POSITION;
}
