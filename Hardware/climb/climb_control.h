#ifndef __CLIMB_CONTROL_H
#define __CLIMB_CONTROL_H

#include "stdint.h"

/* 物理常量，除特别说明外单位均为 mm。 */
#define WHEEL_RADIUS        162.76f
#define BODY_L_W            270.0f
#define SAFE_HEIGHT         60.0f
#define PULLEY_RADIUS       19.0f
#define EXT_GEAR_RATIO      36.0f

/*
 * 兼容旧代码保留的限位宏。
 * climb_control 不再把这些宏作为最终保护源，真实软限位以闭环模块为准。
 */
#define D3_MIN_LENGTH       550.0f
#define D3_MAX_LENGTH       2000.0f
#define YAW_MIN             -6.0f
#define YAW_MAX             22.0f
#define PITCH_MIN           -8.0f
#define PITCH_MAX           30.0f

typedef enum {
    CLIMB_IDLE = 0,
    CLIMB_INIT_POSITION,
    CLIMB_PRE_BIAS_TO_RIGHT,
    CLIMB_RELEASE_LEFT,
    CLIMB_MOVE_LEFT_ARM,
    CLIMB_GRAB_LEFT,
    CLIMB_PULL_UP,
    CLIMB_PRE_BIAS_TO_LEFT,
    CLIMB_RELEASE_RIGHT,
    CLIMB_MOVE_RIGHT_ARM,
    CLIMB_GRAB_RIGHT,
    CLIMB_EMERGENCY_STOP
} Climb_State_t;

typedef enum {
    CLIMB_SIDE_LEFT = 0,
    CLIMB_SIDE_RIGHT = 1
} Climb_Side_t;

typedef enum {
    CLIMB_IK_OK = 0,
    CLIMB_IK_UNREACHABLE,
    CLIMB_IK_LIMITED,
    CLIMB_IK_SINGULAR
} Climb_IKStatus_t;

typedef struct {
    float x;
    float y;
    float z;
} Point3D_t;

typedef struct {
    float theta1;   /* 偏航角 yaw，单位 deg */
    float theta2;   /* 俯仰角 pitch，单位 deg */
    float d3;       /* 伸缩长度，单位 mm */
} JointAngle_t;

typedef struct {
    Point3D_t anchor_w;             /* 该侧爪子锁定时的世界坐标锚点 */
    uint8_t attached;               /* 1=该侧爪子参与里程计闭链约束 */
    JointAngle_t current_joint;     /* 从闭环/编码器读取的当前关节量 */
    JointAngle_t target_joint;      /* climb_control 最近一次下发的目标 */
    Climb_IKStatus_t ik_status;     /* 最近一次 IK 求解状态，便于调试 */
} Climb_ArmState_t;

typedef struct {
    Climb_State_t state;

    Point3D_t body_pos_w;
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    Climb_ArmState_t left;
    Climb_ArmState_t right;

    /* 兼容旧字段名，实际数据与 left/right.anchor_w 同步。 */
    Point3D_t anchor_left_w;
    Point3D_t anchor_right_w;

    float target_dist;
    float current_dist;
    float step_length;
    float start_body_x;
} Climb_Robot_t;

extern Climb_Robot_t robot;

/**
 * @brief 初始化攀爬控制状态机与内部状态缓存。
 * @note 只初始化 climb_control 自身状态，不初始化底层闭环模块。
 */
void Climb_Control_Init(void);

/**
 * @brief 攀爬控制 5ms 周期任务入口。
 * @note 建议在 IMU 和编码器数据刷新后调用；本函数只生成闭环目标。
 */
void Climb_Control_Loop_5ms(void);

/**
 * @brief 启动一次攀爬任务。
 * @param distance 目标攀爬距离，单位 mm。
 */
void Climb_Start(float distance);

/**
 * @brief 停止攀爬任务，并停止两个位置闭环模块的当前输出。
 */
void Climb_Stop(void);

/**
 * @brief 获取当前攀爬状态机状态。
 * @return 当前 Climb_State_t 状态。
 */
Climb_State_t Climb_GetState(void);

/**
 * @brief 单臂局部正运动学。
 * @param joint 单臂关节量，theta1/yaw、theta2/pitch 单位 deg，d3 单位 mm。
 * @param l4 当前侧末端结构参数，单位 mm。
 * @return 爪端在该侧局部基座坐标系 {0_s} 下的位置。
 */
Point3D_t Kinematics_FK(JointAngle_t joint, float l4);

/**
 * @brief 单臂局部逆运动学。
 * @param p0 爪端在该侧局部基座坐标系 {0_s} 下的目标位置。
 * @param side 左右臂侧别，用于选择 L4 与左右限位。
 * @param out_joint 输出求解得到的关节目标。
 * @return IK 求解状态。
 */
Climb_IKStatus_t Climb_Kinematics_IK(Point3D_t p0,
                                      Climb_Side_t side,
                                      JointAngle_t *out_joint);

#endif
