#ifndef __CLIMB_CONTROL_H
#define __CLIMB_CONTROL_H

#include "stdint.h"

/* 物理常量定义 */
#define WHEEL_RADIUS        162.76f    // 车轮半径 R (mm)
#define BODY_L_W            270.0f   // 横向安装距 Lw (mm)
#define SAFE_HEIGHT         60.0f    // 肩关节离墙安全距离 H_safe (mm)
#define PULLEY_RADIUS       19.0f    // 摩擦轮半径 (38mm直径/2)
#define EXT_GEAR_RATIO      36.0f    // 伸缩电机减速比

/* 物理常量更新 */
#define D3_MIN_LENGTH       550.0f   // 伸缩杆最小长度 (mm)
#define D3_MAX_LENGTH       2000.0f   // 【新增】伸缩杆最大长度 (mm) 需根据你的管长设定

/* 关节角度软限位 (度) */
#define YAW_MIN             -6.0f    // 偏航角极小值
#define YAW_MAX             22.0f    // 偏航角极大值
#define PITCH_MIN           -8.0f    // 俯仰角极小值
#define PITCH_MAX           30.0f    // 俯仰角极大值

/* 攀爬状态枚举 */
typedef enum {
    CLIMB_IDLE = 0,             // 待机
    CLIMB_INIT_POSITION,        // 初始位姿确立
    CLIMB_PRE_BIAS_TO_RIGHT,    // 重心移向右臂
    CLIMB_RELEASE_LEFT,         // 释放左臂
    CLIMB_MOVE_LEFT_ARM,        // 左臂前移
    CLIMB_GRAB_LEFT,            // 左臂抓取
    CLIMB_PULL_UP,              // 双臂拉升 (协同运动)
    CLIMB_PRE_BIAS_TO_LEFT,     // 重心移向左臂
    CLIMB_RELEASE_RIGHT,        // 释放右臂
    CLIMB_MOVE_RIGHT_ARM,       // 右臂前移
    CLIMB_GRAB_RIGHT,           // 右臂抓取
    CLIMB_EMERGENCY_STOP        // 紧急停止
} Climb_State_t;

/* 坐标点结构体 */
typedef struct {
    float x;
    float y;
    float z;
} Point3D_t;

/* 关节角度结构体 */
typedef struct {
    float theta1;   // 偏航 (deg)
    float theta2;   // 俯仰 (deg)
    float d3;       // 伸长 (mm)
} JointAngle_t;

/* 机器人全局状态结构体 */
typedef struct {
    Climb_State_t state;
    Point3D_t body_pos_w;       // 机身在世界坐标系下的坐标 (Xb, Yb, R)
    Point3D_t anchor_left_w;    // 左锚点世界坐标
    Point3D_t anchor_right_w;   // 右锚点世界坐标
    float target_dist;          // 目标攀爬距离
    float current_dist;         // 已完成距离
} Climb_Robot_t;

/* 对外接口 */
void Climb_Control_Init(void);
void Climb_Control_Loop_5ms(void);
void Climb_Start(float distance);

#endif
