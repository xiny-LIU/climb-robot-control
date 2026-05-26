#ifndef __MOTOR_PHYSICS_H
#define __MOTOR_PHYSICS_H

/*
 * M3508 官方减速比：
 * i = 3591 / 187 ≈ 19.203
 *
 * 含义：
 * 电机转子转 i 圈，M3508 输出轴转 1 圈。
 */
#define M3508_REDUCTION_RATIO      (3591.0f / 187.0f)

/*
 * 摩擦轮参数。
 * 摩擦轮与 M3508 输出轴同步转动。
 */
#define FRICTION_WHEEL_RADIUS_MM   19.0f
#define FRICTION_WHEEL_DIAMETER_MM (2.0f * FRICTION_WHEEL_RADIUS_MM)

#endif