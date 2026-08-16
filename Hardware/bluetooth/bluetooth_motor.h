#ifndef _BLUETOOTH_MOTOR_H
#define _BLUETOOTH_MOTOR_H

#include "stm32f4xx_hal.h"
#include "stdio.h"

/* 配置选项 */
#define BT_OUTPUT_PERIOD_MS     100     /* 数据输出周期(ms)，默认100ms */
#define BT_OUTPUT_FORMAT_JSON   0       /* 设置为1输出JSON格式，0输出CSV格式 */

/* 初始化函数 */
void BT_Init(void);

/* 数据输出函数 - 需要在主循环中周期性调用 */
void BT_Process(void);

/* 立即发送一次电机数据 */
void BT_SendData(void);

/* 设置输出使能/禁用 */
void BT_SetOutputEnable(uint8_t enable);

#endif /* _BLUETOOTH_MOTOR_H */
