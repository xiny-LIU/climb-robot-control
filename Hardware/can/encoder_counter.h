#ifndef _ENCODER_COUNTER_H
#define _ENCODER_COUNTER_H

#include "stdint.h"

/**
 * @brief 编码器计数器结构体
 */
typedef struct {
    int32_t total_angle;    // 累计总角度（可多圈，正=正转，负=反转）
    int32_t turn_count;     // 完整圈数（正=正转圈数，负=反转圈数）
    uint16_t last_ecd;      // 上一次的ecd值
    uint8_t  initialized;   // 初始化标志（0=未初始化，1=已初始化）
} EncoderCounter_t;

/**
 * @brief 初始化编码器计数器
 * @note 在系统启动后、电机转动前调用一次
 */
void Encoder_Counter_Init(void);

/**
 * @brief 更新指定电机的编码器计数（在CAN中断中调用）
 * @param motor_id: 电机编号 0~3
 * @param current_ecd: 当前编码器值（从CAN数据获取）
 */
void Encoder_Counter_Update(uint8_t motor_id, uint16_t current_ecd);

/**
 * @brief 获取电机累计角度
 * @param motor_id: 电机编号 0~3
 * @return 累计角度（8192 = 1圈，可正负）
 */
int32_t Encoder_Get_Total_Angle(uint8_t motor_id);

/**
 * @brief 获取电机完整圈数
 * @param motor_id: 电机编号 0~3
 * @return 圈数（正=正转，负=反转）
 */
int32_t Encoder_Get_Turn_Count(uint8_t motor_id);

/**
 * @brief 获取电机当前单圈角度
 * @param motor_id: 电机编号 0~3
 * @return 0~8191
 */
uint16_t Encoder_Get_Current_ECD(uint8_t motor_id);

/**
 * @brief 重置指定电机的计数器（设置当前位置为零点）
 * @param motor_id: 电机编号 0~3
 */
void Encoder_Counter_Reset(uint8_t motor_id);

#endif
