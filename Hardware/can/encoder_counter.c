#include "encoder_counter.h"
#include "CAN_receive.h"
#include "string.h"

// 4个电机的计数器（对应M3508的4个ID：0x201~0x204）
static EncoderCounter_t encoder_cnt[4];

/**
 * @brief 初始化编码器计数器
 */
void Encoder_Counter_Init(void)
{
    memset(encoder_cnt, 0, sizeof(encoder_cnt));
    
    // 等待CAN数据就绪（电调已上电并发送数据）
    HAL_Delay(50);
    
    // 用当前位置作为初始零点
    for (uint8_t i = 0; i < 4; i++) {
        const motor_measure_t *motor = get_chassis_motor_measure_point(i);
        encoder_cnt[i].last_ecd = motor->ecd;
        encoder_cnt[i].total_angle = 0;      // 当前位置作为0点
        encoder_cnt[i].turn_count = 0;
        encoder_cnt[i].initialized = 1;
    }
}

/**
 * @brief 更新编码器计数（核心函数，在CAN中断中调用）
 * 
 * 【回绕检测原理】
 * - 正常转动：delta 在 -4096 ~ +4096 之间
 * - 正转回绕：ecd从8191→0，delta ≈ -8192（<-4096）
 * - 反转回绕：ecd从0→8191，delta ≈ +8192（>+4096）
 */
void Encoder_Counter_Update(uint8_t motor_id, uint16_t current_ecd)
{
    if (motor_id > 3) return;
    if (!encoder_cnt[motor_id].initialized) return;
    
    // 计算原始差值（有符号16位，自动处理回绕）
    int16_t delta = (int16_t)current_ecd - (int16_t)encoder_cnt[motor_id].last_ecd;
    
    // ========== 回绕检测与修正 ==========
    if (delta < -4096) {
        // 【正转回绕】ecd从8191跳到0附近
        // 例：last=8190, current=10, delta=-8180
        // 实际变化：+12（正转12个单位）
        encoder_cnt[motor_id].turn_count++;        // 圈数+1
        delta += 8192;                              // 修正：-8180 + 8192 = +12
    }
    else if (delta > 4096) {
        // 【反转回绕】ecd从0跳到8191附近
        // 例：last=10, current=8190, delta=+8180
        // 实际变化：-12（反转12个单位）
        encoder_cnt[motor_id].turn_count--;        // 圈数-1
        delta -= 8192;                              // 修正：+8180 - 8192 = -12
    }
    // 正常情况：delta不变
    
    // 累计总角度
    encoder_cnt[motor_id].total_angle += delta;
    encoder_cnt[motor_id].last_ecd = current_ecd;
}

/**
 * @brief 获取累计角度
 */
int32_t Encoder_Get_Total_Angle(uint8_t motor_id)
{
    if (motor_id > 3) return 0;
    return encoder_cnt[motor_id].total_angle;
}

/**
 * @brief 获取圈数
 */
int32_t Encoder_Get_Turn_Count(uint8_t motor_id)
{
    if (motor_id > 3) return 0;
    return encoder_cnt[motor_id].turn_count;
}

/**
 * @brief 获取当前ecd
 */
uint16_t Encoder_Get_Current_ECD(uint8_t motor_id)
{
    if (motor_id > 3) return 0;
    return encoder_cnt[motor_id].last_ecd;
}

/**
 * @brief 重置计数器（设置当前为零点）
 */
void Encoder_Counter_Reset(uint8_t motor_id)
{
    if (motor_id > 3) return;
    
    const motor_measure_t *motor = get_chassis_motor_measure_point(motor_id);
    encoder_cnt[motor_id].last_ecd = motor->ecd;
    encoder_cnt[motor_id].total_angle = 0;
    encoder_cnt[motor_id].turn_count = 0;
}
