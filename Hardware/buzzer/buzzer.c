#include "buzzer.h"
#include "tim.h"    // CubeMX生成的定时器头文件

/**
 * @brief  蜂鸣器初始化
 * @note   在main函数中调用，确保CubeMX已初始化TIM12
 */
void Buzzer_Init(void)
{
    // 启动PWM，初始静音
    HAL_TIM_PWM_Start(&htim12, BUZZER_TIM_CHANNEL);
    Buzzer_Off();
}

/**
 * @brief  蜂鸣器开启
 * @param  volume: 音量等级（0~33332，建议使用BUZZER_VOL_xx宏）
 */
void Buzzer_On(uint16_t volume)
{
    // 限制音量范围
    if(volume > BUZZER_ARR) volume = BUZZER_ARR;
    
    // 反相驱动：占空比越小，蜂鸣器越响（三极管低电平导通）
    // 所以实际响度与占空比成反比，这里做转换
    uint16_t pulse = (volume >= BUZZER_ARR) ? 0 : (BUZZER_ARR - volume);
    __HAL_TIM_SET_COMPARE(&htim12, BUZZER_TIM_CHANNEL, pulse);
}

/**
 * @brief  蜂鸣器关闭（静音）
 */
void Buzzer_Off(void)
{
    // 占空比100%时，PWM持续高电平，三极管截止，蜂鸣器不响
    __HAL_TIM_SET_COMPARE(&htim12, BUZZER_TIM_CHANNEL, BUZZER_ARR + 1);
}

/**
 * @brief  设置音量
 * @param  volume: 0~100百分比
 */
void Buzzer_SetVolume(uint16_t volume)
{
    if(volume > 100) volume = 100;
    uint16_t pulse = (uint32_t)BUZZER_ARR * volume / 100;
    Buzzer_On(pulse);
}

/**
 * @brief  设置蜂鸣器频率
 * @param  freq_hz: 目标频率（Hz），范围100~20000
 * @note   修改ARR会改变PWM频率，同时保持当前占空比
 */
void Buzzer_SetFreq(uint16_t freq_hz)
{
    if(freq_hz < 100) freq_hz = 100;
    if(freq_hz > 20000) freq_hz = 20000;
    
    uint16_t new_arr = (uint16_t)(90000000UL / freq_hz) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim12, new_arr);
}

/**
 * @brief  蜂鸣器单次鸣响
 * @param  time_ms: 鸣响时间（毫秒）
 * @param  volume: 音量等级
 */
void Buzzer_Beep(uint16_t time_ms, uint16_t volume)
{
    Buzzer_On(volume);
    HAL_Delay(time_ms);
    Buzzer_Off();
}

/**
 * @brief  开机提示音 - 通电运行提醒
 * @note   音效：短-短-长 "滴-滴-哒~"（类似系统就绪提示）
 */
void Buzzer_StartUp_Sound(void)
{
    // 音调1：短促高音（100ms，75%音量）
    Buzzer_SetFreq(3000);           // 稍高于额定频率，更清脆
    Buzzer_On(BUZZER_VOL_75);
    HAL_Delay(100);
    Buzzer_Off();
    HAL_Delay(100);                 // 间隔
    
    // 音调2：短促高音（100ms，75%音量）
    Buzzer_On(BUZZER_VOL_75);
    HAL_Delay(100);
    Buzzer_Off();
    HAL_Delay(100);                 // 间隔
    
    // 音调3：长音（400ms，50%音量，回到额定频率）
    Buzzer_SetFreq(2700);
    Buzzer_On(BUZZER_VOL_50);
    HAL_Delay(400);
    Buzzer_Off();
    
    // 恢复默认频率
    Buzzer_SetFreq(2700);
}

/**
 * @brief  错误提示音 - 故障报警
 * @note   音效：连续短促急促音 "滴滴滴滴"
 */
void Buzzer_Error_Sound(void)
{
    for(uint8_t i = 0; i < 5; i++)
    {
        Buzzer_On(BUZZER_VOL_100);
        HAL_Delay(100);
        Buzzer_Off();
        HAL_Delay(100);
    }
}

