#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"

/* 蜂鸣器硬件定义 */
#define BUZZER_TIM          TIM12
#define BUZZER_TIM_CHANNEL  TIM_CHANNEL_1
#define BUZZER_ARR          33332   // 2700Hz自动重装载值
#define BUZZER_PSC          0       // 不分频

/* 音量等级（占空比百分比） */
#define BUZZER_VOL_0        0       // 静音
#define BUZZER_VOL_25       (BUZZER_ARR / 4)      // 25%音量
#define BUZZER_VOL_50       (BUZZER_ARR / 2)      // 50%音量
#define BUZZER_VOL_75       (BUZZER_ARR * 3 / 4)  // 75%音量
#define BUZZER_VOL_100      BUZZER_ARR            // 100%音量（实际静音，反相驱动）

/* 音符频率定义（简化版，用于提示音旋律） */
#define NOTE_C5             523
#define NOTE_D5             587
#define NOTE_E5             659
#define NOTE_F5             698
#define NOTE_G5             784
#define NOTE_A5             880
#define NOTE_B5             988

/* 函数声明 */
void Buzzer_Init(void);
void Buzzer_On(uint16_t volume);
void Buzzer_Off(void);
void Buzzer_SetVolume(uint16_t volume);
void Buzzer_SetFreq(uint16_t freq_hz);
void Buzzer_Beep(uint16_t time_ms, uint16_t volume);
void Buzzer_StartUp_Sound(void);    // 开机提示音
void Buzzer_Error_Sound(void);      // 错误提示音

#endif /* __BUZZER_H */
