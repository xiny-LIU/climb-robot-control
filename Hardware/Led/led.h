#ifndef __LED_H
#define __LED_H

#include "main.h" 
#include "stdio.h"  
extern uint8_t key_press_flag;
void Led_Task(void);
// ********** 仅声明需要用到的引脚宏（与CubeMX配置一致！）**********
//#define LED_RED_PIN        GPIO_PIN_11    // 必须和CubeMX中LED配置的引脚一致
//#define LED_RED_GPIO_PORT  GPIOE         // 必须和CubeMX配置一致
//#define KEY_PIN            GPIO_PIN_2    // 必须和CubeMX中按键配置的引脚一致
//#define KEY_GPIO_PORT      GPIOB         // 必须和CubeMX配置一致
//#define LED1_PIN           GPIO_PIN_1    // 必须和CubeMX中按键配置的引脚一致
//#define LED1_GPIO_PORT      GPIOG         // 必须和CubeMX配置一致
#endif
