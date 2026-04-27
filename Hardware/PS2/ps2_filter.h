// ps2_filter.h
#ifndef __PS2_FILTER_H
#define __PS2_FILTER_H

#include <stdint.h>

// 快速初始化
void PS2_Filter_Init(void);

// 获取滤波后的摇杆值 - 保证实时性
uint8_t PS2_Filter_Get_LY(uint8_t raw_value);
uint8_t PS2_Filter_Get_RX(uint8_t raw_value);
uint8_t PS2_Filter_Get_LX(uint8_t raw_value);
uint8_t PS2_Filter_Get_RY(uint8_t raw_value);

#endif
