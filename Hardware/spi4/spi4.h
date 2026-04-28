#ifndef __SPI4_H
#define __SPI4_H

#include "main.h" // 包含HAL库和基础引脚定义

// 定义编码器数据结构体
typedef struct {
    uint16_t raw_angle;     // 16位原始角度 (0~65535)
    float degree;           // 转换后的物理角度 (0~360度)
    uint8_t status;         // 状态字
    uint8_t crc;            // CRC校验码
} PQY13_Data_t;

// 声明外部变量，这样 main.c 或其他文件就可以直接读取 encoder_data
extern PQY13_Data_t encoder_data;

// 函数声明
void Read_PQY13_Encoder(void);

#endif /* __SPI4_H */
