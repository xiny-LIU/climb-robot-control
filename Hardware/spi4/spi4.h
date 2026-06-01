#ifndef __SPI4_H
#define __SPI4_H

#include "main.h"  // 必须包含 main.h，因为 CS_ENCx 的宏定义都在里面

// 定义编码器编号枚举
typedef enum {
    ENC_1 = 0,
    ENC_2,
    ENC_3,
    ENC_4
} Encoder_ID_t;

// 定义编码器数据结构体
typedef struct {
    uint16_t raw_angle;     // 16位原始角度 (0~65535)
    float degree;           // 转换后的物理角度 (0~360度)
    uint8_t status;         // 状态字
    uint8_t crc;            // CRC校验码
} PQY13_Data_t;

// 声明一个包含4个元素的数组，存放四个编码器的数据
extern PQY13_Data_t encoder_data[4];

// 函数声明
void Read_PQY13_Encoder(Encoder_ID_t id);
void Update_All_Encoders(void);

#endif /* __SPI4_H */
