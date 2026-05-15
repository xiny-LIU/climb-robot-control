#include "spi4.h"
#include "spi.h"
//该文件为编码器通过硬件spi收发转动角度数据（绝对式编码器）
// 实例化含有4个编码器数据的数组
PQY13_Data_t encoder_data[4];

// 读取PQY13编码器函数，传入你要读取的编码器ID
void Read_PQY13_Encoder(Encoder_ID_t id)
{
    // 发送缓冲: 0x05 是指令，后面4个0x00用于产生时钟以接收数据
    uint8_t tx_buffer[5] = {0x05, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buffer[5] = {0};

    // 1. 根据传入的ID，拉低对应的 CS 引脚 (开始通讯)
    // 使用 CubeMX 自动生成的宏定义，完全解耦硬件引脚
    switch (id) {
        case ENC_1: 
            HAL_GPIO_WritePin(CS_ENC1_GPIO_Port, CS_ENC1_Pin, GPIO_PIN_RESET); 
            break;
        case ENC_2: 
            HAL_GPIO_WritePin(CS_ENC2_GPIO_Port, CS_ENC2_Pin, GPIO_PIN_RESET); 
            break;
        case ENC_3: 
            HAL_GPIO_WritePin(CS_ENC3_GPIO_Port, CS_ENC3_Pin, GPIO_PIN_RESET); 
            break;
        case ENC_4: 
            HAL_GPIO_WritePin(CS_ENC4_GPIO_Port, CS_ENC4_Pin, GPIO_PIN_RESET); 
            break;
        default: 
            return; // 无效ID直接退出
    }
    
    // 2. SPI 收发数据 (5个字节)
    HAL_SPI_TransmitReceive(&hspi4, tx_buffer, rx_buffer, 5, 10);
    
    // 3. 根据传入的ID，拉高对应的 CS 引脚 (结束通讯)
    switch (id) {
        case ENC_1: 
            HAL_GPIO_WritePin(CS_ENC1_GPIO_Port, CS_ENC1_Pin, GPIO_PIN_SET); 
            break;
        case ENC_2: 
            HAL_GPIO_WritePin(CS_ENC2_GPIO_Port, CS_ENC2_Pin, GPIO_PIN_SET); 
            break;
        case ENC_3: 
            HAL_GPIO_WritePin(CS_ENC3_GPIO_Port, CS_ENC3_Pin, GPIO_PIN_SET); 
            break;
        case ENC_4: 
            HAL_GPIO_WritePin(CS_ENC4_GPIO_Port, CS_ENC4_Pin, GPIO_PIN_SET); 
            break;
    }

    // 4. 解析数据并存入对应ID的结构体中
    // rx_buffer[0] 是发送0x05时接收的无意义数据，忽略
    encoder_data[id].raw_angle = (rx_buffer[1] << 8) | rx_buffer[2];
    encoder_data[id].status    = rx_buffer[3];
    encoder_data[id].crc       = rx_buffer[4];
    
    // 5. 将16位原始数据转换为实际的360度物理角度
    encoder_data[id].degree = (float)encoder_data[id].raw_angle * 360.0f / 65536.0f;
}

/**
 * @brief 批量更新所有编码器的数据
 */ 
void Update_All_Encoders(void)
{
    // 轮询读取4个编码器
    for(int i = 0; i < 4; i++)
    {
        Read_PQY13_Encoder((Encoder_ID_t)i);
    }
}
