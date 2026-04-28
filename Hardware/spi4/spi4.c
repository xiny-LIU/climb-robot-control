#include "spi4.h"
#include "spi.h"  // 引入CubeMX生成的spi头文件，以使用 hspi4 句柄

// 实例化编码器数据变量
PQY13_Data_t encoder_data;

// 读取PQY13编码器函数
void Read_PQY13_Encoder(void)
{
    // tx_buffer: 第一个字节是命令 0x05，后面补4个 0x00 用于产生时钟信号来接收数据
    uint8_t tx_buffer[5] = {0x05, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buffer[5] = {0};

    // 1. 拉低片选 (CS)，开始通讯 (以 PE4 为例)
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET); 
    
    // 2. 收发数据
    HAL_SPI_TransmitReceive(&hspi4, tx_buffer, rx_buffer, 5, 10);
    
    // 3. 拉高片选 (CS)，结束通讯
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);   

    // 4. 解析数据
    // rx_buffer[0] 是发送0x05时同步接收的无效数据
    // rx_buffer[1] 和 rx_buffer[2] 是16位角度数据 (高位在前)
    encoder_data.raw_angle = (rx_buffer[1] << 8) | rx_buffer[2];
    
    // 状态字和CRC
    encoder_data.status = rx_buffer[3];
    encoder_data.crc = rx_buffer[4];

    // 5. 将16位原始数据转换为实际的360度物理角度
    encoder_data.degree = (float)encoder_data.raw_angle * 360.0f / 65536.0f;
}
