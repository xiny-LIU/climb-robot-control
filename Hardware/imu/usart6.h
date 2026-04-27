#ifndef _USART6_H
#define _USART6_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdbool.h>
#include "analysis_data.h" 
/*======================================================
 *  用户配置区域 - 根据CubeMX生成的宏修改
 *======================================================*/
#ifndef IMU_USART
  #define IMU_USART           USART6
#endif

#ifndef IMU_USART_DMA
  #define IMU_USART_DMA       DMA2
#endif

#ifndef IMU_USART_RX_DMA_STREAM
  #define IMU_USART_RX_DMA_STREAM  DMA2_Stream1  // USART6_RX 通常是 DMA2_Stream1
#endif

#ifndef IMU_USART_RX_DMA_CHANNEL
  #define IMU_USART_RX_DMA_CHANNEL DMA_CHANNEL_5   // USART6_RX 对应 Channel 5
#endif

/* 帧定义 */
#define IMU_FRAME_HEADER_1      0x59
#define IMU_FRAME_HEADER_2      0x53
#define IMU_FRAME_LENGTH        67          // 固定帧长
#define IMU_PAYLOAD_POS         5           // Payload起始位置

/* 状态机状态 */
typedef enum {
    IMU_STATE_IDLE = 0,     // 等待帧头1
    IMU_STATE_HEAD_1,       // 收到帧头1，等待帧头2
    IMU_STATE_RECEIVING,    // 正在接收数据
    IMU_STATE_PROCESS       // 数据就绪待处理
} imu_recv_state_t;

/* 接收缓冲区结构 */
typedef struct {
    uint8_t  rx_buf[IMU_FRAME_LENGTH];      // DMA缓冲区
    uint8_t  frame_buf[IMU_FRAME_LENGTH];   // 处理缓冲区（双缓冲）
    uint16_t rx_len;                        // 当前接收长度
    uint8_t  data_ready;                    // 数据就绪标志
} imu_rx_buffer_t;

/*======================================================
 *  对外接口函数
 *======================================================*/

/**
 * @brief  IMU串口初始化（在MX_USART6_UART_Init后调用）
 * @param  huart: CubeMX生成的USART6句柄指针
 * @retval None
 */
void IMU_USART_Init(UART_HandleTypeDef *huart);

/**
 * @brief  启动IMU数据接收（DMA循环模式或中断模式）
 * @param  None
 * @retval None
 */
void IMU_USART_StartReceive(void);

/**
 * @brief  处理接收到的IMU数据（放在主循环或定时器中调用）
 * @param  None
 * @retval 0:无新数据  1:处理成功  -1:校验失败
 */
int IMU_USART_ProcessData(void);

/**
 * @brief  获取最新解析后的IMU数据结构体指针
 * @param  None
 * @retval protocol_info_t指针（来自analysis_data.h）
 */
protocol_info_t* IMU_GetOutputInfo(void);

/**
 * @brief  检查是否有新数据就绪（非阻塞查询）
 * @param  None
 * @retval 0:无新数据  1:有新数据
 */
uint8_t IMU_IsDataReady(void);

/**
 * @brief  清除数据就绪标志
 * @param  None
 * @retval None
 */
void IMU_ClearDataReady(void);

/**
 * @brief  字节接收处理函数（用于非DMA模式或备用）
 * @param  byte: 接收到的字节
 * @retval None
 */
void IMU_ByteReceived(uint8_t byte);

/**
 * @brief  DMA接收完成处理（普通函数形式，非回调）
 * @param  huart: 串口句柄
 * @retval None
 */
void IMU_DMA_RxCpltHandler(UART_HandleTypeDef *huart);

/**
 * @brief  串口错误处理（普通函数形式，非回调）
 * @param  huart: 串口句柄
 * @retval None
 */
void IMU_UART_ErrorHandler(UART_HandleTypeDef *huart);

/**
 * @brief  IMU处理任务 - 供定时器调用
 */
void IMU_Process_Task(void);
/*======================================================
 *  全局变量声明
 *======================================================*/
extern imu_rx_buffer_t g_imu_rx;
extern volatile uint8_t g_imu_frame_ready;


#endif



