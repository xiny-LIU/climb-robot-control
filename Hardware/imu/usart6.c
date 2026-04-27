#include "usart6.h"
#include <stdint.h>
#include "usart.h"
#include "analysis_data.h"  // 包含协议解析头文件
#include <string.h>

/*======================================================
 *  全局变量定义
 *======================================================*/

// 接收缓冲区
imu_rx_buffer_t g_imu_rx = {0};

// 帧就绪标志（DMA完成时置位）
volatile uint8_t g_imu_frame_ready = 0;

// 串口句柄指针（保存CubeMX生成的句柄）
static UART_HandleTypeDef *g_imu_huart = NULL;

// 状态机变量（用于字节模式备用）
static imu_recv_state_t g_recv_state = IMU_STATE_IDLE;
static uint8_t g_temp_buf[IMU_FRAME_LENGTH];
static uint8_t g_recv_index = 0;

// 外部声明协议解析用的全局变量
extern protocol_info_t g_output_info;

/*======================================================
 *  内部辅助函数
 *======================================================*/

/**
 * @brief  查找帧头位置
 * @param  buf: 缓冲区
 * @param  len: 长度
 * @retval 帧头位置，-1表示未找到
 */
static int find_frame_header(uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len - 1; i++) {
        if (buf[i] == IMU_FRAME_HEADER_1 && buf[i+1] == IMU_FRAME_HEADER_2) {
            return i;
        }
    }
    return -1;
}

/**
 * @brief  复制帧到处理缓冲区
 * @param  src: 源地址
 * @param  dst: 目标地址
 * @param  len: 长度
 * @retval None
 */
static void copy_frame(uint8_t *src, uint8_t *dst, uint16_t len)
{
    memcpy(dst, src, len);
}

/*======================================================
 *  对外接口实现
 *======================================================*/

/**
 * @brief  IMU串口初始化
 */
void IMU_USART_Init(UART_HandleTypeDef *huart)
{
    g_imu_huart = huart;
    
    // 清空缓冲区
    memset(&g_imu_rx, 0, sizeof(g_imu_rx));
    g_imu_frame_ready = 0;
    g_recv_state = IMU_STATE_IDLE;
    g_recv_index = 0;
    
    // 清空协议数据
    memset(&g_output_info, 0, sizeof(protocol_info_t));
}

/**
 * @brief  启动DMA接收（循环模式）
 * 
 * 方案A: 循环DMA模式 - 持续接收，自动覆盖
 * 方案B: 普通DMA模式 - 接收固定长度后停止
 * 
 * 这里使用方案B（普通模式），接收67字节后触发中断
 */
void IMU_USART_StartReceive(void)
{
    if (g_imu_huart == NULL) return;
    
    // 方案B: 普通DMA模式，接收IMU_FRAME_LENGTH字节
    HAL_UART_Receive_DMA(g_imu_huart, g_imu_rx.rx_buf, IMU_FRAME_LENGTH);
    
    // 如果想用方案A（循环模式），取消下面注释：
    // HAL_UART_Receive_DMA(g_imu_huart, g_imu_rx.rx_buf, IMU_FRAME_LENGTH);
    // 需要配置DMA为循环模式：g_imu_huart->hdmarx->Instance->CR |= DMA_SxCR_CIRC;
}

/**
 * @brief  处理接收到的数据（主循环调用）
 * 
 * 调用流程：
 * 1. 检查g_imu_frame_ready标志
 * 2. 复制数据到处理缓冲区
 * 3. 调用analysis_data解析
 * 4. 重启DMA接收
 */
int IMU_USART_ProcessData(void)
{
    int result = 0;
    
    // 检查是否有新数据（由DMA完成中断置位）
    if (!g_imu_frame_ready) {
        return 0;  // 无新数据
    }
    
    // 复制到处理缓冲区（双缓冲，防止DMA覆盖）
    copy_frame(g_imu_rx.rx_buf, g_imu_rx.frame_buf, IMU_FRAME_LENGTH);
    g_imu_frame_ready = 0;  // 清除标志
    
    // 验证帧头
    if (g_imu_rx.frame_buf[0] != IMU_FRAME_HEADER_1 || 
        g_imu_rx.frame_buf[1] != IMU_FRAME_HEADER_2) {
        // 帧头错误，尝试查找正确帧头
        int pos = find_frame_header(g_imu_rx.frame_buf, IMU_FRAME_LENGTH);
        if (pos < 0) {
            // 本帧无有效数据，重启接收
            IMU_USART_StartReceive();
            return -1;
        }
        // 移动数据使帧头对齐（简化处理：直接丢弃本帧）
        IMU_USART_StartReceive();
        return -1;
    }
    
    // 调用协议解析函数
    //analysis_data() 是原F103代码，我们传入 frame_buf 和长度67，它自动解析并存入 g_output_info
    int parse_result = analysis_data(g_imu_rx.frame_buf, IMU_FRAME_LENGTH);
    
    if (parse_result == analysis_ok) {
        result = 1;  // 解析成功
        g_imu_rx.data_ready = 1;
    } else {
        result = -1; // 校验失败或其他错误
    }
    
    // 重新启动DMA接收（关键！）
    IMU_USART_StartReceive();
    
    return result;
}

/**
 * @brief  字节模式处理（备用/调试）
 */
void IMU_ByteReceived(uint8_t byte)
{
    switch (g_recv_state) {
        case IMU_STATE_IDLE:
            if (byte == IMU_FRAME_HEADER_1) {
                g_temp_buf[0] = byte;
                g_recv_index = 1;
                g_recv_state = IMU_STATE_HEAD_1;
            }
            break;
            
        case IMU_STATE_HEAD_1:
            if (byte == IMU_FRAME_HEADER_2) {
                g_temp_buf[1] = byte;
                g_recv_index = 2;
                g_recv_state = IMU_STATE_RECEIVING;
            } else if (byte == IMU_FRAME_HEADER_1) {
                // 仍然是帧头1，保持状态
                g_temp_buf[0] = byte;
                g_recv_index = 1;
            } else {
                g_recv_state = IMU_STATE_IDLE;
            }
            break;
            
        case IMU_STATE_RECEIVING:
            g_temp_buf[g_recv_index++] = byte;
            if (g_recv_index >= IMU_FRAME_LENGTH) {
                // 接收完成，复制到缓冲区
                copy_frame(g_temp_buf, g_imu_rx.frame_buf, IMU_FRAME_LENGTH);
                g_imu_frame_ready = 1;
                g_recv_state = IMU_STATE_IDLE;
                g_recv_index = 0;
            }
            break;
            
        default:
            g_recv_state = IMU_STATE_IDLE;
            g_recv_index = 0;
            break;
    }
}

/**
 * @brief  DMA接收完成处理（普通函数，在HAL回调中调用）
 * 
 * 使用方式：在stm32f4xx_it.c的DMA中断中调用，或在HAL_UART_RxCpltCallback中调用
 */
void IMU_DMA_RxCpltHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance != IMU_USART) return;
    
    // 设置帧就绪标志
    g_imu_frame_ready = 1;
    
    // 可选：立即处理（不推荐，建议在主循环处理）
    // IMU_USART_ProcessData();
}

/**
 * @brief  串口错误处理（普通函数）
 */
void IMU_UART_ErrorHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance != IMU_USART) return;
    
    // 清除错误标志
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    
    // 重新启动接收
    HAL_UART_DMAStop(huart);
    IMU_USART_StartReceive();
}

/**
 * @brief  IMU处理任务 - 供定时器调用
 * 
 * 原main循环中的轮询逻辑移到这里
 */
void IMU_Process_Task(void)
{
    int result = IMU_USART_ProcessData();
    
    if (result == 1) {
        // 数据解析成功，可在这里触发后续处理
        // 例如：更新姿态控制变量、检查异常等
        
        // 示例：将数据复制到控制用变量（避免在中断中访问）
        // 或者设置另一个标志通知主循环
    }
    else if (result == -1) {
        // 校验失败，可记录错误计数
        // g_imu_error_count++;
    }
}


/*======================================================
 *  数据获取接口
 *======================================================*/

protocol_info_t* IMU_GetOutputInfo(void)
{
    return &g_output_info;
}

uint8_t IMU_IsDataReady(void)
{
    return g_imu_rx.data_ready;
}

void IMU_ClearDataReady(void)
{
    g_imu_rx.data_ready = 0;
}



