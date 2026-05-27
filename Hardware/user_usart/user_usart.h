#ifndef _USER_USART_H
#define _USER_USART_H

#include "stm32f4xx_hal.h"

#define USART_REC_LEN   200                     /* 定义最大接收字节数 200 */
#define USART_EN_RX     1                       /* 使能（1）/禁止（0）串口1接收 */
#define RXBUFFERSIZE    1                       /* 缓存大小 */

//extern UART_HandleTypeDef huart2;       /* UART句柄 */
extern uint32_t num_input;

extern uint8_t  g_usart_rx_buf[USART_REC_LEN];  /* 接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 */
extern uint16_t g_usart_rx_sta;                 /* 接收状态标记 */
extern uint8_t g_rx_buffer[RXBUFFERSIZE];       /* HAL库USART接收Buffer */
extern uint8_t  print_mode;                     /* 打印模式：1-IMU, 2-Motor, 3-Encoder */


void USART2_init(void);             /* 串口初始化函数 */
void USART2_PrintMessage(void);
void Print_Task(void);
void USART2_ProcessCommand(void);
#endif



