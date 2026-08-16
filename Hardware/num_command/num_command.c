#include "num_command.h"
#include <stdio.h>  /* 用于printf */

/* 通过串口出入电流值，赋值给num_control用于电机运行 

*/

/* 电机控制值全局变量定义 */
uint32_t num_control = 0;

/**
 * @brief       解析串口接收的电机控制值
 * @param       无
 * @retval      0: 解析成功, 1: 解析失败
 */
uint8_t ParseMotorControlValue(void)
{
    uint8_t i;
    uint8_t len;
    uint32_t value = 0;
    uint8_t valid_digits = 0;
    
    if ((g_usart_rx_sta & 0x8000) == 0)  /* 检查是否接收完成 */
    {
        return 1;  /* 未接收到完整数据 */
    }
    
    len = g_usart_rx_sta & 0x3FFF;  /* 获取接收到的数据长度 */
    
    /* 检查数据长度是否合理 */
    if (len == 0 || len >= USART_REC_LEN)
    {
        g_usart_rx_sta = 0;  /* 清空接收标志 */
        return 1;  /* 数据长度错误 */
    }
    
    /* 遍历接收到的数据，转换为数字 */
    for (i = 0; i < len; i++)
    {
        /* 只处理数字字符 */
        if (g_usart_rx_buf[i] >= '0' && g_usart_rx_buf[i] <= '9')
        {
            value = value * 10 + (g_usart_rx_buf[i] - '0');
            valid_digits++;
            
            /* 防止数值溢出（最大支持999999 mA） */
            if (value > 16384)
            {
                g_usart_rx_sta = 0;
                return 1;  /* 数值过大 */
            }
        }
        /* 如果遇到回车换行，结束解析 */
        else if (g_usart_rx_buf[i] == 0x0D || g_usart_rx_buf[i] == 0x0A)
        {
            break;
        }
        /* 其他非法字符 */
        else
        {
            g_usart_rx_sta = 0;
            return 1;  /* 包含非法字符 */
        }
    }
    
    /* 检查是否至少有一个有效数字 */
    if (valid_digits == 0)
    {
        g_usart_rx_sta = 0;
        return 1;  /* 未收到有效数字 */
    }
    
    /* 成功解析，保存控制值 */
    num_control = value;
    g_usart_rx_sta = 0;  /* 清空接收标志 */
    
    return 0;  /* 解析成功 */
}

/**
 * @brief       USART6命令处理函数
 * @param       无
 * @retval      无
 */
void USART6_ProcessCommand(void)
{
    static uint16_t times = 0;  /* 静态变量用于状态提示 */
    
    if (g_usart_rx_sta & 0x8000)  /* 接收到数据 */
    {
        /* 尝试解析电机控制值 */
        if (ParseMotorControlValue() == 0)
        {
            /* 解析成功，打印确认信息 */
            printf("\r\n电机电流设置为: %u mA\r\n", num_control);
            
            /* 在这里调用你的电机控制函数 */
            /* Motor_SetCurrent(num_control); */
        }
        else
        {
            /* 解析失败，打印错误信息 */
            printf("\r\n错误: 请输入有效的数字(如: 1500)\r\n");
        }
    }
    else
    {
        /* 未接收到数据时的状态提示（可选） */
        times++;
        
        if (times % 50000 == 0)
        {
            printf("\r\n电机控制系统已就绪\r\n");
        }
        
        if (times % 2000 == 0) 
        {
            printf("请输入电流值(mA),按回车键结束\r\n");
        }
        
        if (times % 300 == 0)
        {
            printf("系统运行中...\r\n");
        }
    }
}

