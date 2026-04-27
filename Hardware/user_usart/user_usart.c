#include "usart.h"
#include "CAN_receive.h"
#include "encoder_counter.h"
#include "user_usart.h"
#include "string.h"
#include <stdio.h>
#include "usart6.h"

#include "ps2_control.h"
#include "ps2.h"
/* 如果使用os,则包括下面的头文件即可 */
#if SYS_SUPPORT_OS
#include "os.h"                               /* os 使用 */
#endif

/******************************************************************************************/
/* 加入以下代码, 支持printf函数, 而不需要选择use MicroLIB */

#if 1
#if (__ARMCC_VERSION >= 6010050)                    /* 使用AC6编译器时 */
__asm(".global __use_no_semihosting\n\t");          /* 声明不使用半主机模式 */
__asm(".global __ARM_use_no_argv \n\t");            /* AC6下需要声明main函数为无参数格式，否则部分例程可能出现半主机模式 */

#else
/* 使用AC5编译器时, 要在这里定义__FILE 和 不使用半主机模式 */
#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};

#endif

/* 不使用半主机模式，至少需要重定义_ttywrch\_sys_exit\_sys_command_string函数,以同时兼容AC6和AC5模式 */
int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

/* 定义_sys_exit()以避免使用半主机模式 */
void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

/* FILE 在 stdio.h里面定义. */
FILE __stdout;

/* 重定义fputc函数, printf函数最终会通过调用fputc输出字符串到串口 */
int fputc(int ch, FILE *f)
{
    while ((USART2->SR & 0X40) == 0);               /* 等待上一个字符发送完成 */

    USART2->DR = (uint8_t)ch;                       /* 将要发送的字符 ch 写入到DR寄存器 */
    return ch;
}
#endif
/***********************************************END*******************************************/
#if USART_EN_RX                                     /* 如果使能了接收 */

/* 接收缓冲, 最大USART_REC_LEN个字节. */
uint8_t g_usart_rx_buf[USART_REC_LEN];

/*  接收状态
 *  bit15，      接收完成标志
 *  bit14，      接收到0x0d
 *  bit13~0，    接收到的有效字节数目
*/
uint16_t g_usart_rx_sta = 0;

uint8_t g_rx_buffer[RXBUFFERSIZE];    /* HAL库使用的串口接收缓冲 */

//UART_HandleTypeDef huart2;    /* UART句柄 */

/**
 * @brief       串口X中断服务函数
 * @param       无
 * @retval      无
 */
/*stm_it文件里有*/
//void USART_UX_IRQHandler(void)
//{ 
//#if SYS_SUPPORT_OS                              /* 使用OS */
//    OSIntEnter();    
//#endif

//    HAL_UART_IRQHandler(&huart2);       /* 调用HAL库中断处理公用函数 */

//#if SYS_SUPPORT_OS                              /* 使用OS */
//    OSIntExit();
//#endif
//}


///**
// * @brief       串口X初始化函数
// * @param       baudrate: 波特率, 根据自己需要设置波特率值
// * @retval      无
// */
//void usart_init(uint32_t baudrate)
void USART2_init(void)
{
    
//    /* 该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量 */
    HAL_UART_Receive_IT(&huart2, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
}


/**
 * @brief       Rx传输回调函数
 * @param       huart: UART句柄类型指针
 * @retval      无
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    
    if (huart->Instance == USART2)            /* 如果是串口2 */
    {   
        if ((g_usart_rx_sta & 0x8000) == 0)     /* 接收未完成 */
        {
            if (g_usart_rx_sta & 0x4000)        /* 接收到了0x0d */
            {
                if (g_rx_buffer[0] != 0x0a)      /* 没接收到0x0a */
                {
                    g_usart_rx_sta = 0;         /* 接收错误,重新开始 */
                }
                else 
                {
                    g_usart_rx_sta |= 0x8000;   /* 接收完成了 */
                }
            }
            else                                /* 还没收到0X0D */
            {
                if (g_rx_buffer[0] == 0x0d)
                {
                    g_usart_rx_sta |= 0x4000;
                }
                else
                {
                    g_usart_rx_buf[g_usart_rx_sta & 0X3FFF] = g_rx_buffer[0] ;
                    g_usart_rx_sta++;
                    if (g_usart_rx_sta > (USART_REC_LEN - 1))
                    {
                        g_usart_rx_sta = 0;     /* 接收数据错误,重新开始接收 */
                    }
                }
            }
        }
        HAL_UART_Receive_IT(&huart2, (uint8_t *)g_rx_buffer, RXBUFFERSIZE);
    }
    else if (huart->Instance == USART6)
    {
        IMU_DMA_RxCpltHandler(huart);
    }
}
///* --------------- 打印接收到的指令（调试用） --------------- */
//uint8_t len;
//uint16_t times = 0;
//void USART2_PrintMessage(void)
//{

// if (g_usart_rx_sta & 0x8000)                                                    /* 接收到了数据? */
//    {
//        len = g_usart_rx_sta & 0x3fff;                                              /* 得到此次接收到的数据长度 */
//        printf("\r\n您发送的消息为:\r\n");

//        HAL_UART_Transmit(&huart2, (uint8_t *)g_usart_rx_buf, len, 1000);   /* 发送接收到的数据 */
//        while(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC) != SET);           /* 等待发送结束 */
//        printf("\r\n\r\n");                                                         /* 插入换行 */
//        g_usart_rx_sta = 0;
//    }
//    else
//    {
//        times++;

//        if (times % 50000 == 0)
//        {
//            printf("\r\n串口调试\r\n");
//        }

//        if (times % 2000 == 0) 
//        {
//            printf("请输入数据,以回车键结束\r\n");
//        }

//        if (times % 300  == 0)
//        {
////            LED0_TOGGLE();                                         /* 闪烁LED,提示系统正在运行 */
//            printf("系统正在运行\r\n");
//        }

//        HAL_Delay(10);
//    }
//}



/**
 * @brief  打印任务 - 默认打印M3508数据，按下PSB_PINK后切换打印姿态数据
 */
void Print_Task(void)
{
    
    // 获取IMU数据指针
    protocol_info_t *imu = IMU_GetOutputInfo();
    
    // 根据模式执行不同的打印
    if (print_mode == 1)
    {
        /* --------------- 打印姿态数据 --------------- */ 
        printf("=== IMU Data (0.5s) ===\r\n");
        printf("Euler: Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n", 
               imu->roll, imu->pitch, imu->yaw);//姿态 
        printf("Acc: %.3f, %.3f, %.3f\r\n", 
               imu->accel_x, imu->accel_y, imu->accel_z);//加速度
        printf("Angle: %.2f, %.2f, %.2f\r\n", 
               imu->angle_x, imu->angle_y, imu->angle_z);//角速度
    }
    else
    {
        int32_t turn0 = Encoder_Get_Turn_Count(0);
        int32_t turn1 = - Encoder_Get_Turn_Count(1);
        int32_t speed_rpm0 = motor_chassis[0].speed_rpm/36;
        int32_t speed_rpm1 = motor_chassis[1].speed_rpm/36;
        int32_t current0 = motor_chassis[0].given_current;
        int32_t current1 = motor_chassis[1].given_current;
        int32_t temp0 = motor_chassis[0].temperate;
        int32_t temp1 = motor_chassis[1].temperate;
        /* --------------- 默认：打印M3508实时数据 --------------- */
        printf("=== M3508 Motor Data ===\r\n");
        printf("M1  speed_rpm:%d current:%d turns:%d temp:%d\r\n",
               speed_rpm0, 
               current0,
               turn0,
               temp0);
        printf("M2  speed_rpm:%d current:%d turns:%d temp:%d\r\n",
               speed_rpm1, 
               current1,
               turn1,
               temp1);//温度大于80度过热
    }
}
#endif


 

 




