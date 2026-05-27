#include "usart.h"
#include "CAN_receive.h"
#include "encoder_counter.h"
#include "user_usart.h"
#include "string.h"
#include <stdio.h>
#include "usart6.h"
#include "spi4.h"

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
//        // 【防挂死优化】：在重新开启中断前，强制清除可能存在的溢出错误标志(ORE)
//        __HAL_UART_CLEAR_OREFLAG(huart);
        
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
 * @brief       串口命令解析器（基于高鲁棒性数值转换逻辑）
 * @note        在 main循环 内部调用，自动处理来自串口助手的指令切换
 */
void USART2_ProcessCommand(void)
{
    uint8_t i;
    uint8_t len;
    uint32_t value = 0;
    uint8_t valid_digits = 0;
    
    // 【防挂死机制】高频打印容易触发 ORE 错误导致中断关闭，在这里强制定期清除
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    
    // 1. 检查是否接收完成（最高位为1表示接收到了完整的 \r\n）
    // 如果没有接收完，直接默默返回，绝对不要在这里打印任何东西，否则会冲刷屏幕
    if ((g_usart_rx_sta & 0x8000) == 0)  
    {
        return;  // 未接收到完整数据，直接返回
    }
    
//    // ===== 能走到这里，说明串口【真正、完整】地收到了一包带回车的数据 =====
//    printf("[DEBUG] 串口接收触发！当前接收寄存器状态码 sta: 0x%04X\r\n", g_usart_rx_sta);
    
    // 2. 获取接收到的有效数据长度
    len = g_usart_rx_sta & 0x3FFF;  
    
    // 3. 检查数据长度是否合理
    if (len == 0 || len >= USART_REC_LEN)
    {
        g_usart_rx_sta = 0;  // 状态错误，清空接收标志
//        printf("[ERROR] 接收长度错误，len = %d\r\n", len);
        return;  
    }
    
    // 4. 遍历接收到的数据，转换为数字
    for (i = 0; i < len; i++)
    {
        /* 只处理数字字符 */
        if (g_usart_rx_buf[i] >= '0' && g_usart_rx_buf[i] <= '9')
        {
            value = value * 10 + (g_usart_rx_buf[i] - '0');
            valid_digits++;
            
            /* 安全限制：防止数值溢出 */
            if (value > 100) // 我们的模式只有1~3，限制到100以内足够了
            {
                g_usart_rx_sta = 0;
//                printf("\r\n[ERROR] 模式数字过大！\r\n\r\n");
                return;  
            }
        }
        /* 如果遇到回车换行，提前结束解析 */
        else if (g_usart_rx_buf[i] == 0x0D || g_usart_rx_buf[i] == 0x0A)
        {
            break;
        }
        /* 其他非法字符（如字母、空格等） */
        else
        {
            g_usart_rx_sta = 0;
//            printf("\r\n[ERROR] 包含非法字符 '%c'！\r\n\r\n", g_usart_rx_buf[i]);
            return;  
        }
    }
    
    // 5. 检查是否至少有一个有效数字
    if (valid_digits == 0)
    {
        g_usart_rx_sta = 0;
//        printf("[WARNING] 未识别到任何有效数字\r\n");
        return;  
    }
    
    // 6. 成功解析，根据数值执行模式切换
    if (value >= 1 && value <= 3)
    {
        print_mode = (uint8_t)value; // 改变全局打印模式
        printf("\r\n>>> [SYS] 成功切换至打印模式 [%d] <<<\r\n\r\n", print_mode);
    }
    else
    {
        printf("\r\n[WARNING] 模式 %d 不存在！请输入 1, 2 或 3\r\n\r\n", value);
    }
    
    // 7. 必须清空接收标志，准备下一次接收
    g_usart_rx_sta = 0;  
}

/**
 * @brief  多子任务打印管理，在串口输入数字进行切换
 */
void Print_Task(void)
{

    // 1. main中循环调用USART2_ProcessCommand解析串口数据
    
    // 2. 获取IMU数据指针
    protocol_info_t *imu = IMU_GetOutputInfo();
    
    // 3. 根据当前 print_mode 选择对应的打印子任务
    switch (print_mode)
    {
        case 1:
        {
            /* --------------- 子任务 1：打印姿态与加速度数据 --------------- */ 
            printf("=== IMU Data (0.5s) ===\r\n");
            printf("Euler: Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n", 
                   imu->roll, imu->pitch, imu->yaw); // 姿态 
            printf("Acc: %.3f, %.3f, %.3f\r\n", 
                   imu->accel_x, imu->accel_y, imu->accel_z); // 加速度
            printf("Angle: %.2f, %.2f, %.2f\r\n", 
                   imu->angle_x, imu->angle_y, imu->angle_z); // 角速度
            break;
        }
        
        case 2:
        {
            /* --------------- 子任务 2：默认打印 M3508 实时数据 --------------- */
            int32_t turn0 = Encoder_Get_Turn_Count(0);
            int32_t turn1 = -Encoder_Get_Turn_Count(1);
            int32_t speed_rpm0 = motor_chassis[0].speed_rpm / 36;
            int32_t speed_rpm1 = motor_chassis[1].speed_rpm / 36;
            int32_t current0 = motor_chassis[0].given_current;
            int32_t current1 = motor_chassis[1].given_current;
            int32_t temp0 = motor_chassis[0].temperate;
            int32_t temp1 = motor_chassis[1].temperate;
            
            printf("=== M3508 Motor Data ===\r\n");
            printf("M1  speed_rpm:%d current:%d turns:%d temp:%d\r\n",
                   speed_rpm0, current0, turn0, temp0);
            printf("M2  speed_rpm:%d current:%d turns:%d temp:%d\r\n",
                   speed_rpm1, current1, turn1, temp1); // 温度大于80度过热
            break;
        }
        
        case 3:
        {
            /* --------------- 子任务 3：专门打印编码器角度数据 --------------- */
            printf("=== Encoder Degree Data ===\r\n");
            printf("ENC: %.1f | %.1f | %.1f | %.1f\r\n", 
                    encoder_data[ENC_1].degree, 
                    encoder_data[ENC_2].degree, 
                    encoder_data[ENC_3].degree, 
                    encoder_data[ENC_4].degree);
            break;
        }
        
        default:
            // 兜底防御，防止外界异常篡改变量
            print_mode = 1;
            break;
    }
}

///**
// * @brief  打印任务 - 默认打印M3508数据，按下PSB_PINK后切换打印姿态数据
// */
//void Print_Task(void)
//{
//    
//    // 获取IMU数据指针
//    protocol_info_t *imu = IMU_GetOutputInfo();
//    
//    // 根据模式执行不同的打印
//    if (print_mode == 1)
//    {
//        /* --------------- 打印姿态数据 --------------- */ 
//        printf("=== IMU Data (0.5s) ===\r\n");
//        printf("Euler: Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n", 
//               imu->roll, imu->pitch, imu->yaw);//姿态 
//        printf("Acc: %.3f, %.3f, %.3f\r\n", 
//               imu->accel_x, imu->accel_y, imu->accel_z);//加速度
//        printf("Angle: %.2f, %.2f, %.2f\r\n", 
//               imu->angle_x, imu->angle_y, imu->angle_z);//角速度
//        
//        printf("ENC: %.1f | %.1f | %.1f | %.1f\r\n", 
//                encoder_data[ENC_1].degree, 
//                encoder_data[ENC_2].degree, 
//                encoder_data[ENC_3].degree, 
//                encoder_data[ENC_4].degree);
//    }
//    else
//    {
//        int32_t turn0 = Encoder_Get_Turn_Count(0);
//        int32_t turn1 = - Encoder_Get_Turn_Count(1);
//        int32_t speed_rpm0 = motor_chassis[0].speed_rpm/36;
//        int32_t speed_rpm1 = motor_chassis[1].speed_rpm/36;
//        int32_t current0 = motor_chassis[0].given_current;
//        int32_t current1 = motor_chassis[1].given_current;
//        int32_t temp0 = motor_chassis[0].temperate;
//        int32_t temp1 = motor_chassis[1].temperate;
//        /* --------------- 默认：打印M3508实时数据 --------------- */
//        printf("=== M3508 Motor Data ===\r\n");
//        printf("M1  speed_rpm:%d current:%d turns:%d temp:%d\r\n",
//               speed_rpm0, 
//               current0,
//               turn0,
//               temp0);
//        printf("M2  speed_rpm:%d current:%d turns:%d temp:%d\r\n",
//               speed_rpm1, 
//               current1,
//               turn1,
//               temp1);//温度大于80度过热
//    }
//}
#endif


 

 




