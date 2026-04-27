#include "ps2_handle.h"
#include "ps2.h"
#include "user_usart.h"
#include "CAN_receive.h"
#include "bsp_can.h"
#include "main.h"       // 包含HAL库和GPIO定义
#include "string.h"
#include <stdio.h>
#include "pwm_motor.h"

// 静态变量，用于保存上次模式
static unsigned char Last_PS2_Mode = 0xFF; // 初始化为无效值

void PS2_Control_Handler(void)
{
    unsigned char KeyNum;
    unsigned char PS2_Mode;
    
    // 获取PS2数据
    KeyNum = ps2_key_serch();  // 内部会调用PS2_ReadData()
    PS2_Mode = ps2_mode_get();    
    
    // 模式切换检测
    if (PS2_Mode != Last_PS2_Mode)
    {
//        printf("PS2 mode changed, stopping all motors\r\n");
        Motor_Stop_All();
        CAN_cmd_chassis(0, 0, 0, 0);
    }
    
    // 处理锁定/解锁按键（优先级最高）
    if (ps2_get_key_state(PSB_BLUE))
    {
//        printf("PSB_BLUE pressed - LOCKING!\r\n");
        Motor_Lock_All();
    }
    
    if (ps2_get_key_state(PSB_RED))
    {
//        printf("PSB_RED pressed - UNLOCKING!\r\n");
        Motor_Unlock_All();
    }
    
    // 根据模式处理控制逻辑
    if (PS2_Mode == PSB_REDLIGHT_MODE)
    {
        unsigned char ps2_lx, ps2_ly, ps2_rx, ps2_ry;
        
//        printf("REDLIGHT_MODE\r\n");
        ps2_lx = ps2_get_anolog_data(PSS_LX);
        ps2_ly = ps2_get_anolog_data(PSS_LY);
        ps2_rx = ps2_get_anolog_data(PSS_RX);
        ps2_ry = ps2_get_anolog_data(PSS_RY);
        
        // 处理CAN底盘控制
        if (ps2_get_key_state(PSB_L1))
        {
//            printf("PSB_L1\r\n");
            CAN_cmd_chassis(800, 0, 0, 0);
        }
        else if (ps2_get_key_state(PSB_L2))
        {
//            printf("PSB_L2\r\n");
            CAN_cmd_chassis(-800, 0, 0, 0);
        }
        else if (ps2_get_key_state(PSB_R1))
        {
//            printf("PSB_R1\r\n");
            CAN_cmd_chassis(0, 800, 0, 0);
        }
        else if (ps2_get_key_state(PSB_R2))
        {
//            printf("PSB_R2\r\n");
            CAN_cmd_chassis(0, -800, 0, 0);
        }
        
        // 处理电机控制（受锁定保护）
        if (Motor_Is_Locked())
        {
            printf("Motor system LOCKED - ignoring motor commands\r\n");
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            HAL_Delay(100);
        }
        else
        {
            // 左摇杆Y轴控制电机A
            if (ps2_ly == 0x00)
            {
//                printf("Motor A forward\r\n");
                Motor_SetSpeed(MOTOR_A, 60);
                Motor_SetDirection(MOTOR_A, DIRECTION_FORWARD);
            }
            else if (ps2_ly == 0xFF)
            {
//                printf("Motor A reverse\r\n");
                Motor_SetSpeed(MOTOR_A, 60);
                Motor_SetDirection(MOTOR_A, DIRECTION_REVERSE);
            }
            // 左摇杆X轴控制电机B
            else if (ps2_lx == 0x00)
            {
//                printf("Motor B forward\r\n");
                Motor_SetSpeed(MOTOR_B, 60);
                Motor_SetDirection(MOTOR_B, DIRECTION_FORWARD);
            }
            else if (ps2_lx == 0xFF)
            {
//                printf("Motor B reverse\r\n");
                Motor_SetSpeed(MOTOR_B, 60);
                Motor_SetDirection(MOTOR_B, DIRECTION_REVERSE);
            }
            // 右摇杆Y轴控制电机C
            else if (ps2_ry == 0x00)
            {
//                printf("Motor C forward\r\n");
                Motor_SetSpeed(MOTOR_C, 60);
                Motor_SetDirection(MOTOR_C, DIRECTION_FORWARD);
            }
            else if (ps2_ry == 0xFF)
            {
//                printf("Motor C reverse\r\n");
                Motor_SetSpeed(MOTOR_C, 60);
                Motor_SetDirection(MOTOR_C, DIRECTION_REVERSE);
            }
            // 右摇杆X轴控制电机D
            else if (ps2_rx == 0x00)
            {
//                printf("Motor D forward\r\n");
                Motor_SetSpeed(MOTOR_D, 60);
                Motor_SetDirection(MOTOR_D, DIRECTION_FORWARD);
            }
            else if (ps2_rx == 0xFF)
            {
//                printf("Motor D reverse\r\n");
                Motor_SetSpeed(MOTOR_D, 60);
                Motor_SetDirection(MOTOR_D, DIRECTION_REVERSE);
            }
            else
            {
//                printf("stop\r\n");
                Motor_Stop_All();
            }
        }
    }
    else if (PS2_Mode == PSB_GREENLIGHT_MODE)  // 绿灯数字模式
    {
//        printf("GREENLIGHT_MODE\r\n");
        
        if (KeyNum)
        {
            // 方向键控制（示例）
            if (ps2_get_key_state(PSB_PAD_UP))
            {
                printf("PSB_PAD_UP\r\n");
                // 添加向上逻辑
            }
            else if (ps2_get_key_state(PSB_PAD_DOWN))
            {
                printf("PSB_PAD_DOWN\r\n");
                // 添加向下逻辑
            }
            else if (ps2_get_key_state(PSB_PAD_LEFT))
            {
                printf("PSB_PAD_LEFT\r\n");
                // 添加向左逻辑
            }
            else if (ps2_get_key_state(PSB_PAD_RIGHT))
            {
                printf("PSB_PAD_RIGHT\r\n");
                // 添加向右逻辑
            }
            else if (ps2_get_key_state(PSB_L1))
            {
                printf("PSB_L1\r\n");
                // 添加L1逻辑
            }
            else if (ps2_get_key_state(PSB_L2))
            {
                printf("PSB_L2\r\n");
                // 添加L2逻辑
            }
            else
            {
//                printf("stop\r\n");
                Motor_Stop_All();
            }
        }
        else
        {
//            printf("stop\r\n");
            Motor_Stop_All();
        }
    }
    else  // 未识别模式
    {
//        printf("stop\r\n");
        Motor_Stop_All();
        CAN_cmd_chassis(0, 0, 0, 0);
    }
    
    // 更新上次模式
    Last_PS2_Mode = PS2_Mode;
    
    // 控制循环延时
    HAL_Delay(100);
//    PS2_Delay_US(100);
}
