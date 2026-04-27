#include "bluetooth_motor.h"
#include "user_usart.h"
#include "CAN_receive.h"
#include "string.h"
#include "stdio.h"

/* 私有变量 */
static uint32_t last_send_time = 0;
static uint8_t output_enabled = 1;
static char tx_buffer[128];  /* 发送缓冲区 */

/**
 * @brief  初始化蓝牙电机数据输出模块
 * @note   会自动初始化USART2
 * @retval None
 */
void BT_Init(void)
{
    /* 初始化USART2（蓝牙串口） */
    USART2_init();
    
    output_enabled = 1;
    last_send_time = HAL_GetTick();
    
    printf("start\r\n");
}

/**
 * @brief  格式化并发送电机数据
 * @note   私有函数，内部使用
 * @retval None
 */
static void SendMotorData(void)
{
    const motor_measure_t *motor1 = get_chassis_motor_measure_point(0);
    const motor_measure_t *motor2 = get_chassis_motor_measure_point(1);
    
    if (motor1 == NULL || motor2 == NULL) return;
    
#if BT_OUTPUT_FORMAT_JSON
    /* JSON格式输出，便于上位机解析 */
    int len = snprintf(tx_buffer, sizeof(tx_buffer), 
        "{\"M1\":{\"A\":%d,\"S\":%d,\"C\":%d},\"M2\":{\"A\":%d,\"S\":%d,\"C\":%d}}\r\n",
        motor1->ecd,
        motor1->speed_rpm,
        motor1->given_current,
        motor2->ecd,
        motor2->speed_rpm,
        motor2->given_current
    );
#else
    /* CSV格式输出：M1,角度,转速,电流|M2,角度,转速,电流 */
    int len = snprintf(tx_buffer, sizeof(tx_buffer), 
        "M1,%d,%d,%d|M2,%d,%d,%d\r\n",
        motor1->ecd,
        motor1->speed_rpm,
        motor1->given_current,
        motor2->ecd,
        motor2->speed_rpm,
        motor2->given_current
    );
#endif

    /* 通过USART2发送（使用printf重定向） */
    printf("%s", tx_buffer);
}

/**
 * @brief  主循环处理函数，按周期自动发送数据
 * @note   需要放在主循环中循环调用
 * @retval None
 */
void BT_Process(void)
{
    if (!output_enabled) return;
    
    uint32_t current_time = HAL_GetTick();
    
    /* 检查是否到达发送周期 */
    if (current_time - last_send_time >= BT_OUTPUT_PERIOD_MS)
    {
        last_send_time = current_time;
        SendMotorData();
    }
}

/**
 * @brief  立即发送一次当前电机数据（无视周期限制）
 * @retval None
 */
void BT_SendData(void)
{
    SendMotorData();
}

/**
 * @brief  设置数据输出使能状态
 * @param  enable: 1-使能输出, 0-禁用输出
 * @retval None
 */
void BT_SetOutputEnable(uint8_t enable)
{
    output_enabled = enable;
    if (enable)
    {
        printf("[BluetoothMotor] Output Enabled\r\n");
    }
    else
    {
        printf("[BluetoothMotor] Output Disabled\r\n");
    }
}
