#include "led.h"

//轮询方式
void Led_Task(void)
{
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
    {
        HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET);// 点亮
        HAL_Delay(1000);  // ? 注意：这里延时2秒，会阻塞主循环
        HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);// 熄灭
//        printf("hello");
    }
    HAL_Delay(10);  // 消抖延时
}


////中断回调方式
//// ********** 内部私有变量（static隔离，不污染全局）**********
//uint8_t key_press_flag = 0;  // 按键中断触发标志
//static volatile uint32_t led_timer_counter = 0; // LED计时器计数器
//static volatile uint8_t led_on_flag = 0;      // LED亮标志

//// ********** 外部中断回调函数（重写HAL弱函数，复用CubeMX的中断配置）**********
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//  if(GPIO_Pin == KEY_Pin)  // 仅处理按键中断，不影响其他外设
//  {
//    key_press_flag = 1;  // 只标记，不做耗时操作

//  }
//}

//// ********** 定时器中断回调（重写HAL弱函数，复用CubeMX的TIM3配置）**********
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  if(htim->Instance == TIM3)  // 匹配你配置的消抖定时器
//  {
//    if(key_press_flag == 1)     // 有按键中断触发时才处理
//    {
//      // 连续读取按键电平，确认是否真的按下（消抖核心）
//      if(HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
//      {

//        HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_RESET); // 红灯亮
//        led_on_flag = 1;
//        led_timer_counter = 200;  // 设置为1秒（TIM3是5ms中断一次）
////      printf("hello");
//        key_press_flag = 0;  // 清除中断标志，等待下一次触发
//      }
////      //灯随动
////      else
////      {
////        HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);   // 红灯灭（可选）
////        led_on_flag = 0;  
////      }

//    }

//    // 1秒后灭
//    if (led_timer_counter > 0)
//    {
//        led_timer_counter--;
//    }
//    else
//    {
//            // 1秒时间到，熄灭LED
//        HAL_GPIO_WritePin(LED_Red_GPIO_Port, LED_Red_Pin, GPIO_PIN_SET);  // 红灯灭
//        led_on_flag = 0;
////              HAL_TIM_Base_Stop_IT(htim);  // 停止计时器
////            key_press_flag = 0;  // 清除中断标志，等待下一次触发
//        }
//    }

//}
