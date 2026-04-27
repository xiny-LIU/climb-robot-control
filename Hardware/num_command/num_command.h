#ifndef NUM_COMMAND_H
#define NUM_COMMAND_H
#include "stm32f4xx_hal.h"
#include "user_usart.h"
#include "string.h"
#include <stdio.h>

extern uint32_t num_control;
void USART6_ProcessCommand(void);

#endif
