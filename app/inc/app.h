#ifndef __APP_H
#define __APP_H

#include "main.h"
#include "cmsis_os2.h"
#include <stdio.h>

/* 调试文件 */
#ifdef DEBUG
#define debug(format, ...) \
        printf("[debug] "format"\n", ##__VA_ARGS__)
#else
#define debug(format, ...)
#endif

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim7;

extern osSemaphoreId_t uartBinarySemHandle;

void UartCallback(void);

#endif