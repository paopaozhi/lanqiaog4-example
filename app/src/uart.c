#include "app.h"
#include <stdio.h>
#include <string.h>

static uint8_t rx_buf[128];
static uint32_t count = 0;

void UsartTask(void *arg) {
    HAL_UART_Receive_IT(&huart1, rx_buf, 1);
    // 清除tim
    __HAL_TIM_SET_COUNTER(&htim7, 0);
    // 清除tim更新中断标志
    __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
    // 缓存区清零
    memset(rx_buf, 0, sizeof(rx_buf));
    while (1) {
        osSemaphoreAcquire(uartBinarySemHandle, osWaitForever);
        HAL_UART_Abort_IT(&huart1);
        /* 接收buf 应用代码 开始 */
        debug("rx_buf:%s",rx_buf);
        /* 接收buf 应用代码 结束 */
        HAL_UART_Receive_IT(&huart1, rx_buf, 1);
        memset(rx_buf, 0, sizeof(rx_buf));
        count = 0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        __HAL_TIM_SET_COUNTER(&htim7, 0);
        HAL_TIM_Base_Start_IT(&htim7);
        // 计数
        count++;
        HAL_UART_Receive_IT(huart, rx_buf + count, 1);
    }
}

uint8_t tim7count = 0;

void UartCallback(void) {
    tim7count++;
    if (tim7count >= 1) {
        tim7count = 0;
        HAL_TIM_Base_Stop_IT(&htim7);
        __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);
        __HAL_TIM_SET_COUNTER(&htim7, 0);
        osSemaphoreRelease(uartBinarySemHandle);
    }
}
