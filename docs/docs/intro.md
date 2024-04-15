---
sidebar_label: '快速开始'
sidebar_position: 1
---

# 蓝桥杯备赛

板载资源

| 资源     | 配备 | 资源     | 配备 |
|--------|----|--------|----|
| 串口     | 1  | E2PROM | 1  |
| LCD    | 1  | 可编程电阻  | 1  |
| 按键     | 4  | 信号发生器  | 2  |
| LED    | 8  | 分压电位器  | 2  |
| E2PROM | 1  |        |    |

:::tip[基本约定]

- 使用`TIM7`作为硬件定时器，中断间隔为`100ms`
- 使用`TIM16`作为FreeRTOS的时基定时器

:::

由于比赛限制，无法重写硬件定时器，对硬件定时器进行代码层面规范

![image-20240329210804533](figures/image-20240329210804533.png)

![image-20240329210824590](figures/image-20240329210824590.png)

```c
static uint8_t <name>count = 0;

// 回调应用函数
void UartCallback(void) {
    <name>count++;
    if (<name>count >= <计时数>) {
        <name>count = 0;
        // 应用代码
    }
}
```

## 基础配置

### 日志配置

由于比赛的限制，本项目的日志设计的非常简单，只包含`debug`和`error`标签，并且，使用printf进行二次封装。

```c
/* if define DEBUG,open debug and error */
#ifdef DEBUG
#define debug(format, ...) \
        printf("[debug] "format"\n", ##__VA_ARGS__)
#define error(format, ...) \
        printf("[error] "format"\n", ##__VA_ARGS__)
#else
#define debug(format, ...)
#define error(format, ...)
#endif
```

### 初始化配置

初始化拆分为外设初始化和应用程序初始化

外设初始化，在调度器开始之前，用于初始化LED、LCD...
应用程序初始化，在调度器开始之后,用于

```c
// app.c
#include "app.h"
#include "FreeRTOS.h"

void Hardware_Init(void){
//    LCD_Init();
//    LCD_Clear(WHITE);
}

int app(void){
    Hardware_Init();

    //debug("sum free:%.2zu\n",xPortGetFreeHeapSize());

    return 0;
    ERROR:
    return -1;
}

#ifdef  __CC_ARM
int fputc(int ch, FILE *stream) {
    while ((USART1->ISR & 0X40) == 0);     //等待上一次串口数据发送完成
    USART1->TDR = (uint8_t) pBuffer[i];    //写DR,串口1将发送数据
    return ch;
}
#elifdef __GNUC__

int _write(int fd, char *pBuffer, int size) {
    for (int i = 0; i < size; i++) {
        while ((USART1->ISR & 0X40) == 0);  //等待上一次串口数据发送完成
        USART1->TDR = (uint8_t) pBuffer[i]; //写DR,串口1将发送数据
    }
    return size;
}

#endif
```

```c
// app.h
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

#endif
```

## 外设

### 串口

对于串口模块只需要关注对于`buf`进行应用

```c
#include "app.h"
#include <stdio.h>
#include <string.h>

static uint8_t rx_buf[20];
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
```

### LCD

改写`LCD`显示字符串

```c
// 小熊派
void LCD_StringLine(uint8_t Line, uint8_t x,uint8_t *ptr)
{
    uint32_t i = 0;
    uint16_t refcolumn = (319 * x * 16); //319

    while ((*ptr != 0) && (i < 20)) // 20
    {
        LCD_DisplayChar(Line, refcolumn, *ptr);
        refcolumn -= 16;
        ptr++;
        i++;
    }
}
```

### 按键

- 扫描按键线程
- 使用软件定时器判断

```c
#include "app.h"
#include <stdio.h>

static uint8_t isKey = 1;
uint8_t key_error = 0;

static uint8_t Key1Value = 0;
static uint8_t Key2Value = 0;

static uint8_t lcdData_key = 1;

void KeyTask(void *arg) {
    while (1) {
        if (isKey == 0) {
            debug("go sta\n");
        } else {

            if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == 0) {
                if (Key1Value == 0) {
                    osTimerStart(key1TimerHandle, 150);
                }
                while (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == 0);
                Key1Value++;
            }

            if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == 0) {
                if (Key2Value == 0) {
                    osTimerStart(key2TimerHandle, 150);
                }
                while (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == 0);
                Key2Value++;
            }
        }

        osDelay(50);
    }
}

void Key1Callback(void *arg) {
    lcdData_t *lcd_data;
    debug("key1 value:%d\n",Key1Value);
    if (Key1Value == 1) {
        LcdPsd_Data(lcdData_key);
        lcd_data = (lcdData_t *) osMemoryPoolAlloc(lcdMemoryPoolHandle, 0);
        lcd_data->flag = 3;
        osMessageQueuePut(lcdQueueHandle, &lcd_data, 0, 0);
    } else if (Key1Value == 2) {

    } else {

    }

    Key1Value = 0;
}

static uint8_t led_data = 0;
void Key2Callback(void *arg) {
    debug("key2 value:%d\n",Key2Value);
    if (Key2Value == 1) {
        lcdData_key++;
        if (lcdData_key == 4) {
            lcdData_key = 1;
        }
        debug("lcdData_key:%d\n",lcdData_key);
    } else if (Key2Value == 2) {
        if (LcdPsd_DataVerify() == osOK) {
            isKey = 0;
            led_data = 0x81;
            osMessageQueuePut(ledQueueHandle,&led_data,0,0);
            HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
            __HAL_TIM_SET_AUTORELOAD(&htim2,500-1);
            __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,50);
        }else{
            key_error ++;
            debug("key_error:%d\n",key_error);
            if(key_error >= 3){
                key_error = 0;
                led_data = 0x41;
                osMessageQueuePut(ledQueueHandle,&led_data,0,0);
            }
        }
    } else {

    }

    Key2Value = 0;
}

void LcdSwitchCallback(void *arg) {
    HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
    __HAL_TIM_SET_AUTORELOAD(&htim2,1000-1);
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,500);

    lcdData_t *lcd_data;

    osMutexAcquire(lcdDataMutexHandle,osWaitForever);
    PSD_B1 = '@';
    PSD_B2 = '@';
    PSD_B3 = '@';
    osMutexRelease(lcdDataMutexHandle);
    lcd_data = (lcdData_t *) osMemoryPoolAlloc(lcdMemoryPoolHandle, 0);
    lcd_data->flag = 1;
    osMessageQueuePut(lcdQueueHandle, &lcd_data, 0, 0);
    isKey = 1;
}
```

### LED

```c
#define LD_GPIO     GPIOC

static uint16_t led_status = 0xff00;

/**
 * LED id
 */
typedef enum {
    LD1 = 0,
    LD2,
    LD3,
    LD4,
    LD5,
    LD6,
    LD7,
    LD8
} Led_Id;

/**
 * 控制LED引脚电平
 * @param id LED id
 * @param pin_state 电平状态
 */
void LedWrite(Led_Id id, GPIO_PinState pin_state) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    if (pin_state) {  // 1
        led_status |= (pin_state & 0x01) << (id + 8);
        debug("led_status:0x%x", led_status);
    } else {
        led_status &= ~((pin_state | 0x01) << (id + 8));
        debug("led_status:0x%x", led_status);
    }
    LD_GPIO->ODR = led_status;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}
```

### E2PROM和可编程电阻(IIC)

由于蓝桥杯得板子没法使用硬件IIC，所以，只能使用软件IIC

```c
// 确保读写API一致

#define    AT24C02_ADDR_WRITE  0xA0
#define    AT24C02_ADDR_READ   0xA1

/**
 * @brief        AT24C02任意地址写一个字节数据
 * @param        addr —— 写数据的地址（0-255）
 * @param        dat  —— 存放写入数据的地址
 * @retval        成功 —— HAL_OK
*/
void At24c02_Write_Byte(uint16_t addr, uint8_t* data)
{
    I2CStart();
 I2CSendByte(AT24C02_ADDR_WRITE);
 I2CWaitAck();
 I2CSendByte(addr);
 I2CWaitAck();
 I2CSendByte(data);
 I2CWaitAck();
 I2CStop();
    HAL_Delay(5);
}

/**
 * @brief        AT24C02任意地址读一个字节数据
 * @param        addr —— 读数据的地址（0-255）
 * @param        read_buf —— 存放读取数据的地址
 * @retval        成功 —— HAL_OK
*/
uint8_t At24c02_Read_Byte(uint16_t addr, uint8_t* read_buf)
{
 I2CStart();
 I2CSendByte(0xa0);
 I2CWaitAck();
 I2CSendByte(addr);
 I2CWaitAck();
 
 I2CStart();
 I2CSendByte(0xa1);
 I2CWaitAck();
 *read_buf = I2CReceiveByte();
 I2CWaitAck();
 I2CStop();
}
```

## 编程方式

### 异步编程

> [!note]
>
> 备注
