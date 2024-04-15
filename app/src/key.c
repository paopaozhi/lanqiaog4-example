#include "app.h"
#include <stdio.h>

static uint8_t isKey = 1;
uint8_t key_error = 0;

#define KEY1_GPIO_Port GPIOB
#define KEY1_Pin       GPIO_PIN_0

#define KEY2_GPIO_Port GPIOB
#define KEY2_Pin       GPIO_PIN_1

#define KEY3_GPIO_Port GPIOB
#define KEY3_Pin       GPIO_PIN_2

#define KEY4_GPIO_Port GPIOA
#define KEY4_Pin       GPIO_PIN_0

static uint8_t Key1Value = 0;
static uint8_t Key2Value = 0;
static uint8_t Key3Value = 0;
static uint8_t Key4Value = 0;

static uint8_t lcdData_key = 1;

static void gpio_init(void){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void KeyTask(void *arg) {
    gpio_init();
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

            if (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == 0) {
                if (Key2Value == 0) {
                    osTimerStart(key3TimerHandle, 150);
                }
                while (HAL_GPIO_ReadPin(KEY3_GPIO_Port, KEY3_Pin) == 0);
                Key2Value++;
            }

            if (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == 0) {
                if (Key2Value == 0) {
                    osTimerStart(key4TimerHandle, 150);
                }
                while (HAL_GPIO_ReadPin(KEY4_GPIO_Port, KEY4_Pin) == 0);
                Key2Value++;
            }
        }

        osDelay(50);
    }
}

void Key1Callback(void *arg) {
    debug("key1 value:%d\n",Key1Value);
    if (Key1Value == 1) {
        /*app start*/

        /*app stop*/
    } else if (Key1Value == 2) {
        /*app start*/

        /*app stop*/
    } else {

    }

    Key1Value = 0;
}

static uint8_t led_data = 0;
void Key2Callback(void *arg) {
    debug("key2 value:%d\n",Key2Value);
    if (Key2Value == 1) {
        
    } else if (Key2Value == 2) {
        
    } else {

    }

    Key2Value = 0;
}
