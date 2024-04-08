#include "led.h"

#define LD_GPIO     GPIOC

#define LED_SET     (uint16_t)0xff00

static uint16_t led_status = 0xff00;

void LedInit(void){
    LD_GPIO->ODR = LED_SET;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

/**
 * 控制LED引脚电平
 * @param id LED id
 * @param pin_state 电平状态
 */
static void LedWrite_Drv(Led_Id id, GPIO_PinState pin_state) {
    if (pin_state) {  // 1
        led_status |= (pin_state & 0x01) << (id + 8);
        debug("led_status:0x%x", led_status);
    } else {
        led_status &= ~((pin_state | 0x01) << (id + 8));
        debug("led_status:0x%x", led_status);
    }
    LD_GPIO->ODR = led_status;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}

void LedWirte(Led_Id id, GPIO_PinState pin_state){
	if (pin_state) {  // 1
        led_status |= (pin_state & 0x01) << (id + 8);
        debug("led_status:0x%x", led_status);
    } else {
        led_status &= ~((pin_state | 0x01) << (id + 8));
        debug("led_status:0x%x", led_status);
    }
    LD_GPIO->ODR = led_status;
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);
}
