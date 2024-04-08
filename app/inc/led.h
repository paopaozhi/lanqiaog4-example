//
// Created by paopa on 2024/4/8.
//

#ifndef __LED_H
#define __LED_H

#include "app.h"

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

void LedInit(void);
void LedWirte(Led_Id id, GPIO_PinState pin_state);

#endif //__LED_H
