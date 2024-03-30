#include "app.h"
#include "FreeRTOS.h"

#ifdef TEST

const osThreadAttr_t testAttr = {
        .name = "test",
        .priority = (osPriority_t) osPriorityHigh,
        .stack_size = 512 * 4
};

extern void testTask(void *arg);

#endif

void Hardware_Init(void) {
//    LCD_Init();
//    LCD_Clear(WHITE);
}

int app(void) {
    Hardware_Init();

    debug("sum free:%d",xPortGetFreeHeapSize());

    if(xPortGetFreeHeapSize() == 0){
         goto ERROR;
    }


#ifdef TEST
    osThreadNew(testTask, NULL,&testAttr);
#endif

    return 0;
    ERROR:
    return -1;
}


#if defined (__CC_ARM)
int fputc(int ch, FILE *stream) {
    while ((USART1->ISR & 0X40) == 0);      //等待上一次串口数据发送完成
    USART1->TDR = (uint8_t) pBuffer[i];     //写DR,串口1将发送数据
    return ch;
}
#elif defined(__GNUC__)

int _write(int fd, char *pBuffer, int size) {
    for (int i = 0; i < size; i++) {
        while ((USART1->ISR & 0X40) == 0);  //等待上一次串口数据发送完成
        USART1->TDR = (uint8_t) pBuffer[i]; //写DR,串口1将发送数据
    }
    return size;
}

#endif


