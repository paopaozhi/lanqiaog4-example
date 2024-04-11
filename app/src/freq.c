#include "app.h"

typedef struct
{
  HAL_TIM_StateTypeDef state;
  uint32_t ccr;
  uint32_t freq;
}freqData_t;

freqData_t tim2Ch1;
freqData_t tim3Ch1;

uint32_t ch1_cnt = 0;
uint32_t ch1_freq = 0;

void FreqTask(void *arg) {
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_IC_Start(&htim2, TIM_CHANNEL_1);

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);
  while (1) {
    tim2Ch1.ccr = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
    tim2Ch1.freq = 1000000 / tim2Ch1.ccr;
    tim2Ch1.state = HAL_TIM_IC_GetState(&htim2);

    tim3Ch1.ccr = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
    tim3Ch1.freq = 1000000 / tim3Ch1.ccr;
    tim3Ch1.state = HAL_TIM_IC_GetState(&htim3);

    /* app start */
    debug("tim2Cn1_state ret:%d",tim2Ch1.state);
    debug("tim2Cn1_freq:%d hz", tim2Ch1.freq);

    debug("tim3Cn1_state ret:%d",tim3Ch1.state);
    debug("tim3Cn1_freq:%d hz", tim3Ch1.freq);
    /* app end */
    osDelay(500);
  }
}
