#include "app.h"

typedef struct {
  uint32_t freq;
  uint32_t dutyCycle;
} pwm_config_t;


#ifdef DEBUG_NN
const osThreadAttr_t pwmTestTask_attributes = {
  .name = "pwmTestTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 256 * 4
};

void PwmTest(void *arg);
#endif

/**
 * @brief 接收配置结构体，配置频率和占空比
 * 
 */
void PwmTask(void) {
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  #ifdef DEBUG_NN
    osThreadNew(PwmTest,NULL,&pwmTestTask_attributes);
  #endif

  pwm_config_t _config;
  osStatus_t ret;
  while (1) {
    ret = osMessageQueueGet(pwmQueueHandle, &_config, 0, osWaitForever);
    if (ret == osOK) {
      // config freq
      __HAL_TIM_SET_AUTORELOAD(&htim4,(uint32_t)(1000000/_config.freq));
      // config dutyCycle
      __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,__HAL_TIM_GET_AUTORELOAD(&htim4) * (_config.dutyCycle / 100));
    }
  }
}

void PwmTest(void *arg){
  osStatus_t ret;

  ret = osMessageQueuePut(pwmQueueHandle,0,0,0);
  if(ret != osOK){
    debug("test pwm error!");
  }

  osThreadExit();
}
