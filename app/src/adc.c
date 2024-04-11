#include "app.h"

double adc1_v = 0;
double adc2_v = 0;

void AdcTask(void *arg){
    uint32_t adc1_value = 0;
    uint32_t adc2_value = 0;



    HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);
    while (1){
        HAL_ADC_Start(&hadc1);
        adc1_value = HAL_ADC_GetValue(&hadc1);

        HAL_ADC_Start(&hadc2);
        adc2_value = HAL_ADC_GetValue(&hadc2);

        adc1_v = 3.3 * (adc1_value / 4096.0);
        adc2_v = 3.3 * (adc2_value / 4096.0);
//        printf("adc1_v: %d.%d\n",adc1_v,adc1_value*100);
        //debug("adc1_value: %ld",adc1_value);
        //debug("adc1_v: %.2lf",adc1_v);
        //debug("adc2_value: %ld",adc2_value);
        //debug("adc1_v: %.2lf",adc1_v);

        osDelay(1000);
    }
}
