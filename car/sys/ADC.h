#ifndef __ADC_H
#define __ADC_H 		
#include "MY_ST_config.h"




#define ADC_PA7 7
#define ADC_PA7_IN  {RCC->IOPENR|=1<<0;GPIOA->MODER&=~(3<<14);GPIOA->MODER|=0<<14;}







extern float BAT_VOL;
void ADC_Init(void);
uint16_t Get_adc(uint8_t ch);


extern uint8_t F_TASK_ADC;
void TASK_ADC(void);


#endif

