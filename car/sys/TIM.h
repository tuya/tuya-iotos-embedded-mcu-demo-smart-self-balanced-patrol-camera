#ifndef __TIM_H
#define __TIM_H 		
#include "MY_ST_config.h"


extern int16_t Enconder_left,Enconder_right;
void READ_Encoder(void);
	
void TIM6_Start(void);	
void TIM6_Stop(void);


void TIM14_Start(void);
void TIM14_Stop(void);
void TIM14_Continue(void);

void TIM16_PWM_Set(uint16_t CCR);
void TIM17_PWM_Set(uint16_t CCR);
void TIM2_Start(void);
void TIM2_Stop(void);
void TIM2_Continue(void);

extern uint8_t F_TASK_SPEED;
void TASK_SPEED(void);

void TIM7_Start(void);
void TIM7_Stop(void);

extern uint8_t F_TASK_TIM6;
extern uint16_t beep_off;
void TASK_TIM6(void);



void TIM_Init(void);

#endif

