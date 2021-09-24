#include "sys.h"
#include "RCC.h"
#include "delay.h"
#include "IO.h"
#include "TIM.h"
#include "USART.h"
#include "ADC.h"
#include "EXTI.h"

uint8_t ipc_init_flag = 1;
uint8_t F_TASK_FLAG = 0;
void System_Init(void)
{
	
	SystemClock_Config();//HSI16/4 -> PLL*8/2->16M	
	delay_init(16);
	TIM_Init();
	Configure_USART_MAIN(USART_MAIN_BOUND);
	ADC_Init();
	IO_Init();
}


void System_Task(void)
{
	if(Direction.State==D_Judged)
	{
		Direction.State=D_Judged_delay;
		Direction_judge();
	}
	if(F_TASK_USART_MAIN)
	{
		F_TASK_USART_MAIN--;
		TASK_USART_MAIN();
	}
	
	if(F_TASK_MPU6050)
	{
		F_TASK_MPU6050=0;
		TASK_MPU6050();	
	}	
	if(F_TASK_ADC)
	{
		F_TASK_ADC=0;
		TASK_ADC();
	}	
}

