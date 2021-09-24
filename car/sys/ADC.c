#include "ADC.h"
#include "delay.h"
#include "USART.h"
#include "TIM.h"
void ADC_IO_Init(void)
{
	ADC_PA7_IN;
}
void ADC_Init(void)
{
	ADC_IO_Init();
	RCC->APBENR2 |= 1<<20 ;//使能ADC时钟,使能系统配置控制器时钟 | 1<<0
	ADC1->CR &= ~(1<<0);//关闭ADC
  ADC1->CFGR1 |= 1<<15; //使能自动关闭模式，可省功耗
	ADC1->CFGR1 &= ~(3<<3);
	ADC1->CFGR1 |= 1<<3;//10位采样精度   //注意ch18,校正值为12位精度
	ADC1->CFGR1 &=~(1<<13);//单次转换模式
	ADC1->CFGR1 |= 1<<16;//不连续模式
  ADC1->SMPR |= 7<<0;//设置采样时间为160.5
/*************************************************/		
	ADC1->CR |= (1<<28);//ADC调压器使能
	delay_us(30);//手册20us
	ADC1->CR |= (uint32_t)1<<31;//校准ADC
  while ((ADC1->ISR & 1<<11) == 0) //等待校准完成
  {
    //time-out
  }
	ADC1->ISR |= 1<<11; //清除校准标志位
/*************************************************/		
	ADC1->CR |= 1<<0;//使能ADC 
  if ((ADC1->CFGR1 &  1<<15) == 1)
  {
    while ((ADC1->ISR & 1<<0) == 0) //等待ADC_Ready
    {
      /* For robust implementation, add here time-out management CA2.4 L0798 */
    }
  }
}

uint16_t Get_adc(uint8_t ch)   
{
	uint16_t timeout=0x1000;
	ADC1->CHSELR=(uint32_t)1<<ch;
  ADC1->CR |= 1<<2;            //开启ADC
  while(!(ADC1->ISR&1<<2))   //等待转换结束	
	{
		timeout--;
		if( 0 == timeout )
		{
			return 0;
		}
	}
	return ADC1->DR;
}


uint8_t F_TASK_ADC=0;
float BAT_VOL;
void TASK_ADC(void)
{
	BAT_VOL=Get_adc(ADC_PA7)*330*5.7/1023;
	//printf("POW=%.0f",BAT_VOL);
	if(BAT_VOL<1080)
	{
		beep_off=300;
	}
	else
	{
		beep_off=400;
	}
}


