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
	RCC->APBENR2 |= 1<<20 ;//ʹ��ADCʱ��,ʹ��ϵͳ���ÿ�����ʱ�� | 1<<0
	ADC1->CR &= ~(1<<0);//�ر�ADC
  ADC1->CFGR1 |= 1<<15; //ʹ���Զ��ر�ģʽ����ʡ����
	ADC1->CFGR1 &= ~(3<<3);
	ADC1->CFGR1 |= 1<<3;//10λ��������   //ע��ch18,У��ֵΪ12λ����
	ADC1->CFGR1 &=~(1<<13);//����ת��ģʽ
	ADC1->CFGR1 |= 1<<16;//������ģʽ
  ADC1->SMPR |= 7<<0;//���ò���ʱ��Ϊ160.5
/*************************************************/		
	ADC1->CR |= (1<<28);//ADC��ѹ��ʹ��
	delay_us(30);//�ֲ�20us
	ADC1->CR |= (uint32_t)1<<31;//У׼ADC
  while ((ADC1->ISR & 1<<11) == 0) //�ȴ�У׼���
  {
    //time-out
  }
	ADC1->ISR |= 1<<11; //���У׼��־λ
/*************************************************/		
	ADC1->CR |= 1<<0;//ʹ��ADC 
  if ((ADC1->CFGR1 &  1<<15) == 1)
  {
    while ((ADC1->ISR & 1<<0) == 0) //�ȴ�ADC_Ready
    {
      /* For robust implementation, add here time-out management CA2.4 L0798 */
    }
  }
}

uint16_t Get_adc(uint8_t ch)   
{
	uint16_t timeout=0x1000;
	ADC1->CHSELR=(uint32_t)1<<ch;
  ADC1->CR |= 1<<2;            //����ADC
  while(!(ADC1->ISR&1<<2))   //�ȴ�ת������	
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


