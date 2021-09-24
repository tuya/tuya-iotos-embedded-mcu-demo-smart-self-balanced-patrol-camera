#include "TIM.h"
#include "IO.h"
#include "USART.h"
#include "ADC.h"
#include "EXTI.h"

/*
TIM1 高级
null

TIM2/3 通用
编码器功能

TIM15/16/17 通用


TIM14 通用


TIM6/7 基本


LPTIM
null
*/


#if 1 //两个编码器
//编码器 PB4 PB5  
void TIM3_Encoder_Init(void)//right
{
	RCC->IOPENR |= 1<<1;
	GPIOB->MODER &=~(3<<8|3<<10);
  GPIOB->MODER |=  2<<8|2<<10; //复用功能
	GPIOB->AFR[0] &= ~(0xf<<16|0xf<<20);
	GPIOB->AFR[0] |= 1<<16|1<<20;//选择复用功能AF1
	
	RCC->APBENR1 |= 1<<1;
	TIM3->CCMR1 |= 1<<8|1<<0;//设置映射
	TIM3->CCER &= ~(1<<1|1<<3|1<<5|1<<7);//设置极性,未反相
	TIM3->SMCR |= 3<<0;//编码器模式3
	TIM3->CR1 |= 1<<0;
}
//编码器 PA0 PA1 
void TIM2_Encoder_Init(void)//left
{
	RCC->IOPENR |= 1<<0;
	GPIOA->MODER &=~(3<<0|3<<2);
  GPIOA->MODER |=  2<<0|2<<2; //复用功能
	GPIOA->AFR[0] &= ~(0xf<<0|0xf<<4);
	GPIOA->AFR[0] |= 2<<0|2<<4;//选择复用功能AF2
	
	RCC->APBENR1 |= 1<<0;
	TIM2->CCMR1 |= 1<<8|1<<0;//设置映射
	TIM2->CCER &= ~(1<<1|1<<3|1<<5|1<<7);//设置极性,未反相
	TIM2->SMCR |= 3<<0;//编码器模式3
	TIM2->CR1 |= 1<<0;
}
#define average_speed 10
#define pi 3.14
#define wheel_d 65
#define circumference (pi*wheel_d)
uint8_t F_TASK_SPEED=0;
void TASK_SPEED(void)//电机内部转一圈，一个相有13个上升沿，1：60电机外部一圈，4倍技术，内部总共产生3120的跳变沿,0XC30
{
	static uint8_t t=0,i=0;
	static uint16_t curr_c=0,curr_sum=0;
	
	t++;
	curr_c=TIM2->CNT;
	TIM2->CNT=0;
	if(curr_c>0x5000)
		curr_c=0xffff-curr_c;	
	
	if(t==2)
	{
		curr_sum=curr_sum+curr_c;
		i++;
		if(i==average_speed)
		{
			curr_sum=curr_sum*10;//200ms估算1s内的平均跳变沿
			printf("%d-%.2f %d-%.2f\r\n",curr_sum,(curr_sum*circumference/3120),curr_c*100,(curr_c*circumference/31.2));
			curr_sum=0;
			i=0;
		}
		t=0;
	}
}


int16_t Enconder_left,Enconder_right;
void READ_Encoder(void)
{
	Enconder_left=TIM2->CNT;
	TIM2->CNT=0;	
	Enconder_right=TIM3->CNT;
	TIM3->CNT=0;
  //printf("%d,%d",Enconder_left,Enconder_right);	
}
#endif
#if 1 //TASK分配

void TIM6_Init(uint16_t arr,uint16_t psc)//TASK分配
{
	RCC->APBENR1 |= 1<<4;
	TIM6->ARR = arr;//重载值
	TIM6->PSC = psc;//预分频
	TIM6->DIER |= 1<<0;//使能更新中断
	TIM6->CR1 |= 1<<0;//使能计数器
	NVIC_SetPriority(TIM6_DAC_LPTIM1_IRQn, 2); 
	NVIC_EnableIRQ(TIM6_DAC_LPTIM1_IRQn);
}
void TIM6_Start(void)
{
	TIM6->CR1 |=1<<0;//使能计数器
}
void TIM6_Stop(void)
{
	TIM6->CR1 &=~(1<<0);	//禁止计数器
	TIM6->CNT =0;//计数清零
}

uint16_t beep_off=400;
#define beep_count_max 400
extern uint8_t ipc_init_flag;
uint16_t ipc_init_count = 0;

void TIM6_DAC_LPTIM1_IRQHandler(void)
{
	static uint16_t beep_count=0;
	if ((TIM6->SR & 1) == 1) /* Check ARR match */ 
	{	
		TIM6->SR &= ~(1<<0);//清除中断标志位
    beep_count++;
		
		if(ipc_init_flag)ipc_init_count++;

		if(ipc_init_count>6500)//65秒，跳过前期摄像头校准环节
    {
		  ipc_init_count = 0;
		  ipc_init_flag = 0;
		}
		if(beep_count<beep_off)
		{
			BEEP_RESET;
		}
		else
		{
			BEEP_SET;
		}
		if(beep_count>=beep_count_max)
		{
			beep_count=0;
			F_TASK_ADC=1;//4s查询一次
		}		
	}
}

#endif


#if 1 //步进电机信号识别
void TIM7_Init(uint16_t arr,uint16_t psc)
{
	RCC->APBENR1 |= 1<<5;
	TIM7->ARR = arr;
	TIM7->PSC = psc;
	TIM7->DIER |= 1<<0;//使能更新中断
	TIM7->CR1 |= 1<<0;//使能计数器
	NVIC_SetPriority(TIM7_LPTIM2_IRQn, 2); 
	NVIC_EnableIRQ(TIM7_LPTIM2_IRQn);
}
void TIM7_Start(void)
{
	TIM7->CR1 |=1<<0;//使能计数器
	TIM7->CNT =0;
}
void TIM7_Stop(void)
{
	TIM7->CR1 &=~(1<<0);	//禁止计数器
	TIM7->CNT =0;//计数清零
}

void TIM7_LPTIM2_IRQHandler(void)
{
	static uint16_t delay_count=0;

	if ((TIM7->SR & 1) == 1) /* Check ARR match */ 
	{	
		TIM7->SR &= ~(1<<0);//清除中断标志位
		if(Direction.State==D_Judging)
		{
			Direction.count++;
			if(Direction.count>10)
			{
				TIM7_Stop();
				Direction.count=0;
			}
		}
		else if(Direction.State==D_Judged_delay)//判断结束后，需间隔一段时间再开始下次判断
		{
			delay_count++;
			if(delay_count>150)//150*2ms
			{
				delay_count=0;
				Direction.State=D_Judging;
			}
		}
	}
}
#endif
#if 1
void TIM14_Init(uint16_t arr,uint16_t psc)//MAIN串口超时
{
	RCC->APBENR2 |= 1<<15;
	TIM14->ARR = arr;
	TIM14->PSC = psc;
	TIM14->DIER |= 1<<0;//使能更新中断
	TIM14->CR1 |= 1<<0;//使能计数器
	NVIC_SetPriority(TIM14_IRQn, 1); 
	NVIC_EnableIRQ(TIM14_IRQn);
}
void TIM14_Start(void)
{
	TIM14->CR1 |=1<<0;//使能计数器
}
void TIM14_Stop(void)
{
	TIM14->CR1 &=~(1<<0);	//禁止计数器
	TIM14->CNT =0;//计数清零
}
void TIM14_Continue(void)
{
	TIM14->CR1 |=1<<0;//使能计数器
	TIM14->CNT =0;//计数清零
}
void TIM14_IRQHandler(void)
{
	if ((TIM14->SR & 1) == 1) /* Check ARR match */ 
	{	
		TIM14->SR &= ~(1<<0);//清除中断标志位
		TIM14_Stop();
		if(F_PGRESS_USART_MAIN==1)
		{
			rear_inc(&Q_Main);
			F_TASK_USART_MAIN++;
			F_PGRESS_USART_MAIN=0;
		}		
	}
}
#endif
#if 1 //两个电机转速控制 TIM16 TIM17 --PWM  PD0 PD1  
void ConfigureTIM16_CH1_AsPWM_EdgeAligned(void)//T16_CH1 PWM PD0 AF2 舵机   right
{
  RCC->IOPENR |= 1<<3;
	GPIOD->MODER &=~(3<<0);
  GPIOD->MODER |=  2<<0; //复用功能
	GPIOD->AFR[0]&=0XFFFFFFF0;
  GPIOD->AFR[0] |= 0x02 << 0; 

  RCC->APBENR2 |= 1<<17;  
  TIM16->PSC = 1; //16M/(15+1)=1M
  TIM16->ARR = 7999;//1*（999+1）=1Ms 总周期
  TIM16->CCR1 = 0; // 高电平时间
  TIM16->CCMR1 |=6<<4|1<<3;//CH_1 110模式,预装载使能（默认为输出模式）
  TIM16->CCER |= 1<<0;  //CH_1开启输出PWM
  TIM16->CR1 |= 1<<0; //使能计数器(边沿对齐 递增)
  TIM16->EGR |= 1<<0; //更新中断
	TIM16->BDTR |=1<<14;//自动输出使能AOE
}
void TIM16_PWM_Stop(void)
{
	TIM16->CCER &=~(1<<0);//关闭输出PWM
	TIM16->CR1 &=~(1<<0);	//禁止计数器
	TIM16->CNT =0;//计数清零
}
void TIM16_PWM_Start(void)
{
	TIM16->CCER |=1<<0;//开启输出PWM
	TIM16->CR1 |=1<<0;//使能计数器
}
void TIM16_PWM_Set(uint16_t CCR)
{
	TIM16->CCR1 = CCR;
}
void ConfigureTIM17_CH1_AsPWM_EdgeAligned(void)//T17_CH1 PWM PD1 AF2  left
{
  RCC->IOPENR |= 1<<3;
	GPIOD->MODER &=~(3<<2);
  GPIOD->MODER |=  2<<2; //复用功能
	GPIOD->AFR[0]&=0XFFFFFF0F;
  GPIOD->AFR[0] |= 0x02 << 4;
  
	RCC->APBENR2 |= 1<<18;
  TIM17->PSC = 1; //16M/(15+1)=1M, step=1us
  TIM17->ARR = 7999;//1*（999+1）=1Ms 总周期 
  TIM17->CCR1 = 0; // 高电平时间
  TIM17->CCMR1 |=6<<4|1<<3;//CH_1 110模式,预装载使能（默认为输出模式）
  TIM17->CCER |= 1<<0;  //CH_1开启输出PWM
  TIM17->CR1 |= 1<<0; //使能计数器(边沿对齐 递增)
  TIM17->EGR |= 1<<0; //更新中断
	TIM17->BDTR |=1<<14;//自动输出使能AOE
}
void TIM17_PWM_Stop(void)
{
	TIM17->CCER &=~(1<<0);//关闭输出PWM
	TIM17->CR1 &=~(1<<0);	//禁止计数器
	TIM17->CNT =0;//计数清零
}
void TIM17_PWM_Start(void)
{
	TIM17->CCER |=1<<0;//开启输出PWM
	TIM17->CR1 |=1<<0;//使能计数器
}
void TIM17_PWM_Set(uint16_t CCR)
{
	TIM17->CCR1 = CCR;
}
#endif

void TIM_Init(void)
{	
	TIM3_Encoder_Init();//right
	TIM2_Encoder_Init();//left
	TIM6_Init(10,15999);//1000=1s,10为10ms用于TASK_TIM6
  TIM7_Init(200,159);//1000=10ms，200为2ms，步进电机信号识别
	TIM14_Init(10,15999);//串口10ms接受超时
	ConfigureTIM16_CH1_AsPWM_EdgeAligned();//right
	ConfigureTIM17_CH1_AsPWM_EdgeAligned();//left
}

