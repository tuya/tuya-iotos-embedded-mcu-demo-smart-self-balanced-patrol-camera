#include "USART.h"
#include "TIM.h"
#include "IO.h"

void creat_sq(Queue* SQ)//设置队列参数
{
	SQ->front=0;
	SQ->rear=0;
	SQ->size=quene_main_buf_total;
}
void front_inc(Queue* SQ)//队首指针
{
	SQ->front++;
	if(SQ->front==SQ->size)
	{
		SQ->front=0;
	}
}
void rear_inc(Queue* SQ)//队尾指针
{
	SQ->rear++;
	if(SQ->rear==SQ->size)
	{
		SQ->rear=0;
	}
}

Queue Q_Main;
struct quene_buf_type1 q_mian_buf[quene_main_buf_total];
uint8_t USART_MAIN_BUF[USART_MAIN_BUF_LEN];
uint8_t F_TASK_USART_MAIN=0;
uint8_t F_PGRESS_USART_MAIN=0;
void Configure_USART_MAIN(uint32_t bound) //TX PC4, RX PC5 USART1
{
	RCC->APBRSTR2 &=~(1<<14);//恢复串口1
	RCC->IOPENR |= 1<<2;//使能GPIOC时钟
	GPIOC->MODER &=~(3<<8|3<<10);
	GPIOC->MODER |=2<<8|2<<10;//复用模式
	GPIOC->AFR[0] &=~(0xf<<16|0xf<<20);
	GPIOC->AFR[0] |=1<<16|1<<20;//选择复用功能AF1
	RCC->APBENR2 |=1<<14;//使能串口1时钟	
	USART_MAIN->BRR = 16000000 / bound; 
	USART_MAIN->CR1 |= 1<<0|1<<2|1<<3|1<<5;//串口使能，使能接收，使能发送,
	while((USART_MAIN->ISR & 1<<6) != 1<<6)//发送完成标志位
	{ 
		break;/* add time out here for a robust application */
	}	
	NVIC_SetPriority(USART1_IRQn, 2);
	NVIC_EnableIRQ(USART1_IRQn);
	creat_sq(&Q_Main);
	F_TASK_USART_MAIN=0;//清除定时器初始化时的误触发 必须清除！！！因为接收超时定时器开启时已经误触发一次
}
void USART1_IRQHandler(void)
{
	if((USART_MAIN->ISR & 1<<5) == 1<<5)//接收寄存器数据不为空
	{
		q_mian_buf[Q_Main.rear].data[q_mian_buf[Q_Main.rear].length]=(uint8_t)(USART_MAIN->RDR);
		q_mian_buf[Q_Main.rear].length++;
		TIM14_Continue();	
		F_PGRESS_USART_MAIN=1;
	}
	else
	{
		//NVIC_DisableIRQ(USART1_IRQn); /* Disable USART1_IRQn */
	}	
	if((USART_MAIN->ISR & (1<<3)) == (1<<3))//ORE
	{
		USART_MAIN->ICR =1<<3;
	}
}
uint8_t USART_Send(USART_TypeDef * MY_usart,uint8_t *data,uint16_t len)
{
	uint8_t ret=1;
	uint16_t timeout=0x8000;
	while(len>0)
	{
		timeout=0x8000;
		MY_usart->TDR = *data;
		while((MY_usart->ISR&1<<6)!=1<<6)//等待发送完毕
		{
			timeout--;
			if( 0 == timeout )
			{
				ret = 1;
				break;
			}
		}
		data++;
		len--;
	}
	if( 0 != timeout )
	{
		ret = 0;
	}
	return ret;
}

void TASK_USART_MAIN(void)
{
  //printf("\r\n%d: ",Q_Main.front);//查看隊列
	//USART_Send(USART_MAIN,q_mian_buf[Q_Main.front].data,q_mian_buf[Q_Main.front].length);
	switch (q_mian_buf[Q_Main.front].data[0])
	{
		case 0x31:
		Balance_Kp=Balance_Kp+1;
		printf("%0.2f",Balance_Kp);
		break;
		case 0x32:
		Balance_Kp=Balance_Kp-1;
		printf("%0.2f",Balance_Kp);
		break;
		case 0x33:
		Balance_Kd=Balance_Kd+0.02;
		printf("%0.2f",Balance_Kd);
		break;
		case 0x34:
		Balance_Kd=Balance_Kd-0.02;
		printf("%0.2f",Balance_Kd);
		break;
		case '?':
		printf("%0.2f %0.2f\r\n",Balance_Kp,Balance_Kd);
		break;		
	}
	memset(q_mian_buf[Q_Main.front].data,0,q_mian_buf[Q_Main.front].length);
	q_mian_buf[Q_Main.front].length=0;
	front_inc(&Q_Main);	
}

#if 1
#pragma import(__use_no_semihosting)  
//解决HAL库使用时,某些情况可能报错的bug
int _ttywrch(int ch)    
{
    ch=ch;
	return ch;
}
//标准库需要的支持函数       

struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 

/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
#define USART_fput USART_MAIN
int fputc(int ch, FILE *f)
{      
	while((USART_fput->ISR&0X40)==0);//循环发送,直到发送完毕   
	USART_fput->TDR = (uint8_t) ch;          	
	return ch;
}
#endif
