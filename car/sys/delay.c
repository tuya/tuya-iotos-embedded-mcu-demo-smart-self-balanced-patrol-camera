#include "MY_ST_config.h"
#include "delay.h"
 	 
static uint8_t  fac_us=0;							//us延时倍乘数			   
static uint16_t fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数
	
	
void delay_init(uint8_t SYSCLK)
{
 	SysTick->CTRL&=~(1<<2);					//SYSTICK使用外部时钟源	 
	fac_us=SYSCLK/8;						//不论是否使用OS,fac_us都需要使用
	fac_ms=(uint16_t)fac_us*1000;				//非OS下,代表每个ms需要的systick时钟数   

}								    
void delay_us(uint32_t nus)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=nus*fac_us; 				//时间加载	  		 
	SysTick->VAL=0x00;        				//清空计数器
	SysTick->CTRL=0x01 ;      				//开始倒数 	 
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
	SysTick->CTRL=0x00;      	 			//关闭计数器
	SysTick->VAL =0X00;       				//清空计数器	 
}

void delay_ms(uint16_t nms)
{	 		  	  
	uint32_t temp;
	SysTick->LOAD=(uint32_t)nms*fac_ms;			//时间加载(SysTick->LOAD为24bit)
	SysTick->VAL =0x00;           			//清空计数器
	SysTick->CTRL=0x01 ;          			//开始倒数  
	do
	{
		temp=SysTick->CTRL;
	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达   
	SysTick->CTRL=0x00;      	 			//关闭计数器
	SysTick->VAL =0X00;       				//清空计数器	  	    
} 
 







