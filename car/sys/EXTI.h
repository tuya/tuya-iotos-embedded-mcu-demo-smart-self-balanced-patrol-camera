#ifndef __EXTI_H
#define __EXTI_H 		
#include "MY_ST_config.h"






//	 	PC10
#define PC10_OUT {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<20);GPIOC->MODER|=1<<20;} 
#define PC10_IN {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<20);GPIOC->MODER|=0<<20;GPIOC->PUPDR|=1<<20;} 
#define PC10_SET GPIOC->ODR|=1<<10
#define PC10_RESET  GPIOC->ODR&=~(1<<10)
#define PC10_TOG GPIOC->ODR^=1<<10

//	 	PC11
#define PC11_OUT {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<22);GPIOC->MODER|=1<<22;} 
#define PC11_IN {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<22);GPIOC->MODER|=0<<22;GPIOC->PUPDR|=1<<22;} 
#define PC11_SET GPIOC->ODR|=1<<11
#define PC11_RESET  GPIOC->ODR&=~(1<<11)
#define PC11_TOG GPIOC->ODR^=1<<11


//	 	PB14
#define PB14_OUT {RCC->IOPENR|=1<<1;GPIOB->MODER&=~(3<<28);GPIOB->MODER|=1<<28;} 
#define PB14_IN {RCC->IOPENR|=1<<1;GPIOB->MODER&=~(3<<28);GPIOB->MODER|=0<<28;GPIOB->PUPDR|=1<<28;} 
#define PB14_SET GPIOB->ODR|=1<<14
#define PB14_RESET  GPIOB->ODR&=~(1<<14)
#define PB14_TOG GPIOB->ODR^=1<<14

//	 	PB15
#define PB15_OUT {RCC->IOPENR|=1<<1;GPIOB->MODER&=~((uint32_t)3<<30);GPIOB->MODER|=(uint32_t)1<<30;} 
#define PB15_IN {RCC->IOPENR|=1<<1;GPIOB->MODER&=~((uint32_t)3<<30);GPIOB->MODER|=(uint32_t)0<<30;GPIOB->PUPDR|=(uint32_t)1<<30;} 
#define PB15_SET GPIOB->ODR|=1<<15
#define PB15_RESET  GPIOB->ODR&=~(1<<15)
#define PB15_TOG GPIOB->ODR^=1<<15


//	 	PC0
#define PC0_OUT {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<0);GPIOC->MODER|=1<<0;} 
#define PC0_IN {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<0);GPIOC->MODER|=0<<0;} 
#define PC0_SET GPIOC->ODR|=1<<0
#define PC0_RESET  GPIOC->ODR&=~(1<<0)
#define PC0_TOG GPIOC->ODR^=1<<0

typedef enum
{
  KEEP_STOP =0,
	GO_STRAIGHT,
  GO_BACK,
	TURN_RIGHT,
	TURN_LEFT,
	TURN_OFF
}Direc; 


typedef enum
{
  D_NULL =0,
	D_Judging,
  D_Judged,
	D_Judged_delay,
}D_State; 


typedef struct
{
  uint8_t Current;
	uint8_t Next ;
	uint8_t State;
  uint8_t count;
	uint8_t count2;
}Direct;

extern  Direct Direction;
void Direction_Init(void);
void Direction_judge(void);
void EXTI0_Init(void);
void EXTI_Key_Init(void);
#endif
