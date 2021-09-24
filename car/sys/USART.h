#ifndef __USART_H
#define __USART_H 		
#include "MY_ST_config.h"
#include "stdio.h"
#include "string.h"



//采样队列方式 超时接收
#define USART_MAIN  USART1
#define USART_MAIN_BUF_LEN 40
#define USART_MAIN_BOUND 921600



typedef struct {
	uint8_t front;//队首指针
	uint8_t rear;//队尾指针
	uint8_t size;//队列容量
}Queue;

void front_inc(Queue* SQ);
void rear_inc(Queue* SQ);



struct quene_buf_type1
{
	uint8_t tab;
	uint16_t length;
	uint8_t data[USART_MAIN_BUF_LEN];
};	
#define quene_main_buf_total 4
extern struct quene_buf_type1 q_mian_buf[quene_main_buf_total];
extern Queue Q_Main;
extern uint8_t F_TASK_USART_MAIN;
extern uint8_t F_PGRESS_USART_MAIN;
void Configure_USART_MAIN(uint32_t bound);//TX PC4, RX PC5 USART1
void TASK_USART_MAIN(void);



uint8_t USART_Send(USART_TypeDef * MY_usart,uint8_t *data,uint16_t len);

#endif

