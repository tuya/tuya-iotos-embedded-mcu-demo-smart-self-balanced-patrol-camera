#include "EXTI.h"
#include "IO.h"
#include "delay.h"
#include "TIM.h"
#include "USART.h"
extern uint8_t ipc_init_flag;


//����ת��1234�� ���࣬4*2.5ms  ������10ms
void Direction_Init(void)
{
	PC10_IN;
	PC11_IN;
	PB14_IN;
	PB15_IN;
	
	Direction.count=0;
	Direction.count2=0;
	Direction.Next=KEEP_STOP;
	Direction.Current=KEEP_STOP;
	Direction.State=D_Judged_delay;
	
	EXTI->EXTICR[2] &= ~(0xf<<16 | 0xf<<24);
	EXTI->EXTICR[2] |= 2<<16| 2<<24 ;//����PC10 PC11Ϊ�ж�
  EXTI->IMR1 |= 1<<10|1<<11;//��������Line10 11���ж�
	EXTI->FTSR1 |= 1<<10|1<<11;//����Line10 11�½����ж�

	EXTI->EXTICR[3] &= ~(0xf<<16 | 0xf<<24);
	EXTI->EXTICR[3] |= 1<<16| 1<<24 ;//����PB14 PB15Ϊ�ж�
  EXTI->IMR1 |= 1<<14|1<<15;//��������Line14 15���ж�
	EXTI->FTSR1 |= 1<<14|1<<15;//����Line14 15�½����ж�
	
  NVIC_SetPriority(EXTI4_15_IRQn, 2);
  NVIC_EnableIRQ(EXTI4_15_IRQn);
}

Direct Direction;
void EXTI4_15_IRQHandler(void)
{
	if((EXTI->FPR1 & 1<<10) == 1<<10)//�������½��ش�������
  {
    EXTI->FPR1 = 1<<10;// Clear EXTI  flag
		if(ipc_init_flag)return;
		if(Direction.State==D_Judging)
		{
			TIM7_Start();
			Direction.count=0;
		}
  }
  if((EXTI->FPR1 & 1<<11) == 1<<11)//�������½��ش�������
  {
    static uint8_t Dright=0;
		static uint8_t Dleft=0;
		EXTI->FPR1 = 1<<11;// Clear EXTI  flag
		if(ipc_init_flag)return;
		if(Direction.count==1)
		{
			Dright++;
			Dleft=0;
			if(Dright>1)
			{
				if(Direction.State==D_Judging)
				{
					Direction.Next=TURN_RIGHT;
					Dright=0;
					Direction.State=D_Judged;
				}
			}
		}
		else if(Direction.count==3)
		{
			Dleft++;
			Dright=0;
			if(Dleft>1)
			{
				if(Direction.State==D_Judging)
				{
					Direction.Next=TURN_LEFT;
					Dleft=0;
					Direction.State=D_Judged;
				}
			}
		}
		Direction.count=0;	
  }
	
  //����ǰ������ҵ��źţ�������ms���Ⱥ��������û��Ҫ�½�Direction.count
	if((EXTI->FPR1 & 1<<14) == 1<<14)//�������½��ش�������
  {
    EXTI->FPR1 = 1<<14;// Clear EXTI  flag
		if(ipc_init_flag)return;
		if(Direction.State==D_Judging)
		{
			TIM7_Start();
			Direction.count=0;
		}
  }
  if((EXTI->FPR1 & 1<<15) == 1<<15)//�������½��ش�������
  {
    static uint8_t Dup=0;
		static uint8_t Ddwon=0;
		EXTI->FPR1 = 1<<15;// Clear EXTI  flag
		if(ipc_init_flag)return;
		if(Direction.count==1)
		{
			Ddwon++;
			Dup=0;
			if(Ddwon>1)
			{
				if(Direction.State==D_Judging)
				{
					Direction.Next=GO_BACK;
					Ddwon=0;
					Direction.State=D_Judged;
				}
			}
		}
		else if(Direction.count>=2)
		{
			Dup++;
			Ddwon=0;
			if(Dup>1)
			{
				if(Direction.State==D_Judging)
				{
					Direction.Next=GO_STRAIGHT;
					Dup=0;
					Direction.State=D_Judged;
				}
			}
		}
		Direction.count=0;	
  }	
}
void Direction_judge(void)
{
	TIM7_Start();
	if(Direction.Current!=TURN_OFF)
	{
		if(Direction.Current==KEEP_STOP)
		{
			Direction.Current=Direction.Next;
			printf("%d",Direction.Current);
		}
		else
		{
			if(Direction.Next!=Direction.Current)
			{
				Direction.Current=KEEP_STOP;
				printf("stop");
			}
			else 
			{
				printf("%d",Direction.Current);
			}
		}
	}
}

void EXTI0_Init(void)
{
	PC0_IN;
	EXTI->EXTICR[0] &= ~(0xf<<0 );
	EXTI->EXTICR[0] |= 2<<0 ;//����PC0Ϊ�ж�
  EXTI->IMR1 |= 1<<0;//��������Line0���ж�
	EXTI->FTSR1 |= 1<<0;//����Line0�½����ж�
	
  NVIC_SetPriority(EXTI0_1_IRQn, 1);
  NVIC_EnableIRQ(EXTI0_1_IRQn);	
}
void EXTI0_1_IRQHandler(void)
{
	if((EXTI->FPR1 & 1<<0) == 1<<0)//�������½��ش�������
	{
		EXTI->FPR1 = 1<<0;
		F_TASK_MPU6050=1;
	}
}

