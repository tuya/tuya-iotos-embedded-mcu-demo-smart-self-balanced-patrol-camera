#ifndef __IO_H
#define __IO_H 		
#include "MY_ST_config.h"
#include "math.h"
#include "stdbool.h"


//IIC_SDA	  PC12
#define IIC_SDA_OUT {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<24);GPIOC->MODER|=1<<24;GPIOC->PUPDR|=1<<24;} 
#define IIC_SDA_IN  {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<24);GPIOC->MODER|=0<<24;} 
#define IIC_SDA_SET GPIOC->ODR|=1<<12
#define IIC_SDA_RESET  GPIOC->ODR&=~(1<<12)
#define IIC_SDA_State ((GPIOC->IDR & 1<<12) == 1<<12)

//IIC_SCL	  PC13
#define IIC_SCL_OUT {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<26);GPIOC->MODER|=1<<26;GPIOC->PUPDR|=1<<26;}  
#define IIC_SCL_IN  {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<26);GPIOC->MODER|=0<<26;} 
#define IIC_SCL_SET GPIOC->ODR|=1<<13
#define IIC_SCL_RESET  GPIOC->ODR&=~(1<<13)
#define IIC_SCL_State ((GPIOC->IDR & 1<<13) == 1<<13)

#if 1
void IIC_Init(void);
void IIC_Start(void);//产生IIC起始信号
void IIC_Stop(void);//产生IIC停止信号
void IIC_Ack(void);//产生ACK应答
void IIC_NAck(void);//不产生ACK应答	
uint8_t IIC_Wait_Ack(void);//等待应答信号到来:1,接收应答失败;0,接收应答成功
void IIC_Send_Byte(uint8_t txd);//IIC发送一个字节; 先发送高位
uint8_t IIC_Read_Byte(unsigned char ack);//读一个字节，可加是否应答位,1加ack，0不加ack 从高位开始读
uint8_t IIC_Write_Byte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t data);//直接写一个字节
uint8_t IIC_ReadMulByte(uint8_t DrvAddr,uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead);//读字节
uint8_t IIC_WriteMulByte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite);//可一次写多个字节

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data);
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data);
#endif



//PA5
#define KEY_IN  {RCC->IOPENR|=1<<0;GPIOA->MODER&=~(3<<10);GPIOA->MODER|=0<<10;} 
#define KEY_State ((GPIOA->IDR & 1<<5) == 1<<5)


//BEEP PC3
#define BEEP_OUT {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<6);GPIOC->MODER|=1<<6;} 
#define BEEP_IN  {RCC->IOPENR|=1<<2;GPIOC->MODER&=~(3<<6);GPIOC->MODER|=0<<6;} 
#define BEEP_SET  GPIOC->ODR|=1<<3
#define BEEP_RESET  GPIOC->ODR&=~(1<<3)
#define BEEP_State ((GPIOC->IDR & 1<<3) == 1<<3)

//AIN2 PD2
#define AIN2_OUT {RCC->IOPENR|=1<<3;GPIOD->MODER&=~(3<<4);GPIOD->MODER|=1<<4;} 
#define AIN2_IN  {RCC->IOPENR|=1<<3;GPIOD->MODER&=~(3<<4);GPIOD->MODER|=0<<4;} 
#define AIN2_SET GPIOD->ODR|=1<<2
#define AIN2_RESET  GPIOD->ODR&=~(1<<2)

//AIN1 PD3
#define AIN1_OUT {RCC->IOPENR|=1<<3;GPIOD->MODER&=~(3<<6);GPIOD->MODER|=1<<6;} 
#define AIN1_IN  {RCC->IOPENR|=1<<3;GPIOD->MODER&=~(3<<6);GPIOD->MODER|=0<<6;} 
#define AIN1_SET GPIOD->ODR|=1<<3
#define AIN1_RESET  GPIOD->ODR&=~(1<<3)

//BIN1 PD4
#define BIN1_OUT {RCC->IOPENR|=1<<3;GPIOD->MODER&=~(3<<8);GPIOD->MODER|=1<<8;} 
#define BIN1_IN  {RCC->IOPENR|=1<<3;GPIOD->MODER&=~(3<<8);GPIOD->MODER|=0<<8;} 
#define BIN1_SET GPIOD->ODR|=1<<4
#define BIN1_RESET  GPIOD->ODR&=~(1<<4)

//BIN2 PD5
#define BIN2_OUT {RCC->IOPENR|=1<<3;GPIOD->MODER&=~(3<<10);GPIOD->MODER|=1<<10;} 
#define BIN2_IN  {RCC->IOPENR|=1<<3;GPIOD->MODER&=~(3<<10);GPIOD->MODER|=0<<10;} 
#define BIN2_SET GPIOD->ODR|=1<<5
#define BIN2_RESET  GPIOD->ODR&=~(1<<5)

//A相右轮 B相左轮
#define mot_stop {AIN2_RESET;AIN1_RESET;BIN1_RESET;BIN2_RESET;}
#define mot_break {AIN2_SET;AIN1_SET;BIN1_SET;BIN2_SET;}
#define mot_stright {AIN2_RESET;AIN1_SET;BIN1_SET;BIN2_RESET;}
#define mot_back {AIN2_SET;AIN1_RESET;BIN1_RESET;BIN2_SET;}
#define mot_left {AIN2_RESET;AIN1_SET;BIN1_RESET;BIN2_SET;}
#define mot_right {AIN2_SET;AIN1_RESET;BIN1_SET;BIN2_RESET;}	
	
typedef struct 
{
	int16_t x;
	int16_t y;
	int16_t z;
}i16_xyz;

typedef struct 
{
	uint16_t x;
	uint16_t y;
	uint16_t z;
}u16_xyz;

typedef struct 
{
	float x;
	float y;
	float z;
}float_xyz;


#define PI (3.1416)
#define Gyro_X_OFFSET (-31)
#define Gyro_Z_OFFSET (-13)
#define DIFFERENCE (50)
#define Angle_OFFSET 0

extern float Balance_Kp,Balance_Kd  ,Velocity_Kp,Velocity_Ki ;
extern uint8_t F_TASK_MPU6050;
extern uint8_t Abnormal_Flag ;
void TASK_MPU6050(void);
void BEEP_TASK(void);
void TASK_STATUS(void);
void IO_Init(void);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);
#endif

