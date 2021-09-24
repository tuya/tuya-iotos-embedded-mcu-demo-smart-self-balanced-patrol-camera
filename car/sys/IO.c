#include "IO.h"
#include "delay.h"
#include "ADC.h"
#include "TIM.h"
#include "EXTI.h"
#include "USART.h"
#include "math.h"
#include "stdlib.h"
#include "MPU6050.h"
#include "filter.h"
#if 1//IIC  //IIC_SDA	  PB11  //IIC_SCL	  PB12
void IIC_Init(void)
{
	IIC_SCL_OUT;
	IIC_SDA_OUT;
}
void IIC_Start(void)//����IIC��ʼ�ź�
{
	IIC_SDA_OUT;     //sda�����
	IIC_SDA_SET;//IIC_SDA=1;	  	  
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SDA_RESET;//IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;//ǯסI2C���ߣ�׼�����ͻ�������� 
}
void IIC_Stop(void)//����IICֹͣ�ź�
{
	IIC_SDA_OUT;//sda�����
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_RESET;//IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1; 
	delay_us(5);
	IIC_SDA_SET;//IIC_SDA=1;//����I2C���߽����ź�				   	
}
void IIC_Ack(void)//����ACKӦ��
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_RESET;//IIC_SDA=0;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}   
void IIC_NAck(void)//������ACKӦ��	
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_SET;//IIC_SDA=1;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}	
uint8_t IIC_Wait_Ack(void)//�ȴ�Ӧ���źŵ���:1,����Ӧ��ʧ��;0,����Ӧ��ɹ�
{
	uint8_t ucErrTime=0;
	IIC_SDA_IN;      //SDA����Ϊ����     
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);	 
	while(IIC_SDA_State)//���SDA�Ƿ���Ϊ�ߵ�ƽ
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_RESET;//IIC_SCL=0;
	return 0;  
} 
void IIC_Send_Byte(uint8_t txd)//IIC����һ���ֽ�; �ȷ��͸�λ
{                        
	uint8_t t;   
	IIC_SDA_OUT; 	    
	IIC_SCL_RESET;//IIC_SCL=0;//����ʱ�ӿ�ʼ���ݴ���
	for(t=0;t<8;t++)
	{              
			//IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_SET;//IIC_SDA=1;
		else
			IIC_SDA_RESET;//IIC_SDA=0;
		
		txd<<=1; 	  
		delay_us(5);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_SET;//IIC_SCL=1;
		delay_us(5); 
		IIC_SCL_RESET;//IIC_SCL=0;	
		delay_us(5);
	}	 
} 
uint8_t IIC_Read_Byte(unsigned char ack)//��һ���ֽڣ��ɼ��Ƿ�Ӧ��λ,1��ack��0����ack �Ӹ�λ��ʼ��
{
	unsigned char i,receive=0;
	IIC_SDA_IN;//SDA����Ϊ����
	for(i=0;i<8;i++ )
	{
		IIC_SCL_RESET;// IIC_SCL=0; 
		delay_us(5);
		IIC_SCL_SET;//IIC_SCL=1;
		receive<<=1;
		if(IIC_SDA_State)
			receive++;   
		delay_us(5); 
	}					 
		if (ack)
			IIC_Ack(); //����ACK
		else
			IIC_NAck();//����nACK   
		return receive;
}

uint8_t IIC_Write_Byte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t data)//ֱ��дһ���ֽ�
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//���͵�ַ	  
	ret |= IIC_Wait_Ack();		
	IIC_Send_Byte(data);     //�����ֽ�							   
	ret |= IIC_Wait_Ack(); 

	IIC_Stop();
	delay_us(10);
	return ret;
}
uint8_t IIC_ReadMulByte(uint8_t DrvAddr,uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)//���ֽ�
{  	    																 
	uint8_t ret=0;
	
	IIC_Start();  
	IIC_Send_Byte(DrvAddr);	   //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);//���͵�ַ	    
	ret |= IIC_Wait_Ack();	    
	
	IIC_Start();
	IIC_Send_Byte(DrvAddr+1);           //�������ģʽ			   
	ret |= IIC_Wait_Ack();
	while(NumToRead)
	{
		if(NumToRead==1)
		{
			*pBuffer=IIC_Read_Byte(0);	
		}
		else
		{
			*pBuffer=IIC_Read_Byte(1);
		}
		pBuffer++;
		NumToRead--;
	}
	IIC_Stop();//����һ��ֹͣ����	
	return ret;	
}
uint8_t IIC_WriteMulByte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)//��һ��д����ֽ�
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //����д����
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//���͵�ַ	  
	ret |= IIC_Wait_Ack();		

	while(NumToWrite--)
	{ 										  		   
		IIC_Send_Byte(*pBuffer);     //�����ֽ�							   
		ret |= IIC_Wait_Ack(); 
		pBuffer++;
	}
	IIC_Stop();
	delay_us(10);
	return ret;
}

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	uint8_t ret=0;
	ret |= IIC_WriteMulByte( addr<<1,  reg,  data, len);
	return ret;
}
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
	uint8_t ret=0;
	ret |= IIC_ReadMulByte( addr<<1,  reg,  data, len);
	return ret;	
}
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr)
{
	unsigned char res=0;
	
	IIC_Start();	
	IIC_Send_Byte(I2C_Addr);	   //����д����
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //���͵�ַ
	IIC_Wait_Ack();	  
	//IIC_Stop();//����һ��ֹͣ����	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //�������ģʽ			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
  IIC_Stop();//����һ��ֹͣ����

	return res;
}
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	IIC_ReadMulByte( dev,  reg,  data, length);
	return length;	
}
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
	IIC_WriteMulByte( dev,  reg,  data, length);
	return 1;	
}
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
{
	*data=I2C_ReadOneByte(dev, reg);
    return 1;
}
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}
uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data)
{

    uint8_t b;
    if (IICreadByte(dev, reg, &b) != 0) {
        uint8_t mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}
uint8_t IICwriteBit(uint8_t dev, uint8_t reg, uint8_t bitNum, uint8_t data)
{
    uint8_t b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}
#endif

/************************************************************************************************************/
#if 1 //MPU6050

uint8_t Way_Angle=2;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲� 
int Temperature;                            //��ʾ�¶�
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Acceleration_Z;                       //Z����ٶȼ�
float Accel_Angle;


uint8_t Abnormal_Flag = 0;      //����쳣״̬�� 0������    1��С���㵹      2��С��������

void Get_Angle(uint8_t way)
{ 
	float Accel_Y,Accel_Z,Gyro_X,Gyro_Z;
	Temperature=Read_Temperature();      //===��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
	if(way==1)                           //===DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
	{	
			Read_DMP();                      //===��ȡ���ٶȡ����ٶȡ����
			Angle_Balance=-Roll;             //===����ƽ�����
			Gyro_Balance=-gyro[0];            //===����ƽ����ٶ�
			Gyro_Turn=gyro[2];               //===����ת����ٶ�
			Acceleration_Z=accel[2];         //===����Z����ٶȼ�
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡY��������
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ�
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
		//Accel_G=sqrt(pow(Accel_X, 2) + pow(Accel_Y, 2) + pow(Accel_Z, 2));
		if(Gyro_X>32768)  Gyro_X-=65536;                       //��������ת��  Ҳ��ͨ��shortǿ������ת��
		if(Gyro_Z>32768)  Gyro_Z-=65536;                       //��������ת��
		if(Accel_Y>32768) Accel_Y-=65536;                      //��������ת��
		if(Accel_Z>32768) Accel_Z-=65536;                      //��������ת��
		Gyro_X=Gyro_X-Gyro_X_OFFSET;
		Gyro_Z=Gyro_Z-Gyro_Z_OFFSET;
		//printf("%.2f,%.2f\r\n",Gyro_X,Gyro_Z);
		Gyro_Balance=Gyro_X;                                  //����ƽ����ٶ�
		Accel_Angle=atan2(Accel_Y,Accel_Z)*180/PI;                 //�������	
		Gyro_X=Gyro_X/16.4;                                    //����������ת��	
		if(way==2)		  	
			Kalman_Filter(Accel_Angle,Gyro_X);//�������˲�	
		else if(way==3)   
			First_order_Filter(Accel_Angle,Gyro_X);    //�����˲�
		Angle_Balance=angle;                                     //����ƽ�����
		Gyro_Turn=Gyro_Z;                                      //����ת����ٶ�
		Acceleration_Z=Accel_Z;                                //===����Z����ٶȼ�	
		
	}
}


//float Balance_Kp=600,Balance_Kd=2.4  ,Velocity_Kp=170,Velocity_Ki=0.85 ;//PID���� //0
//float Balance_Kp=465,Balance_Kd=2.2  ,Velocity_Kp=120,Velocity_Ki=0.4 ;//PID���� //1
float Balance_Kp=600,Balance_Kd=2.2  ,Velocity_Kp=150,Velocity_Ki=0.8 ;//PID���� //2
int balance(float Angle,float Gyro)
{  
   float Bias;
	 int balancePID;
	 Bias=Angle-Angle_OFFSET;                       //===���ƽ��ĽǶ���ֵ �ͻ�е���
	 balancePID=Balance_Kp*Bias+Gyro*Balance_Kd;   //===����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	 return balancePID;
}
uint8_t Turn_Off(float angle, int voltage)
{
	    uint8_t temp;
	    if(angle<-45||angle>45||Direction.Current==TURN_OFF)
			{	                                                 //===��Ǵ���45�ȹرյ��
				temp=1;                                            //===Flag_Stop��1�رյ��
				mot_stop;
				Abnormal_Flag = 1;
				Direction.Current=TURN_OFF;

      }
			else if(Pick_Up(Acceleration_Z,Angle_Balance,Enconder_left,Enconder_right)){//===����Ƿ�С��������
				temp=1;
				mot_stop;
				Abnormal_Flag = 2;
				Direction.Current=TURN_OFF;

			}
			else if(Put_Down(Angle_Balance,Enconder_left,Enconder_right)){
			  temp=0;
				Abnormal_Flag = 0;
				Direction.Current=KEEP_STOP;

			}
			else{
	      temp=0;
			}

      return temp;			
}
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
uint8_t Flag_speed=4;
int velocity(int encoder_left,int encoder_right)
{  
	static float Velocity,Encoder_Least,Encoder,Movement;
	static float Encoder_Integral,Target_Velocity;
	//=============ң��ǰ�����˲���=======================// 
	Target_Velocity=40;                 
	if(Direction.Current==GO_STRAIGHT)    	Movement=-Target_Velocity/Flag_speed;	         //===ǰ����־λ��1 
	else if(Direction.Current==GO_BACK)	Movement=Target_Velocity/Flag_speed;         //===���˱�־λ��1
	else  Movement=0;	
 //=============�ٶ�PI������=======================//	
	Encoder_Least =(encoder_left+encoder_right)-0;                    //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩 
	Encoder *= 0.8;		                                                //===һ�׵�ͨ�˲���       
	Encoder += Encoder_Least*0.2;	                                    //===һ�׵�ͨ�˲���    
	Encoder_Integral +=Encoder;                                       //===���ֳ�λ�� ����ʱ�䣺10ms
	Encoder_Integral=Encoder_Integral-Movement;                       //===����ң�������ݣ�����ǰ������
	if(Encoder_Integral>8000)  	Encoder_Integral=8000;             //===�����޷�
	if(Encoder_Integral<-8000)	Encoder_Integral=-8000;              //===�����޷�	
	Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===�ٶȿ���	
	if(Turn_Off(Angle_Balance,BAT_VOL)==1||Direction.Current==TURN_OFF)   
		Encoder_Integral=0;      //===����رպ��������
	return Velocity;
}
int turn(int encoder_left,int encoder_right,float gyro)//ת�����
{
	static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count; 
	float Turn_Amplitude=40/Flag_speed,Kp=32,Kd=0; 	
	//=============ң��������ת����=======================//
	if(Direction.Current==TURN_LEFT||Direction.Current==TURN_RIGHT)                      //��һ������Ҫ�Ǹ�����תǰ���ٶȵ����ٶȵ���ʼ�ٶȣ�����С������Ӧ��
	{
		if(++Turn_Count==1)
		Encoder_temp=myabs(encoder_left+encoder_right);
		Turn_Convert=50/Encoder_temp;
		if(Turn_Convert<0.6)Turn_Convert=0.6;
		if(Turn_Convert>3)Turn_Convert=3;
	}	
	else
	{
		Turn_Convert=0.9;
		Turn_Count=0;
		Encoder_temp=0;
	}			
	if(Direction.Current==TURN_LEFT){
		Turn_Target+=Turn_Convert;
	}
	else if(Direction.Current==TURN_RIGHT){
		Turn_Target-=Turn_Convert; 
	}
	else Turn_Target=0;

	if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===ת���ٶ��޷�
	if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
	if(Direction.Current==GO_STRAIGHT||Direction.Current==GO_BACK||Direction.Current==KEEP_STOP)  Kd=-1 ;        
	else Kd=0;   //ת���ʱ��ȡ�������ǵľ��� �е�ģ��PID��˼��
	//=============ת��PD������=======================//
	Turn=-Turn_Target*Kp-gyro*Kd;                 //===���Z�������ǽ���PD����
	return Turn;
}
int Moto1,Moto2; 
void Limit_Pwm(void)
{	
	int Amplitude=7600;    //===PWM������8000 ������7600

	if(Moto1<-Amplitude) Moto1=-Amplitude;	
	if(Moto1>Amplitude)  Moto1=Amplitude;	
	if(Moto2<-Amplitude) Moto2=-Amplitude;	
	if(Moto2>Amplitude)  Moto2=Amplitude;		
}
void Set_Pwm(int moto1,int moto2)
{     
	if(moto2>0)			{AIN2_RESET;AIN1_SET;}
	else 	          {AIN2_SET;AIN1_RESET;}
	TIM16_PWM_Set(myabs(moto2));
	
	if(moto1>0)	{BIN1_RESET;			BIN2_SET;}
	else        {BIN1_SET;			BIN2_RESET;}
	TIM17_PWM_Set(myabs(moto1));

}

/**************************************************************************
�������ܣ����С���Ƿ�����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	static uint16_t flag,count0,count1,count2;
	if(flag==0)                                                                   //��һ��
	 {
	      if(myabs(encoder_left)+myabs(encoder_right)<30){
				    count0++;
				}                        //����1��С���ӽ���ֹ
        else{
            count0=0;	
				}

        if(count0>10)				
		    flag=1,count0=0; 
	 } 
	 if(flag==1)                                                                  //����ڶ���
	 {
		    if(++count1>200){
				    count1=0;
				    flag=0;
				}                                 //��ʱ���ٵȴ�2000ms
	      if(Acceleration>15000&&(Angle>(-20+Angle_OFFSET))&&(Angle<(20+Angle_OFFSET)))   //����2��С������0�ȸ���������
		    flag=2; 
	 } 
	 if(flag==2)                                                                  //������
	 {
		  if(++count2>100){
			    count2=0;
				  flag=0;   
			}                                //��ʱ���ٵȴ�1000ms
	    if(myabs(encoder_left+encoder_right)>135)                                 //����3��С������̥��Ϊ�������ﵽ����ת��   
      {
				flag=0;
        				
				return 1;                                                               //��⵽С��������
			}
	 }
	 return 0;
}

/**************************************************************************
�������ܣ����С���Ƿ񱻷���
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static uint16_t flag,count;	 
	 if(Abnormal_Flag != 2)                           //��ֹ���      
   return 0;	                 
	 if(flag==0)                                               
	 {
	      if(Angle>(-10+Angle_OFFSET)&&Angle<(10+Angle_OFFSET)&&encoder_left==0&&encoder_right==0)         //����1��С������0�ȸ�����
		    flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                                          //��ʱ���ٵȴ� 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left<-3&&encoder_right<-3&&encoder_left>-60&&encoder_right>-60)                //����2��С������̥��δ�ϵ��ʱ����Ϊת��  
      {
				flag=0;
				return 1;                                             //��⵽С��������
			}
	 }
	return 0;
}


int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
uint8_t F_TASK_MPU6050=0;
void TASK_MPU6050(void)
{
	READ_Encoder();
	Get_Angle(Way_Angle);                                               //===������̬		
	Balance_Pwm = balance(Angle_Balance,Gyro_Balance);                   //===ƽ��PID����	
	Velocity_Pwm= velocity(Enconder_left,Enconder_right);                  //===�ٶȻ�PID����	 ��ס���ٶȷ�����������������С�����ʱ��Ҫ����������Ҫ���ܿ�һ��
	Turn_Pwm   = turn(Enconder_left,Enconder_right,Gyro_Turn);            //===ת��PID����	
	Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                            //===�������ֵ������PWM
	Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                            //===�������ֵ������PWM
	Limit_Pwm();                                                       //===PWM�޷�	
	
	if(Abnormal_Flag == 0){
			Set_Pwm(Moto1,Moto2);                                               //===��ֵ��PWM�Ĵ��� 
	}
 //printf("%.2f,%.2f,%.2f\r\n",Accel_Angle,angle,angle1);
}
#endif
void BEEP_TASK(void)
{
	BEEP_OUT;
	TIM6_Stop();
	BEEP_SET;
	delay_ms(1500);
	BEEP_RESET;
  TIM6_Start();	
}
void TASK_STATUS(void)
{
	//printf(" a=%.1f", Accel_Angle);
	  //printf(" next=%d", Direction.Next);
	//printf(" L=%d R=%d", Enconder_left,Enconder_right);
	//printf(" L=%d R=%d \r\n", TIM17->CCR1,TIM16->CCR1);
}
void Modules_Init(void)
{
	IIC_Init();
  MPU6050_initialize();           //=====MPU6050��ʼ��	
  DMP_Init();                     //=====��ʼ��DMP   	
}
void SwitchIO_Init(void)
{
	BEEP_OUT;
	BEEP_TASK();
	Direction_Init();
	EXTI0_Init();
	AIN2_OUT;
	AIN1_OUT;
	BIN1_OUT;
	BIN2_OUT;
}
void IO_Init(void)
{
	SwitchIO_Init();
	Modules_Init();
}
