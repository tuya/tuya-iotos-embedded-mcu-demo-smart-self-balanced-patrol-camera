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
void IIC_Start(void)//产生IIC起始信号
{
	IIC_SDA_OUT;     //sda线输出
	IIC_SDA_SET;//IIC_SDA=1;	  	  
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SDA_RESET;//IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
}
void IIC_Stop(void)//产生IIC停止信号
{
	IIC_SDA_OUT;//sda线输出
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_RESET;//IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1; 
	delay_us(5);
	IIC_SDA_SET;//IIC_SDA=1;//发送I2C总线结束信号				   	
}
void IIC_Ack(void)//产生ACK应答
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_RESET;//IIC_SDA=0;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}   
void IIC_NAck(void)//不产生ACK应答	
{
	IIC_SCL_RESET;//IIC_SCL=0;
	IIC_SDA_OUT;//SDA_OUT();
	IIC_SDA_SET;//IIC_SDA=1;
	delay_us(5);
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);
	IIC_SCL_RESET;//IIC_SCL=0;
}	
uint8_t IIC_Wait_Ack(void)//等待应答信号到来:1,接收应答失败;0,接收应答成功
{
	uint8_t ucErrTime=0;
	IIC_SDA_IN;      //SDA设置为输入     
	IIC_SCL_SET;//IIC_SCL=1;
	delay_us(5);	 
	while(IIC_SDA_State)//检测SDA是否仍为高电平
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
void IIC_Send_Byte(uint8_t txd)//IIC发送一个字节; 先发送高位
{                        
	uint8_t t;   
	IIC_SDA_OUT; 	    
	IIC_SCL_RESET;//IIC_SCL=0;//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
			//IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_SET;//IIC_SDA=1;
		else
			IIC_SDA_RESET;//IIC_SDA=0;
		
		txd<<=1; 	  
		delay_us(5);   //对TEA5767这三个延时都是必须的
		IIC_SCL_SET;//IIC_SCL=1;
		delay_us(5); 
		IIC_SCL_RESET;//IIC_SCL=0;	
		delay_us(5);
	}	 
} 
uint8_t IIC_Read_Byte(unsigned char ack)//读一个字节，可加是否应答位,1加ack，0不加ack 从高位开始读
{
	unsigned char i,receive=0;
	IIC_SDA_IN;//SDA设置为输入
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
			IIC_Ack(); //发送ACK
		else
			IIC_NAck();//发送nACK   
		return receive;
}

uint8_t IIC_Write_Byte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t data)//直接写一个字节
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //发送写命令
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//发送地址	  
	ret |= IIC_Wait_Ack();		
	IIC_Send_Byte(data);     //发送字节							   
	ret |= IIC_Wait_Ack(); 

	IIC_Stop();
	delay_us(10);
	return ret;
}
uint8_t IIC_ReadMulByte(uint8_t DrvAddr,uint16_t ReadAddr,uint8_t *pBuffer,uint16_t NumToRead)//读字节
{  	    																 
	uint8_t ret=0;
	
	IIC_Start();  
	IIC_Send_Byte(DrvAddr);	   //发送写命令
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(ReadAddr);//发送地址	    
	ret |= IIC_Wait_Ack();	    
	
	IIC_Start();
	IIC_Send_Byte(DrvAddr+1);           //进入接收模式			   
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
	IIC_Stop();//产生一个停止条件	
	return ret;	
}
uint8_t IIC_WriteMulByte(uint8_t DrvAddr,uint16_t WriteAddr,uint8_t *pBuffer,uint16_t NumToWrite)//可一次写多个字节
{
	uint8_t ret=0;
	IIC_Start();

	IIC_Send_Byte(DrvAddr);	    //发送写命令
	ret |= IIC_Wait_Ack();
	IIC_Send_Byte(WriteAddr);//发送地址	  
	ret |= IIC_Wait_Ack();		

	while(NumToWrite--)
	{ 										  		   
		IIC_Send_Byte(*pBuffer);     //发送字节							   
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
	IIC_Send_Byte(I2C_Addr);	   //发送写命令
	res++;
	IIC_Wait_Ack();
	IIC_Send_Byte(addr); res++;  //发送地址
	IIC_Wait_Ack();	  
	//IIC_Stop();//产生一个停止条件	
	IIC_Start();
	IIC_Send_Byte(I2C_Addr+1); res++;          //进入接收模式			   
	IIC_Wait_Ack();
	res=IIC_Read_Byte(0);	   
  IIC_Stop();//产生一个停止条件

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

uint8_t Way_Angle=2;                             //获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波 
int Temperature;                            //显示温度
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Acceleration_Z;                       //Z轴加速度计
float Accel_Angle;


uint8_t Abnormal_Flag = 0;      //电机异常状态， 0：正常    1：小车倾倒      2：小车被捡起

void Get_Angle(uint8_t way)
{ 
	float Accel_Y,Accel_Z,Gyro_X,Gyro_Z;
	Temperature=Read_Temperature();      //===读取MPU6050内置温度传感器数据，近似表示主板温度。
	if(way==1)                           //===DMP的读取在数据采集中断读取，严格遵循时序要求
	{	
			Read_DMP();                      //===读取加速度、角速度、倾角
			Angle_Balance=-Roll;             //===更新平衡倾角
			Gyro_Balance=-gyro[0];            //===更新平衡角速度
			Gyro_Turn=gyro[2];               //===更新转向角速度
			Acceleration_Z=accel[2];         //===更新Z轴加速度计
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
		//Accel_G=sqrt(pow(Accel_X, 2) + pow(Accel_Y, 2) + pow(Accel_Z, 2));
		if(Gyro_X>32768)  Gyro_X-=65536;                       //数据类型转换  也可通过short强制类型转换
		if(Gyro_Z>32768)  Gyro_Z-=65536;                       //数据类型转换
		if(Accel_Y>32768) Accel_Y-=65536;                      //数据类型转换
		if(Accel_Z>32768) Accel_Z-=65536;                      //数据类型转换
		Gyro_X=Gyro_X-Gyro_X_OFFSET;
		Gyro_Z=Gyro_Z-Gyro_Z_OFFSET;
		//printf("%.2f,%.2f\r\n",Gyro_X,Gyro_Z);
		Gyro_Balance=Gyro_X;                                  //更新平衡角速度
		Accel_Angle=atan2(Accel_Y,Accel_Z)*180/PI;                 //计算倾角	
		Gyro_X=Gyro_X/16.4;                                    //陀螺仪量程转换	
		if(way==2)		  	
			Kalman_Filter(Accel_Angle,Gyro_X);//卡尔曼滤波	
		else if(way==3)   
			First_order_Filter(Accel_Angle,Gyro_X);    //互补滤波
		Angle_Balance=angle;                                     //更新平衡倾角
		Gyro_Turn=Gyro_Z;                                      //更新转向角速度
		Acceleration_Z=Accel_Z;                                //===更新Z轴加速度计	
		
	}
}


//float Balance_Kp=600,Balance_Kd=2.4  ,Velocity_Kp=170,Velocity_Ki=0.85 ;//PID参数 //0
//float Balance_Kp=465,Balance_Kd=2.2  ,Velocity_Kp=120,Velocity_Ki=0.4 ;//PID参数 //1
float Balance_Kp=600,Balance_Kd=2.2  ,Velocity_Kp=150,Velocity_Ki=0.8 ;//PID参数 //2
int balance(float Angle,float Gyro)
{  
   float Bias;
	 int balancePID;
	 Bias=Angle-Angle_OFFSET;                       //===求出平衡的角度中值 和机械相关
	 balancePID=Balance_Kp*Bias+Gyro*Balance_Kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balancePID;
}
uint8_t Turn_Off(float angle, int voltage)
{
	    uint8_t temp;
	    if(angle<-45||angle>45||Direction.Current==TURN_OFF)
			{	                                                 //===倾角大于45度关闭电机
				temp=1;                                            //===Flag_Stop置1关闭电机
				mot_stop;
				Abnormal_Flag = 1;
				Direction.Current=TURN_OFF;

      }
			else if(Pick_Up(Acceleration_Z,Angle_Balance,Enconder_left,Enconder_right)){//===检查是否小车被拿起
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
	//=============遥控前进后退部分=======================// 
	Target_Velocity=40;                 
	if(Direction.Current==GO_STRAIGHT)    	Movement=-Target_Velocity/Flag_speed;	         //===前进标志位置1 
	else if(Direction.Current==GO_BACK)	Movement=Target_Velocity/Flag_speed;         //===后退标志位置1
	else  Movement=0;	
 //=============速度PI控制器=======================//	
	Encoder_Least =(encoder_left+encoder_right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
	Encoder *= 0.8;		                                                //===一阶低通滤波器       
	Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
	Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
	Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
	if(Encoder_Integral>8000)  	Encoder_Integral=8000;             //===积分限幅
	if(Encoder_Integral<-8000)	Encoder_Integral=-8000;              //===积分限幅	
	Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===速度控制	
	if(Turn_Off(Angle_Balance,BAT_VOL)==1||Direction.Current==TURN_OFF)   
		Encoder_Integral=0;      //===电机关闭后清除积分
	return Velocity;
}
int turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
	static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count; 
	float Turn_Amplitude=40/Flag_speed,Kp=32,Kd=0; 	
	//=============遥控左右旋转部分=======================//
	if(Direction.Current==TURN_LEFT||Direction.Current==TURN_RIGHT)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
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

	if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
	if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
	if(Direction.Current==GO_STRAIGHT||Direction.Current==GO_BACK||Direction.Current==KEEP_STOP)  Kd=-1 ;        
	else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
	//=============转向PD控制器=======================//
	Turn=-Turn_Target*Kp-gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
	return Turn;
}
int Moto1,Moto2; 
void Limit_Pwm(void)
{	
	int Amplitude=7600;    //===PWM满幅是8000 限制在7600

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
函数功能：检测小车是否被拿起
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	static uint16_t flag,count0,count1,count2;
	if(flag==0)                                                                   //第一步
	 {
	      if(myabs(encoder_left)+myabs(encoder_right)<30){
				    count0++;
				}                        //条件1，小车接近静止
        else{
            count0=0;	
				}

        if(count0>10)				
		    flag=1,count0=0; 
	 } 
	 if(flag==1)                                                                  //进入第二步
	 {
		    if(++count1>200){
				    count1=0;
				    flag=0;
				}                                 //超时不再等待2000ms
	      if(Acceleration>15000&&(Angle>(-20+Angle_OFFSET))&&(Angle<(20+Angle_OFFSET)))   //条件2，小车是在0度附近被拿起
		    flag=2; 
	 } 
	 if(flag==2)                                                                  //第三步
	 {
		  if(++count2>100){
			    count2=0;
				  flag=0;   
			}                                //超时不再等待1000ms
	    if(myabs(encoder_left+encoder_right)>135)                                 //条件3，小车的轮胎因为正反馈达到最大的转速   
      {
				flag=0;
        				
				return 1;                                                               //检测到小车被拿起
			}
	 }
	 return 0;
}

/**************************************************************************
函数功能：检测小车是否被放下
入口参数：int
返回  值：unsigned int
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static uint16_t flag,count;	 
	 if(Abnormal_Flag != 2)                           //防止误检      
   return 0;	                 
	 if(flag==0)                                               
	 {
	      if(Angle>(-10+Angle_OFFSET)&&Angle<(10+Angle_OFFSET)&&encoder_left==0&&encoder_right==0)         //条件1，小车是在0度附近的
		    flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                                          //超时不再等待 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left<-3&&encoder_right<-3&&encoder_left>-60&&encoder_right>-60)                //条件2，小车的轮胎在未上电的时候被人为转动  
      {
				flag=0;
				return 1;                                             //检测到小车被放下
			}
	 }
	return 0;
}


int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
uint8_t F_TASK_MPU6050=0;
void TASK_MPU6050(void)
{
	READ_Encoder();
	Get_Angle(Way_Angle);                                               //===更新姿态		
	Balance_Pwm = balance(Angle_Balance,Gyro_Balance);                   //===平衡PID控制	
	Velocity_Pwm= velocity(Enconder_left,Enconder_right);                  //===速度环PID控制	 记住，速度反馈是正反馈，就是小车快的时候要慢下来就需要再跑快一点
	Turn_Pwm   = turn(Enconder_left,Enconder_right,Gyro_Turn);            //===转向环PID控制	
	Moto1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                            //===计算左轮电机最终PWM
	Moto2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                            //===计算右轮电机最终PWM
	Limit_Pwm();                                                       //===PWM限幅	
	
	if(Abnormal_Flag == 0){
			Set_Pwm(Moto1,Moto2);                                               //===赋值给PWM寄存器 
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
  MPU6050_initialize();           //=====MPU6050初始化	
  DMP_Init();                     //=====初始化DMP   	
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
