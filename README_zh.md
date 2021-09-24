# Tuya IoTOS Embedded Mcu Demo Smart self-balanced Patrol Camera

[English](./README.md) | [中文](./README_zh.md)

## 简介 

本Demo通过涂鸦智能云平台、涂鸦智能APP、涂鸦云台摄像机和 MCU 实现一款智能巡逻摄像头。

已实现功能包括：

+ 保持平衡

+ 巡逻移动

+ 视频监控

+ 远程通话

  



## 快速上手 

### 编译与烧录
+ 下载Tuya IoTOS Embeded Code

+ 执行test.uvprojx文件

+ 点击软件中的编译，并完成下载


### 文件介绍 

```
├── User
│   ├── main.c
│   ├── MY_ST_config.h
├── CMSIS
│   ├── system_stm32g0xx.c
│   ├── startup_stm32g071xx.s
├── System
│   ├── sys.c
│   ├── sys.h
│   ├── RCC.c
│   ├── RCC.h
│   ├── delay.c
│   ├── delay.h
│   ├── USART.c
│   ├── USART.h
│   ├── IO.c
│   ├── IO.h
│   ├── ADC.c
│   ├── ADC.h
│   ├── TIM.c
│   ├── TIM.h
└── Balance
    ├── MPU6050.c
    ├── MPU6050.h
    ├── inv_mpu.c
    ├── inv_mpu.h
    ├── inv_mpu_dmp_motion_driver.c
    ├── inv_mpu_dmp_motion_driver.h
    ├── filter.c
    ├── filter.h 
    ├── dmpKey.h      
    └── dmpmap.h
    
```



### Demo入口

入口文件：main.c

重要函数：main()

+ 对mcu的IO口，USART，定时器等进行初始化配置，所有事件在while(1)中轮询判断。





### I/O 列表 

|    I/O     |       功能       |
| :--------: | :--------------: |
|  PB4  PB5  |  右轮编码器检测  |
|  PA0  PA1  |  左轮编码器检测  |
|    PD0     | 右轮速度PWM控制  |
|    PD1     | 左轮速度PWM控制  |
|  PD2  PD3  |   右轮方向控制   |
|  PD4  PD5  |   左轮方向控制   |
|    PA7     |   电池电量ADC    |
| PC10  PC11 | 左右步进电机信号 |
| PB14  PB15 | 前后步进电机信号 |
|    PC3     |      蜂鸣器      |
| PC12  PC13 | MPU6050_IIC接口  |

## 相关文档

涂鸦Demo中心：https://developer.tuya.com/demo



## 技术支持

您可以通过以下方法获得涂鸦的支持:

- 开发者中心：https://developer.tuya.com
- 帮助中心: https://support.tuya.com/help
- 技术支持工单中心: [https://service.console.tuya.com](https://service.console.tuya.com/) 