# Tuya IoTOS Embedded Mcu Demo Smart self-balanced Patrol Camera

[English](./README.md) | [中文](./README_zh.md)

## Introduction  

This Demo uses the Tuya smart cloud platform, Tuya smart APP, Tuya Pylon camera and  MCU to realize a smart self-balanced patrol camera.



The implemented features include:

+ Keep balance
+ Patrol mobile
+  Video surveillance
+ Remote calls

## Quick start  

### Compile & Burn
+ Download Tuya IoTOS Embedded Code
+ Execute the test.uvprojx file
+ Click Compile in the software and complete the download

### File introduction 

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



### Demo entry

Entry file ：main.c

Important functions ：main()

+ Initialize and configure MCU IO port, USART, timer, etc. All events are polled and judged in while(1)。



### I/O List  

|    I/O     |            Function             |
| :--------: | :-----------------------------: |
|  PB4  PB5  |  Right wheel encoder detection  |
|  PA0  PA1  |  Left wheel encoder detection   |
|    PD0     |     Right wheel PWM control     |
|    PD1     |     Left wheel PWM control      |
|  PD2  PD3  | Right wheel  direction  control |
|  PD4  PD5  | Left wheel  direction  control  |
|    PA7     |               ADC               |
| PC10  PC11 |    Left /Right motor signal     |
| PB14  PB15 |    Forward/Back motor signal    |
|    PC3     |              BEEP               |
| PC12  PC13 |           MPU6050_IIC           |

## Related Documents

  Tuya Demo Center: https://developer.tuya.com/demo



## Technical Support

  You can get support for Tuya by using the following methods:

- Developer Center: https://developer.tuya.com
- Help Center: https://support.tuya.com/help
- Technical Support Work Order Center: [https://service.console.tuya.com](https://service.console.tuya.com/) 

