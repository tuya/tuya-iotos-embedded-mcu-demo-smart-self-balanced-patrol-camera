# Tuya IoTOS Embedded Smart Self-Balancing Robot Car with Camera

[English](./README.md) | [中文](./README_zh.md)

## Introduction

In this demo, we will show you how to use the MCU SDK to develop a smart self-balancing robot car and connect it to the Tuya IoT Cloud with the Tuya Smart app. This robot has cameras on board, enabling you to control it to patrol your house.

Features:

+ Self-balancing

+ Patrol areas

+ Video surveillance

+ Remote control





## Get started

### Compile and flash
+ Download Tuya IoTOS Embedded Code.

+ Run `test.uvprojx`.

+ Click **Compile** on the software to download the code.


### File introduction

```
├── User
│   ├── main.c
│   ├── MY_ST_config.h
├── CMSIS
│   ├── system_stm32g0xx.c
│   ├── startup_stm32g071xx.s
├── System
│   ├── sys.c
│   ├── sys.h
│   ├── RCC.c
│   ├── RCC.h
│   ├── delay.c
│   ├── delay.h
│   ├── USART.c
│   ├── USART.h
│   ├── IO.c
│   ├── IO.h
│   ├── ADC.c
│   ├── ADC.h
│   ├── TIM.c
│   ├── TIM.h
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

Entry file: `main.c`

Main function: `main()`

+ Initialize and configure I/Os, USART, and timer of the MCU. All events are polled and determined in `while(1)`.





### Pin configuration

| I/O | Function |
| :--------: | :--------------: |
| PB4  PB5 | Detect the encoder of the right wheel. |
| PA0  PA1 | Detect the encoder of the left wheel. |
| PD0 | PWM signal for controlling the motor speed of the right wheel. |
| PD1 | PWM signal for controlling the motor speed of the left wheel. |
| PD2  PD3 | Control which direction the motor of the right wheel is turning. |
| PD4  PD5 | Control which direction the motor of the left wheel is turning. |
| PA7 | Battery level ADC |
| PC10  PC11 | The signal to control the stepper motor to move left or right. |
| PB14  PB15 | The signal to control the stepper motor to move forward or backward. |
| PC3 | Buzzer |
| PC12  PC13 | `MPU6050_IIC` interface |

## Reference

[Tuya Project Hub](https://developer.tuya.com/demo)


## Technical support

You can get support from Tuya with the following methods:

- [Tuya Developer Platform](https://developer.tuya.com/en/)
- [Help Center](https://support.tuya.com/en/help)
- [Service & Support](https://service.console.tuya.com/8/3/list?source=support_center)

<br>
