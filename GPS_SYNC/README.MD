# GPS_SYNC

## 芯片STM32F103C8T6,GPS数据接收，激光雷达和相机时间同步，向上位机发送GPS消息，通过lora模块接收基站GPS消息

## 1. GPS数据接收 USART3 && DMA

USART3
引脚 RX ———— PB11

## 2. 激光雷达和相机时间同步 TIM1 && TIM2 && USART1

TIM1 PWM 中断 1HZ
引脚 TIM1_CH1 ———— PA8
TIM1产生1HZ的中断和PWM波形，中断目的是产生模拟GPS信号的时间戳，PWM是向MID360发送脉冲

USART1
引脚 TX ———— PA9
发送函数在TIM1中断中，发送模拟GPS信号

TIM2 PWM 10HZ
引脚 TIM2_CH2 ———— PA1
向相机发送10HZ脉冲

## 3. 向上位机发送GPS消息 USART3 && DMA
USART3
引脚 TX ———— PB10

## 4. 通过lora模块接收基站GPS消息
USART2
引脚 TX ———— PA2
引脚 RX ———— PA3
STA ———— PB3
KEY ———— PA15