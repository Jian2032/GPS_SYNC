#ifndef __LORA_H
#define __LORA_H

#include "header.h"

#define LORAMASTER_KEY		 GPIOA
#define LORAMASTER_KEY_PIN GPIO_PIN_15

#define LORAMASTER_STA		 GPIOB
#define LORAMASTER_STA_PIN GPIO_PIN_3

typedef struct lora
{
	uint16_t     KEY_PIN;	 //lora模块key引脚用来设定参数，输出低电平设定参数
	GPIO_TypeDef *KEY;		 //key控制GPIO
	
	uint16_t     STA_PIN;	 //lora模块sta引脚用来确定当前工作状态，输入
	GPIO_TypeDef *STA;		 //sta控制GPIO
	
	uint32_t 			baud;		 //波特率设定
	
	uint8_t       receiveData[23];	//接收数据
	uint8_t 			transmitData[16];
	
	uint8_t 			receiveFlag;			//接收数据标志位
	
	uint8_t       channel;	// 无线信道 信道范围1-50，如果有问题设置为0
	
	uint8_t       speed;		//无线速率 1-8 越低传输距离越远 越高传输距离越近速率越快
	
	uint8_t       power;		//发射功率6-20 
	
	int8_t  			staFlag;	//状态标志位，1是当前AT指令OK 2是当前波特率为115200 3是 取负数是对应的模式或者状态error
	uint8_t       setFlag;	//模式设定标志位
}lora_define;

extern lora_define loraMaster;		//主机lora模块
#define LORA_RX_LEN 23

// lora模块初始化
void Lora_Init(void);

// 给lora模块发送数据
void Lora_Transmit_Cmd(char str[]);
// 给lora模块发送数据
void Lora_Transmit_Data(uint8_t str[],uint16_t length);
// 查看lora模块当前是否正常
void CheckSet_Lora_STA(lora_define *mylora);
// 查看lora模块当前波特率是否正确，不正确则设定
void CheckSet_Lora_BAUD(lora_define *mylora);
// 查看无线信道数
void CheckSet_Lora_Channel(lora_define *mylora);
// 查看无线通讯速率
void CheckSet_Lora_Speed(lora_define *mylora);
// 查看无线通讯发射功率
void CheckSet_Lora_POWER(lora_define *mylora);

// 发送时间测试
void Lora_Transmit_Time_Test(void);
// 解算接收到的数据
void Lora_Decode(void);
#endif // __LORA_H



