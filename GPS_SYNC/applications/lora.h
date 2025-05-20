#ifndef __LORA_H
#define __LORA_H

#include "header.h"

#define LORAMASTER_KEY		 GPIOA
#define LORAMASTER_KEY_PIN GPIO_PIN_15

#define LORAMASTER_STA		 GPIOB
#define LORAMASTER_STA_PIN GPIO_PIN_3

typedef struct lora
{
	uint16_t     KEY_PIN;	 //loraģ��key���������趨����������͵�ƽ�趨����
	GPIO_TypeDef *KEY;		 //key����GPIO
	
	uint16_t     STA_PIN;	 //loraģ��sta��������ȷ����ǰ����״̬������
	GPIO_TypeDef *STA;		 //sta����GPIO
	
	uint32_t 			baud;		 //�������趨
	
	uint8_t       receiveData[23];	//��������
	uint8_t 			transmitData[16];
	
	uint8_t 			receiveFlag;			//�������ݱ�־λ
	
	uint8_t       channel;	// �����ŵ� �ŵ���Χ1-50���������������Ϊ0
	
	uint8_t       speed;		//�������� 1-8 Խ�ʹ������ԽԶ Խ�ߴ������Խ������Խ��
	
	uint8_t       power;		//���书��6-20 
	
	int8_t  			staFlag;	//״̬��־λ��1�ǵ�ǰATָ��OK 2�ǵ�ǰ������Ϊ115200 3�� ȡ�����Ƕ�Ӧ��ģʽ����״̬error
	uint8_t       setFlag;	//ģʽ�趨��־λ
}lora_define;

extern lora_define loraMaster;		//����loraģ��
#define LORA_RX_LEN 23

// loraģ���ʼ��
void Lora_Init(void);

// ��loraģ�鷢������
void Lora_Transmit_Cmd(char str[]);
// ��loraģ�鷢������
void Lora_Transmit_Data(uint8_t str[],uint16_t length);
// �鿴loraģ�鵱ǰ�Ƿ�����
void CheckSet_Lora_STA(lora_define *mylora);
// �鿴loraģ�鵱ǰ�������Ƿ���ȷ������ȷ���趨
void CheckSet_Lora_BAUD(lora_define *mylora);
// �鿴�����ŵ���
void CheckSet_Lora_Channel(lora_define *mylora);
// �鿴����ͨѶ����
void CheckSet_Lora_Speed(lora_define *mylora);
// �鿴����ͨѶ���书��
void CheckSet_Lora_POWER(lora_define *mylora);

// ����ʱ�����
void Lora_Transmit_Time_Test(void);
// ������յ�������
void Lora_Decode(void);
#endif // __LORA_H



