#include "lora.h"

lora_define loraMaster;		//����loraģ��


// loraģ���ʼ��
void Lora_Init(void)
{
	loraMaster.KEY = LORAMASTER_KEY;
	loraMaster.KEY_PIN = LORAMASTER_KEY_PIN;
	
	loraMaster.STA = LORAMASTER_STA;
	loraMaster.STA_PIN = LORAMASTER_STA_PIN;
	
	// �趨�����ŵ�
	loraMaster.channel = 28;
	// �趨ͨѶ�ٶ�
	loraMaster.speed = 8;
	// �趨����Ƶ��
	loraMaster.power = 20;
	
	// ����������0��λ
	memset(loraMaster.receiveData,0,16);
	
	// �������ݱ�־λ��0
	loraMaster.receiveFlag = 0;
	
	//�жϷ���ֵ�Ƿ�OK
	CheckSet_Lora_STA(&loraMaster);
	//�жϲ������Ƿ���ȷ
	CheckSet_Lora_BAUD(&loraMaster);
	// ��ѯģ��������ŵ������Խ����趨
	CheckSet_Lora_Channel(&loraMaster);
	// ��ѯģ����������ʣ����Խ����趨
	CheckSet_Lora_Speed(&loraMaster);
	// �鿴����ͨѶ���书�ʣ����Խ����趨
	CheckSet_Lora_POWER(&loraMaster);
	
}

// ��loraģ�鷢������
void Lora_Transmit_Cmd(char str[])
{
	HAL_UART_Transmit_DMA(&huart3,(uint8_t *)str,strlen(str));
}

// �鿴loraģ�鵱ǰ�Ƿ�����
void CheckSet_Lora_STA(lora_define *mylora)
{
	// �趨Ϊ������ѯ���߲�������ģʽ
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_RESET);
	HAL_Delay(50);
	
	//ѯ�ʵ�ǰloraģ�鹤��״̬������AT ÿ�η�����Ϣ�ظ�3�η�ֹû�н��յ�
	for(int i=0;i<3;i++)
	{
		Lora_Transmit_Cmd("AT");
		HAL_Delay(50);	 //ģ����Խ��ܵ����ͨѶƵ��25hz ������20hz
	}
	while(!strstr((char *)mylora->receiveData,"OK"))
	{
		mylora->staFlag = -1;
	}	//�������ص�ǰģ��������� staFlag = 0
	if(strstr((char *)mylora->receiveData,"OK"))
	{
		mylora->staFlag = 1;
		// ������0
		memset(mylora->receiveData,0,sizeof(mylora->receiveData));
	}
	HAL_Delay(50);
	
	// ������������ģ���趨Ϊ����ģʽ
	if(mylora->staFlag == -1)
	{
		mylora->setFlag = 1;	//�趨��־λ��1�������ڽ��лָ�����ģʽ
		// ���ͻָ���������ָ��
		for(int i=0;i<3;i++)
		{
			Lora_Transmit_Cmd("AT+DEFAULT");
			HAL_Delay(50);	 //ģ����Խ��ܵ����ͨѶƵ��25hz ������20hz
		}
		while(!strstr((char *)mylora->receiveData,"DEFAULT"))	//���û�з���DEFAULTģ�鳹�׼���
		{
			mylora->staFlag = -128;	//ģ�鳹�׼���
		}
		if(strstr((char *)mylora->receiveData,"DEFAULT"))
		{
			mylora->staFlag = 1;
			mylora->setFlag = 0;
		}
	}
	
	// �趨Ϊ����ģʽ
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_SET);
	HAL_Delay(50);
}

// �鿴loraģ�鵱ǰ�������Ƿ���ȷ������ȷ���趨
void CheckSet_Lora_BAUD(lora_define *mylora)
{
	// �趨Ϊ������ѯ���߲�������ģʽ
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_RESET);
	HAL_Delay(50);
	
	if(mylora->staFlag == 1)
	{
		for(int i=0;i<3;i++)
		{
			//ѯ�ʵ�ǰloraģ��Ĳ�����
			Lora_Transmit_Cmd("AT+B?");
			HAL_Delay(50);	//ģ����Խ��ܵ����ͨѶƵ��25hz ������20hz
		}
		while(!strstr((char *)mylora->receiveData,"115200"))
		{
			mylora->staFlag = -2;
		}	//�ȴ������ʷ����Ƿ�Ϊ115200
		if(strstr((char *)mylora->receiveData,"115200"))
		{
			mylora->staFlag = 2;
			mylora->baud = atoi((char *)mylora->receiveData);
		}
	}
	
	// �趨�������漰���ı䴮�ڲ����ʵĲ����������Ȳ����޸�
	
	// �趨Ϊ����ģʽ
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_SET);
	HAL_Delay(50);
}

// �鿴�����ŵ���
void CheckSet_Lora_Channel(lora_define *mylora)
{
	// �趨Ϊ������ѯ���߲�������ģʽ
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_RESET);
	HAL_Delay(50);
	
	uint8_t tempChannel;
	
	// ��ͨѶ�ɹ�������ͬһ�������ŵ�
	if(mylora->staFlag == 2)
	{
		for(int i=0;i<3;i++)
		{
			//ѯ�ʵ�ǰloraģ��������ŵ�
			Lora_Transmit_Cmd("AT+C?");
			HAL_Delay(50);	//ģ����Խ��ܵ����ͨѶƵ��25hz ������20hz
		}
		char strtemp[3];
		for(int i=0;i<3;i++)
		{
			strtemp[i] = mylora->receiveData[i];
		}
		while(!(atoi(strtemp) >= 1 && atoi(strtemp) <= 50))
		{
			mylora->staFlag = -3;	//�ŵ��Ƿ�
			mylora->channel = 0;	//
		}
		if(atoi(strtemp) >= 1 && atoi(strtemp) <= 50)
		{
			tempChannel = atoi(strtemp);
		}
	}
	
	// �趨�����ŵ���
	if(tempChannel == mylora->channel)
	{
		mylora->staFlag = 3;
	}
	else
	{
		char strtemp[8];
		sprintf(strtemp,"AT+C%03d",mylora->channel);
		for(int i=0;i<3;i++)
		{
			//ѯ�ʵ�ǰloraģ��������ŵ�
			Lora_Transmit_Cmd(strtemp);
			HAL_Delay(50);	//ģ����Խ��ܵ����ͨѶƵ��25hz ������20hz
		}
		memset(strtemp,0,8);
		for(int i=0;i<3;i++)
		{
			strtemp[i] = mylora->receiveData[i];
		}
		while(!(atoi(strtemp) >= 1 && atoi(strtemp) <= 50))
		{
			mylora->staFlag = -3;	//�ŵ��Ƿ�
			mylora->channel = 0;	//
		}
		if(atoi(strtemp) == mylora->channel)
		{
			mylora->staFlag = 3; //ģʽOK
		}		
	}
	
	// �趨Ϊ����ģʽ
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_SET);
	HAL_Delay(50);
}

// �鿴����ͨѶ����
void CheckSet_Lora_Speed(lora_define *mylora)
{
	// �趨Ϊ������ѯ���߲�������ģʽ
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_RESET);
	HAL_Delay(50);
	
	uint8_t tempSpeed;
	
	// ��ѯ��ǰ��������
	if(mylora->staFlag == 3)
	{
		for(int i=0;i<3;i++)
		{
			//ѯ�ʵ�ǰloraģ�����������
			Lora_Transmit_Cmd("AT+S?");
			HAL_Delay(50);	//ģ����Խ��ܵ����ͨѶƵ��25hz ������20hz
		}
		char strtemp[1];
		strtemp[0] = mylora->receiveData[5];
		while(!(atoi(strtemp) >= 1 && atoi(strtemp) <= 8))
		{
			mylora->staFlag = -4;	//���ʷǷ�
			mylora->speed = 0;	//
		}
		if(atoi(strtemp) >= 1 && atoi(strtemp) <= 8)
		{
			tempSpeed = atoi(strtemp);
		}
	}
	
	// �趨�����ŵ���
	if(tempSpeed == mylora->speed)
	{
		mylora->staFlag = 4;
	}
	else
	{
		char strtemp[8];
		sprintf(strtemp,"AT+S%d",mylora->speed);
		for(int i=0;i<3;i++)
		{
			//ѯ�ʵ�ǰloraģ����ٶ�
			Lora_Transmit_Cmd(strtemp);
			HAL_Delay(50);	//ģ����Խ��ܵ����ͨѶƵ��25hz ������20hz
		}
		memset(strtemp,0,8);

		strtemp[0] = mylora->receiveData[5];
		while(!(atoi(strtemp) >= 1 && atoi(strtemp) <= 8))
		{
			mylora->staFlag = -4;	//�ٶȷǷ�
			mylora->channel = 0;	//
		}
		if(atoi(strtemp) == mylora->speed)
		{
			mylora->staFlag = 4; //ģʽOK
		}		
	}
	
	// �趨Ϊ����ģʽ
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_SET);
	HAL_Delay(50);
}

// �鿴����ͨѶ���书��
void CheckSet_Lora_POWER(lora_define *mylora)
{
	// �趨Ϊ������ѯ���߲�������ģʽ
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_RESET);
	HAL_Delay(50);
	
	uint8_t temPower;
	
	// ��ѯ��ǰ��������
	if(mylora->staFlag == 4)
	{
		for(int i=0;i<3;i++)
		{
			//ѯ�ʵ�ǰloraģ��ķ��书��
			Lora_Transmit_Cmd("AT+P?");
			HAL_Delay(50);	//ģ����Խ��ܵ����ͨѶƵ��25hz ������20hz
		}
		char strtemp[2];
		strtemp[0] = mylora->receiveData[0];
		strtemp[1] = mylora->receiveData[1];
		while(!(atoi(strtemp) >= 6 && atoi(strtemp) <= 20))
		{
			mylora->staFlag = -5;	//���书�ʷǷ�
			mylora->power = 0;	//���书��
		}
		if(atoi(strtemp) >= 6 && atoi(strtemp) <= 20)
		{
			temPower = atoi(strtemp);
		}
	}

	// �趨���书��
	if(temPower == mylora->power)
	{
		mylora->staFlag = 5;
	}
	else
	{
		char strtemp[6];
		sprintf(strtemp,"AT+P%d",mylora->power);
		for(int i=0;i<3;i++)
		{
			//ѯ�ʵ�ǰloraģ��Ĺ���
			Lora_Transmit_Cmd(strtemp);
			HAL_Delay(50);	//ģ����Խ��ܵ����ͨѶƵ��25hz ������20hz
		}
		memset(strtemp,0,6);
		strtemp[0] = mylora->receiveData[0];
		strtemp[1] = mylora->receiveData[1];
		while(!(atoi(strtemp) >= 6 && atoi(strtemp) <= 20))
		{
			mylora->staFlag = -5;	//���书�ʷǷ�
			mylora->power = 0;	//
		}
		if(atoi(strtemp) == mylora->speed)
		{
			mylora->staFlag = 5; //ģʽOK
		}		
	}	
	
	// �趨Ϊ����ģʽ
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_SET);
	HAL_Delay(50);
}

// ��loraģ�鷢������
void Lora_Transmit_Data(uint8_t str[],uint16_t length)
{
	HAL_UART_Transmit_DMA(&huart3,str,length);
}


// ����ʱ�����
void Lora_Transmit_Time_Test(void)
{
	loraMaster.transmitData[0] = 0x06;
	loraMaster.transmitData[1] = 0xfe;
	
//	loraMaster.transmitData[2] = ds1307_data.year >> 8;
//	loraMaster.transmitData[3] = ds1307_data.year & 0xff;
//	loraMaster.transmitData[4] = ds1307_data.month;
//	loraMaster.transmitData[5] = ds1307_data.week;
//	loraMaster.transmitData[6] = ds1307_data.hour;
//	loraMaster.transmitData[7] = ds1307_data.minute;
//	loraMaster.transmitData[8] = ds1307_data.second;
	
//	loraMaster.transmitData[2] = time_stamp.time_stamp_10us >> 28;
//	loraMaster.transmitData[3] = time_stamp.time_stamp_10us >> 24;
//	loraMaster.transmitData[4] = time_stamp.time_stamp_10us >> 16;
//	loraMaster.transmitData[5] = time_stamp.time_stamp_10us >> 8;
//	loraMaster.transmitData[6] = time_stamp.time_stamp_10us & 0xff;
	
//	Append_CRC16_Check_Sum(loraMaster.transmitData,16);
	
	Lora_Transmit_Data(loraMaster.transmitData,16);
}

// ������յ���GPS����
extern gps_packet gps_receive;
// ������յ�������
void Lora_Decode(void)
{
	/* ȷ����ʼ��־λ */
	if(loraMaster.receiveData[0] == 0x55 && loraMaster.receiveData[1] == 0xAA)
	{
		/* ��ȡ���յ����ݵ������λ */
		uint8_t last_1 = loraMaster.receiveData[21];
		uint8_t last   = loraMaster.receiveData[22];
		/* CRCУ�� */
		Append_CRC16_Check_Sum(loraMaster.receiveData, 23);
		
		/* �������Ҫ����н��� */
		if(last_1 == loraMaster.receiveData[21] && last == loraMaster.receiveData[22])
		{
					memcpy(gps_receive.bytes,loraMaster.receiveData,sizeof(loraMaster.receiveData));
		}
	}
}

