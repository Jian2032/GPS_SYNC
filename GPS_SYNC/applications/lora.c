#include "lora.h"

lora_define loraMaster;		//主机lora模块


// lora模块初始化
void Lora_Init(void)
{
	loraMaster.KEY = LORAMASTER_KEY;
	loraMaster.KEY_PIN = LORAMASTER_KEY_PIN;
	
	loraMaster.STA = LORAMASTER_STA;
	loraMaster.STA_PIN = LORAMASTER_STA_PIN;
	
	// 设定无线信道
	loraMaster.channel = 28;
	// 设定通讯速度
	loraMaster.speed = 8;
	// 设定发射频率
	loraMaster.power = 20;
	
	// 接收数据清0复位
	memset(loraMaster.receiveData,0,16);
	
	// 接收数据标志位清0
	loraMaster.receiveFlag = 0;
	
	//判断返回值是否OK
	CheckSet_Lora_STA(&loraMaster);
	//判断波特率是否正确
	CheckSet_Lora_BAUD(&loraMaster);
	// 查询模块的无线信道，不对进行设定
	CheckSet_Lora_Channel(&loraMaster);
	// 查询模块的无线速率，不对进行设定
	CheckSet_Lora_Speed(&loraMaster);
	// 查看无线通讯发射功率，不对进行设定
	CheckSet_Lora_POWER(&loraMaster);
	
}

// 给lora模块发送命令
void Lora_Transmit_Cmd(char str[])
{
	HAL_UART_Transmit_DMA(&huart3,(uint8_t *)str,strlen(str));
}

// 查看lora模块当前是否正常
void CheckSet_Lora_STA(lora_define *mylora)
{
	// 设定为参数查询或者参数设置模式
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_RESET);
	HAL_Delay(50);
	
	//询问当前lora模块工作状态，发送AT 每次发送消息重复3次防止没有接收到
	for(int i=0;i<3;i++)
	{
		Lora_Transmit_Cmd("AT");
		HAL_Delay(50);	 //模块可以接受的最快通讯频率25hz 这里用20hz
	}
	while(!strstr((char *)mylora->receiveData,"OK"))
	{
		mylora->staFlag = -1;
	}	//若不返回当前模块出现问题 staFlag = 0
	if(strstr((char *)mylora->receiveData,"OK"))
	{
		mylora->staFlag = 1;
		// 数据清0
		memset(mylora->receiveData,0,sizeof(mylora->receiveData));
	}
	HAL_Delay(50);
	
	// 如果不正常则把模块设定为出厂模式
	if(mylora->staFlag == -1)
	{
		mylora->setFlag = 1;	//设定标志位置1，代表在进行恢复出厂模式
		// 发送恢复出厂设置指令
		for(int i=0;i<3;i++)
		{
			Lora_Transmit_Cmd("AT+DEFAULT");
			HAL_Delay(50);	 //模块可以接受的最快通讯频率25hz 这里用20hz
		}
		while(!strstr((char *)mylora->receiveData,"DEFAULT"))	//如果没有返回DEFAULT模块彻底寄了
		{
			mylora->staFlag = -128;	//模块彻底寄了
		}
		if(strstr((char *)mylora->receiveData,"DEFAULT"))
		{
			mylora->staFlag = 1;
			mylora->setFlag = 0;
		}
	}
	
	// 设定为正常模式
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_SET);
	HAL_Delay(50);
}

// 查看lora模块当前波特率是否正确，不正确则设定
void CheckSet_Lora_BAUD(lora_define *mylora)
{
	// 设定为参数查询或者参数设置模式
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_RESET);
	HAL_Delay(50);
	
	if(mylora->staFlag == 1)
	{
		for(int i=0;i<3;i++)
		{
			//询问当前lora模块的波特率
			Lora_Transmit_Cmd("AT+B?");
			HAL_Delay(50);	//模块可以接受的最快通讯频率25hz 这里用20hz
		}
		while(!strstr((char *)mylora->receiveData,"115200"))
		{
			mylora->staFlag = -2;
		}	//等待波特率返回是否为115200
		if(strstr((char *)mylora->receiveData,"115200"))
		{
			mylora->staFlag = 2;
			mylora->baud = atoi((char *)mylora->receiveData);
		}
	}
	
	// 设定波特率涉及到改变串口波特率的操作，这里先不做修改
	
	// 设定为正常模式
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_SET);
	HAL_Delay(50);
}

// 查看无线信道数
void CheckSet_Lora_Channel(lora_define *mylora)
{
	// 设定为参数查询或者参数设置模式
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_RESET);
	HAL_Delay(50);
	
	uint8_t tempChannel;
	
	// 想通讯成功必须在同一个无线信道
	if(mylora->staFlag == 2)
	{
		for(int i=0;i<3;i++)
		{
			//询问当前lora模块的无线信道
			Lora_Transmit_Cmd("AT+C?");
			HAL_Delay(50);	//模块可以接受的最快通讯频率25hz 这里用20hz
		}
		char strtemp[3];
		for(int i=0;i<3;i++)
		{
			strtemp[i] = mylora->receiveData[i];
		}
		while(!(atoi(strtemp) >= 1 && atoi(strtemp) <= 50))
		{
			mylora->staFlag = -3;	//信道非法
			mylora->channel = 0;	//
		}
		if(atoi(strtemp) >= 1 && atoi(strtemp) <= 50)
		{
			tempChannel = atoi(strtemp);
		}
	}
	
	// 设定无线信道数
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
			//询问当前lora模块的无线信道
			Lora_Transmit_Cmd(strtemp);
			HAL_Delay(50);	//模块可以接受的最快通讯频率25hz 这里用20hz
		}
		memset(strtemp,0,8);
		for(int i=0;i<3;i++)
		{
			strtemp[i] = mylora->receiveData[i];
		}
		while(!(atoi(strtemp) >= 1 && atoi(strtemp) <= 50))
		{
			mylora->staFlag = -3;	//信道非法
			mylora->channel = 0;	//
		}
		if(atoi(strtemp) == mylora->channel)
		{
			mylora->staFlag = 3; //模式OK
		}		
	}
	
	// 设定为正常模式
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_SET);
	HAL_Delay(50);
}

// 查看无线通讯速率
void CheckSet_Lora_Speed(lora_define *mylora)
{
	// 设定为参数查询或者参数设置模式
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_RESET);
	HAL_Delay(50);
	
	uint8_t tempSpeed;
	
	// 查询当前无线速率
	if(mylora->staFlag == 3)
	{
		for(int i=0;i<3;i++)
		{
			//询问当前lora模块的无线速率
			Lora_Transmit_Cmd("AT+S?");
			HAL_Delay(50);	//模块可以接受的最快通讯频率25hz 这里用20hz
		}
		char strtemp[1];
		strtemp[0] = mylora->receiveData[5];
		while(!(atoi(strtemp) >= 1 && atoi(strtemp) <= 8))
		{
			mylora->staFlag = -4;	//速率非法
			mylora->speed = 0;	//
		}
		if(atoi(strtemp) >= 1 && atoi(strtemp) <= 8)
		{
			tempSpeed = atoi(strtemp);
		}
	}
	
	// 设定无线信道数
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
			//询问当前lora模块的速度
			Lora_Transmit_Cmd(strtemp);
			HAL_Delay(50);	//模块可以接受的最快通讯频率25hz 这里用20hz
		}
		memset(strtemp,0,8);

		strtemp[0] = mylora->receiveData[5];
		while(!(atoi(strtemp) >= 1 && atoi(strtemp) <= 8))
		{
			mylora->staFlag = -4;	//速度非法
			mylora->channel = 0;	//
		}
		if(atoi(strtemp) == mylora->speed)
		{
			mylora->staFlag = 4; //模式OK
		}		
	}
	
	// 设定为正常模式
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_SET);
	HAL_Delay(50);
}

// 查看无线通讯发射功率
void CheckSet_Lora_POWER(lora_define *mylora)
{
	// 设定为参数查询或者参数设置模式
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_RESET);
	HAL_Delay(50);
	
	uint8_t temPower;
	
	// 查询当前无线速率
	if(mylora->staFlag == 4)
	{
		for(int i=0;i<3;i++)
		{
			//询问当前lora模块的发射功率
			Lora_Transmit_Cmd("AT+P?");
			HAL_Delay(50);	//模块可以接受的最快通讯频率25hz 这里用20hz
		}
		char strtemp[2];
		strtemp[0] = mylora->receiveData[0];
		strtemp[1] = mylora->receiveData[1];
		while(!(atoi(strtemp) >= 6 && atoi(strtemp) <= 20))
		{
			mylora->staFlag = -5;	//发射功率非法
			mylora->power = 0;	//发射功率
		}
		if(atoi(strtemp) >= 6 && atoi(strtemp) <= 20)
		{
			temPower = atoi(strtemp);
		}
	}

	// 设定发射功率
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
			//询问当前lora模块的功率
			Lora_Transmit_Cmd(strtemp);
			HAL_Delay(50);	//模块可以接受的最快通讯频率25hz 这里用20hz
		}
		memset(strtemp,0,6);
		strtemp[0] = mylora->receiveData[0];
		strtemp[1] = mylora->receiveData[1];
		while(!(atoi(strtemp) >= 6 && atoi(strtemp) <= 20))
		{
			mylora->staFlag = -5;	//发射功率非法
			mylora->power = 0;	//
		}
		if(atoi(strtemp) == mylora->speed)
		{
			mylora->staFlag = 5; //模式OK
		}		
	}	
	
	// 设定为正常模式
	HAL_GPIO_WritePin(mylora->KEY,mylora->KEY_PIN,GPIO_PIN_SET);
	HAL_Delay(50);
}

// 给lora模块发送数据
void Lora_Transmit_Data(uint8_t str[],uint16_t length)
{
	HAL_UART_Transmit_DMA(&huart3,str,length);
}


// 发送时间测试
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

// 定义接收到的GPS数据
extern gps_packet gps_receive;
// 解算接收到的数据
void Lora_Decode(void)
{
	/* 确定起始标志位 */
	if(loraMaster.receiveData[0] == 0x55 && loraMaster.receiveData[1] == 0xAA)
	{
		/* 获取接收到数据的最后两位 */
		uint8_t last_1 = loraMaster.receiveData[21];
		uint8_t last   = loraMaster.receiveData[22];
		/* CRC校验 */
		Append_CRC16_Check_Sum(loraMaster.receiveData, 23);
		
		/* 如果满足要求进行解算 */
		if(last_1 == loraMaster.receiveData[21] && last == loraMaster.receiveData[22])
		{
					memcpy(gps_receive.bytes,loraMaster.receiveData,sizeof(loraMaster.receiveData));
		}
	}
}

