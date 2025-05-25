#include "schedule.h"
extern gps_packet gps_send;
extern nmea_msg gpsx; 

void TDT_Loop_1000Hz(void) // 1msִ��һ��
{
}

float loop_time_500hz;
void TDT_Loop_500Hz(void) // 2msִ��һ��
{
}

void TDT_Loop_200Hz(void) // 5msִ��һ��
{
}

void TDT_Loop_100Hz(void) // 10msִ��һ��
{
}

void TDT_Loop_50Hz(void) // 20msִ��һ��
{
}

void TDT_Loop_20Hz(void) // 50msִ��һ��
{
		gps_read(&gpsx,&gps_send);
}

void TDT_Loop_10Hz(void) // 100msִ��һ��
{
		Append_CRC16_Check_Sum(gps_send.bytes,sizeof(gps_send.bytes));
		HAL_UART_Transmit_DMA(&huart2,gps_send.bytes,sizeof(gps_send.bytes));
}

void TDT_Loop_2Hz(void) // 500msִ��һ��
{
}

void TDT_Loop_1Hz(void) // 1000msִ��һ��
{
}

void TDT_Loop(schedule *robotSchdule)
{
    if (robotSchdule->cnt_1ms >= 1)
    {
        TDT_Loop_1000Hz();
        robotSchdule->cnt_1ms = 0;
    }
    if (robotSchdule->cnt_2ms >= 2)
    {
        TDT_Loop_500Hz();
        robotSchdule->cnt_2ms = 0;
    }
    if (robotSchdule->cnt_5ms >= 5)
    {
        TDT_Loop_200Hz();
        robotSchdule->cnt_5ms = 0;
    }
    if (robotSchdule->cnt_10ms >= 10)
    {
        TDT_Loop_100Hz();
        robotSchdule->cnt_10ms = 0;
    }
    if (robotSchdule->cnt_20ms >= 20)
    {
        TDT_Loop_50Hz();
        robotSchdule->cnt_20ms = 0;
    }
    if (robotSchdule->cnt_50ms >= 50)
    {
        TDT_Loop_20Hz();
        robotSchdule->cnt_50ms = 0;
    }
    if (robotSchdule->cnt_100ms >= 100)
    {
        TDT_Loop_10Hz();
        robotSchdule->cnt_100ms = 0;
    }
    if (robotSchdule->cnt_500ms >= 500)
    {
        TDT_Loop_2Hz();
        robotSchdule->cnt_500ms = 0;
    }
    if (robotSchdule->cnt_1000ms >= 1000)
    {
        TDT_Loop_1Hz();
        robotSchdule->cnt_1000ms = 0;
    }
}
