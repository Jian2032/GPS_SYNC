#include "schedule.h"

extern gps_packet gps_send;
extern int asd;
int zxc = 0;

void TDT_Loop_1000Hz(void) // 1ms执行一次
{
}

float loop_time_500hz;
void TDT_Loop_500Hz(void) // 2ms执行一次
{
}

void TDT_Loop_200Hz(void) // 5ms执行一次
{
}

void TDT_Loop_100Hz(void) // 10ms执行一次
{
}

void TDT_Loop_50Hz(void) // 20ms执行一次
{
}

void TDT_Loop_20Hz(void) // 50ms执行一次
{
		HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&gps_send.data, sizeof(gps_send.data));
}

void TDT_Loop_10Hz(void) // 100ms执行一次
{
}

void TDT_Loop_2Hz(void) // 500ms执行一次
{
}
int qwe;
void TDT_Loop_1Hz(void) // 1000ms执行一次
{
	zxc++;
	qwe=asd/zxc;
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
