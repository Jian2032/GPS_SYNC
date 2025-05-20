#ifndef __SCHEDULE_H
#define __SCHEDULE_H

/*!***************************************************
 * @file: schedule.c
 * @brief: 程序调度框架，为了解决原框架下无法使用OLED监控各项数据
				所以采用最新的schedule框架
 * @author: TDT
 * @date: 2021/10/14
 * @note:
 ****************************************************/
#include "header.h"

typedef struct _schedule
{
	uint16_t cnt_1ms;
	uint16_t cnt_2ms;
	uint16_t cnt_5ms;
	uint16_t cnt_10ms;
	uint16_t cnt_20ms;
	uint16_t cnt_50ms;
	uint16_t cnt_100ms;
	uint16_t cnt_500ms;
	uint16_t cnt_1000ms;
} schedule;

void TDT_Loop_1000Hz(void); // 1ms执行一次
void TDT_Loop_500Hz(void);	// 2ms执行一次
void TDT_Loop_200Hz(void);	// 5ms执行一次
void TDT_Loop_100Hz(void);	// 10ms执行一次
void TDT_Loop_50Hz(void);	// 20ms执行一次
void TDT_Loop_20Hz(void);	// 50ms执行一次
void TDT_Loop_10Hz(void);	// 100ms执行一次
void TDT_Loop_2Hz(void);	// 500ms执行一次
void TDT_Loop_1Hz(void);	// 1000ms执行一次

void TDT_Loop(schedule *robotSchdule);

#endif
