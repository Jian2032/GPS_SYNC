#ifndef __SCHEDULE_H
#define __SCHEDULE_H

/*!***************************************************
 * @file: schedule.c
 * @brief: ������ȿ�ܣ�Ϊ�˽��ԭ������޷�ʹ��OLED��ظ�������
				���Բ������µ�schedule���
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

void TDT_Loop_1000Hz(void); // 1msִ��һ��
void TDT_Loop_500Hz(void);	// 2msִ��һ��
void TDT_Loop_200Hz(void);	// 5msִ��һ��
void TDT_Loop_100Hz(void);	// 10msִ��һ��
void TDT_Loop_50Hz(void);	// 20msִ��һ��
void TDT_Loop_20Hz(void);	// 50msִ��һ��
void TDT_Loop_10Hz(void);	// 100msִ��һ��
void TDT_Loop_2Hz(void);	// 500msִ��һ��
void TDT_Loop_1Hz(void);	// 1000msִ��һ��

void TDT_Loop(schedule *robotSchdule);

#endif
