#ifndef __HEADER_H
#define __HEADER_H
/* ��Ƭ���� */
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
//#include "adc.h"
#include "stm32f1xx_hal.h"
//#include "sdio.h"
//#include "i2c.h"
#include "struct_typedef.h"

/* C���Կ� */
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"	 

// �����������ݵķ�Χ
#define LIMIT_VAL(a,min,max) ((a)<(min)?(min):((a)>(max)?(max):(a)))		

/* �û��� */
//#include "sys.h"
#include "crc.h"
//#include "delay.h"
//#include "MyUsart.h"
//#include "malloc.h"

//#include "eeprom.h"
//#include "oled.h"
//#include "matrix_key.h"
//#include "config_init.h"
#include "schedule.h"
//#include "time_stamp.h"
//#include "myiic.h"
//#include "ds1307.h"
//#include "calendar.h"
#include "lora.h"

//#include "sdio_sdcard.h"
//#include "bsp_driver_sd.h"
//#include "sd_diskio.h"
//#include "sd_card.h"
//#include "fatfs.h"

#include "gps.h"

#endif
