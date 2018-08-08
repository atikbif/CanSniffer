/*
 * datetime.c
 *
 *  Created on: 8 ���. 2018 �.
 *      Author: Roman
 */

#include "datetime.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#define JD0 2451545 // ���� �� 01 ��� 2000 ��

extern RTC_HandleTypeDef hrtc;
static RTC_TimeTypeDef sTime;
static RTC_DateTypeDef sDate;
static ftime_t sdt;

unsigned long cTime = 0;

static unsigned long EncodeDateTime(ftime_t * ftime)
{
	uint8_t a;
	uint16_t y;
	uint8_t m;
	uint32_t JDN;

	// ���������� ����������� �������������
	a=(14-ftime->month)/12;
	y=ftime->year+4800-a;
	m=ftime->month+(12*a)-3;
	// ��������� �������� �������� ���������� ���
	JDN=ftime->day;
	JDN+=(153*m+2)/5;
	JDN+=365*y;
	JDN+=y/4;
	JDN+=-y/100;
	JDN+=y/400;
	JDN+=-32045;
	JDN+=-JD0; // ��� ��� ������� � ��� �����������, ������ ��� ������� ������ �� 01 ��� 2001
	JDN*=86400;     // ��������� ��� � �������
	JDN+=(ftime->hour*3600); // � ��������� ��� �������� �������� ���
	JDN+=(ftime->minute*60);
	JDN+=(ftime->second);
	// ����� ����� ���������� ������ � 00-00 01 ��� 2000
	return JDN;
}


void datetimeTask(void const * argument) {
	for(;;)
	{
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		portDISABLE_INTERRUPTS();
		sdt.second = sTime.Seconds;
		sdt.minute = sTime.Minutes;
		sdt.hour = sTime.Hours;
		sdt.day = sDate.Date;
		sdt.month = sDate.Month;
		sdt.year = sDate.Year + 2000;
		cTime = EncodeDateTime(&sdt);
		portENABLE_INTERRUPTS();
		osDelay(1000);
	}
}

void updateCurrentTime(unsigned long counter) {

	uint32_t ace;
	uint8_t b;
	uint8_t d;
	uint8_t m;

	ace=(counter/86400)+32044+JD0;
	b=(4*ace+3)/146097;
	ace=ace-((146097*b)/4);
	d=(4*ace+3)/1461;
	ace=ace-((1461*d)/4);
	m=(5*ace+2)/153;

	portDISABLE_INTERRUPTS();

	sdt.day=ace-((153*m+2)/5)+1;
	sdt.month=m+3-(12*(m/10));
	sdt.year=100*b+d-4800+(m/10);
	sdt.hour=(counter/3600)%24;
	sdt.minute=(counter/60)%60;
	sdt.second=(counter%60);

	sTime.Seconds = sdt.second;
	sTime.Minutes = sdt.minute;
	sTime.Hours = sdt.hour;
	sDate.Date = sdt.day;
	sDate.Month = sdt.month;
	if(sdt.year>=2000) sDate.Year = sdt.year - 2000;else sDate.Year = 0;
	HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	cTime = counter;

	portENABLE_INTERRUPTS();
}
