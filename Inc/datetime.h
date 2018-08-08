/*
 * datetime.h
 *
 *  Created on: 8 рту. 2018 у.
 *      Author: Roman
 */

#ifndef DATETIME_H_
#define DATETIME_H_

#include <stdlib.h>

typedef struct{
	unsigned short year;
	unsigned char month;
	unsigned char day;
	unsigned char hour;
	unsigned char minute;
	unsigned char second;
} ftime_t;

void updateCurrentTime(unsigned long counter);

void datetimeTask(void const * argument);

#endif /* DATETIME_H_ */
