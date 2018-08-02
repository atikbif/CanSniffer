/*
 * canLogger.h
 *
 *  Created on: 2 рту. 2018 у.
 *      Author: Roman
 */

#ifndef CANLOGGER_H_
#define CANLOGGER_H_

#define LOG_CNT	5

struct logBuf {
	unsigned char buf[528];
	unsigned char fillFlag;
	struct logBuf *next;
}logs[LOG_CNT];

void initLogger();
struct logBuf* getFirstLog();
void canLoggerTask(void const * argument);
void clearLogBuf(struct logBuf *ptr);

#endif /* CANLOGGER_H_ */
