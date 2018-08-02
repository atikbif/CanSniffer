#include "canLogger.h"
#include "cmsis_os.h"
#include "spi_flash.h"
#include "stm32f4xx_hal.h"

static unsigned short pageNum = 0;

void initLogger() {
	int i = 0;
	for(i=0;i<LOG_CNT-1;i++) {
		logs[i].next = &logs[i+1];
		logs[i].fillFlag = 0;
	}
	logs[i].next = &logs[0];
	logs[i].fillFlag = 0;
}

struct logBuf* getFirstLog() {
	return &logs[0];
}

void clearLogBuf(struct logBuf *ptr) {
	int i=0;
	for(i=0;i<528;i++) ptr->buf[i] = 0;
}

void canLoggerTask(void const * argument) {
	int i=0;
	//erase_chip();
	for(;;)
	{
		for(i=0;i<LOG_CNT;i++) {
			if(logs[i].fillFlag) {
				write_page(pageNum,logs[i].buf);
				pageNum++;if(pageNum>=8192) pageNum=0;
				logs[i].fillFlag = 0;
				HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
				osDelay(1);
			}
		}
		osDelay(1);
	}
}
