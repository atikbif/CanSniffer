/*
 * canViewer.c
 *
 *  Created on: 31 èþë. 2018 ã.
 *      Author: Roman
 */


#include "canViewer.h"
#include "cmsis_os.h"
#include "spi_flash.h"
#include "stm32f4xx_hal.h"

osThreadId canViewerTaskHandle;

static unsigned char fl_buf[528];

void CanViewerTask(void const * argument) {
	unsigned short pageNum = 0;
	unsigned short i=0;
	for(;;)
	{
		for(i=0;i<528;i++) {fl_buf[i]=0;}
		fl_buf[0] = pageNum >> 8;
		fl_buf[1] = pageNum & 0xFF;
		write_page(pageNum,fl_buf);
		osDelay(20);
		//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
		pageNum++;if(pageNum>=8192) pageNum=0;
	}
}

