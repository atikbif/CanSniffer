/*
 * canViewer.c
 *
 *  Created on: 31 èþë. 2018 ã.
 *      Author: Roman
 */


#include "canViewer.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "canLogger.h"
#include "crc.h"
#include "datetime.h"

osThreadId canViewerTaskHandle;
static CAN_RxHeaderTypeDef   RxHeader;
static uint8_t               RxData[8];
static unsigned char crc_buf[64];
static unsigned char timeFlag = 1;
static unsigned long extTime = 0;

static struct logBuf* log;
static unsigned short remainBytes = 528;
extern unsigned long cTime;

extern CAN_HandleTypeDef hcan1;

static inline unsigned char writeByteToBuffer(unsigned char value) {
	if(remainBytes) {
		log->buf[528-remainBytes] = value;
	}else {
		if(log->next->fillFlag==0) {
			timeFlag=1;
			log->fillFlag = 1;
			log = log->next;
			remainBytes = 528;
			log->buf[0] = value;
		}else return 0;
	}
	remainBytes--;
	return 1;
}

void initCANViewerFilter() {
	CAN_FilterTypeDef  sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);
}



void canViewerTask(void const * argument) {
	//unsigned short pageNum = 0;
	unsigned short i=0;
	unsigned char crc8;
	initCANViewerFilter();
	HAL_CAN_Start(&hcan1);

	log = getFirstLog();
	for(;;)
	{
		if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)) {
			if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
				if(((RxData[0] & 0x1F)==0x1F)&&(RxData[1]==0x0E)&&(RxHeader.DLC==6)) {
					extTime = RxData[5];
					extTime<<=8;extTime|=RxData[4];
					extTime<<=8;extTime|=RxData[3];
					extTime<<=8;extTime|=RxData[2];
					//updateCurrentTime(extTime);
				}
				while(writeByteToBuffer(0x31)==0) {osDelay(1);}
				//while(writeByteToBuffer(0x01)==0) {osDelay(1);}
				//while(writeByteToBuffer(0x02)==0) {osDelay(1);}
				//while(writeByteToBuffer(0x03)==0) {osDelay(1);}
				//while(writeByteToBuffer(0x04)==0) {osDelay(1);}
				crc_buf[0]=RxHeader.StdId >> 8;while(writeByteToBuffer(RxHeader.StdId >> 8)==0) {osDelay(1);}
				crc_buf[1]=RxHeader.StdId & 0xFF;while(writeByteToBuffer(RxHeader.StdId & 0xFF)==0) {osDelay(1);}

				crc_buf[2]=(cTime >> 24) & 0xFF;while(writeByteToBuffer((cTime >> 24) & 0xFF)==0) {osDelay(1);}
				crc_buf[3]=(cTime >> 16) & 0xFF;while(writeByteToBuffer((cTime >> 16) & 0xFF)==0) {osDelay(1);}
				crc_buf[4]=(cTime >> 8) & 0xFF;while(writeByteToBuffer((cTime >> 8) & 0xFF)==0) {osDelay(1);}
				crc_buf[5]=cTime & 0xFF;while(writeByteToBuffer(cTime & 0xFF)==0) {osDelay(1);}

				crc_buf[6]=RxHeader.DLC & 0xFF;while(writeByteToBuffer(RxHeader.DLC & 0xFF)==0) {osDelay(1);}
				for(i=0;i<RxHeader.DLC;i++) {
					crc_buf[7+i]=RxData[i];while(writeByteToBuffer(RxData[i])==0) {osDelay(1);}
				}
				crc8 = GetCRC8(crc_buf,7+RxHeader.DLC);
				while(writeByteToBuffer(crc8)==0) {osDelay(1);}

				//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
				/*if((RxHeader.StdId == 0x02)                     &&
				     (RxHeader.RTR == CAN_RTR_DATA)               &&
				     (RxHeader.IDE == CAN_ID_STD)                 &&
				     (RxHeader.DLC == 2)                          &&
				     ((RxData[0]<<8 | RxData[1]) == 0x0405)) {

				}*/
			}
		}
		/*for(i=0;i<528;i++) {fl_buf[i]=0;}
		fl_buf[0] = pageNum >> 8;
		fl_buf[1] = pageNum & 0xFF;
		write_page(pageNum,fl_buf);
		pageNum++;if(pageNum>=8192) pageNum=0;*/
		osDelay(1);
	}
}

