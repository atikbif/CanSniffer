#include "canDebugMaster.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "datetime.h"

osThreadId canDebugTaskHandle;
extern CAN_HandleTypeDef hcan2;
extern unsigned long cTime;
static unsigned long syncTime = 0;

static CAN_TxHeaderTypeDef   TxHeader;
static uint32_t              TxMailbox=0;
static uint8_t               TxData[8];

struct canRequest {
	unsigned short id;
	unsigned char data[8];
	unsigned short length;
};

const struct canRequest reqs[8] = {
		{0x0001,{0x01,0x01,0x01},3},
		{0x0002,{0x02,0x02},2},
		{0x0003,{0x03,0x03,0x03,0x03,0x03,0x03},6},
		{0x0004,{0x04},1},
		{0x0005,{0x05,0x05,0x05},3},
		{0x0006,{},0},
		{0x0007,{0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07},8},
		{0x0008,{0x08,0x08,0x08,0x08},4},
};

void initCANDebugFilter() {
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

	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
}

void canDebugTask(void const * argument) {
	static unsigned short timeCnt = 0;
	initCANDebugFilter();
	HAL_CAN_Start(&hcan2);
	int num = 0;
	int i = 0;
	for(;;)
	{
		timeCnt++;
		if(timeCnt>=5000) {
			//HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
			timeCnt = 0;
			TxHeader.StdId = 0x0439;
			TxHeader.ExtId = 0;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_STD;
			TxHeader.DLC = 2 + 4;
			TxHeader.TransmitGlobalTime = DISABLE;
			TxData[0] = 0x1F;
			TxData[1] = 0x0E;
			syncTime = cTime;
			TxData[5] = (syncTime >> 24) & 0xFF;
			TxData[4] = (syncTime >> 16) & 0xFF;
			TxData[3] = (syncTime >> 8) & 0xFF;
			TxData[2] = syncTime & 0xFF;
		}else {
			TxHeader.StdId = reqs[num].id;
			TxHeader.ExtId = 0;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_STD;
			TxHeader.DLC = reqs[num].length;
			TxHeader.TransmitGlobalTime = DISABLE;
			for(i=0;i<reqs[num].length;i++) TxData[i] = reqs[num].data[i];
			num++;
			if(num>=8) num=0;
		}
		HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox);
		while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2) != 3) {osDelay(1);}
		//osDelay(20);
		osDelay(1);
	}
}
