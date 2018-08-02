/*
 * spi_flash.c
 *
 *  Created on: 30 июл. 2018 г.
 *      Author: Roman
 */


#include "spi_flash.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#define WAIT_TIME_MS	20

extern SPI_HandleTypeDef hspi3;


volatile unsigned char _Sys_SPI_Buzy=0;

void erase_chip() {
	unsigned char Data;
	// ожидание завершения работы с Flash другими процессами
	volatile portTickType startTick,currentTick;
	startTick = xTaskGetTickCount();
	while(_Sys_SPI_Buzy){currentTick = xTaskGetTickCount();if(currentTick - startTick>=WAIT_TIME_MS) return;osDelay(1);}
	// захват Flash текущим процессом
	portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=1;
	portENABLE_INTERRUPTS();

	// непосредственно чтение
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

	Data = 0xC7;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	Data = 0x94;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	Data = 0x80;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	Data = 0x9A;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

	do{
		osDelay(1);
		Data = 0xD7;
		HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
		HAL_SPI_Receive(&hspi3,&Data,1,100);
	}while((Data & 0x80)!=0x80);

	// освобождение Flash
	portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=0;
	portENABLE_INTERRUPTS();
}

unsigned char read_page(unsigned short pageNum, unsigned char *buf) {
	unsigned char Data;

	// ожидание завершения работы с Flash другими процессами
	volatile portTickType startTick,currentTick;
	startTick = xTaskGetTickCount();
	while(_Sys_SPI_Buzy){currentTick = xTaskGetTickCount();if(currentTick - startTick>=WAIT_TIME_MS) return 0;osDelay(1);}
	// захват Flash текущим процессом
	portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=1;
	portENABLE_INTERRUPTS();

	// непосредственно чтение
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

	Data = 0xD2;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	Data = pageNum >> 6;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	Data = pageNum << 2;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	Data = 0x00;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);

	Data = 0x00;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);

	HAL_SPI_Receive(&hspi3,buf,528,100);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

	// освобождение Flash
	portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=0;
	portENABLE_INTERRUPTS();
	return 1;
}

unsigned char write_page(unsigned short pageNum, unsigned char *buf) {
	unsigned char Data;

	// ожидание завершения работы с Flash другими процессами
	volatile portTickType startTick,currentTick;
	startTick = xTaskGetTickCount();
	while(_Sys_SPI_Buzy){currentTick = xTaskGetTickCount();if(currentTick - startTick>=WAIT_TIME_MS) return 0;osDelay(1);}
	// захват Flash текущим процессом
	portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=1;
	portENABLE_INTERRUPTS();

	// непосредственно запись

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

	Data = 0x82;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	Data = pageNum >> 6;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	Data = pageNum << 2;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
	Data = 0x00;
	HAL_SPI_Transmit(&hspi3, &Data, 1, 100);

	HAL_SPI_Transmit(&hspi3,buf,528,100);

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

	do{
		osDelay(1);
		Data = 0xD7;
		HAL_SPI_Transmit(&hspi3, &Data, 1, 100);
		HAL_SPI_Receive(&hspi3,&Data,1,100);
	}while((Data & 0x80)!=0x80);

	// освобождение Flash
	portDISABLE_INTERRUPTS();
	_Sys_SPI_Buzy=0;
	portENABLE_INTERRUPTS();
	return 1;
}

