/*
 * spi_flash.h
 *
 *  Created on: 30 ���. 2018 �.
 *      Author: Roman
 */

#ifndef SPI_FLASH_H_
#define SPI_FLASH_H_

// ������� ������ ���������� ������ �� ��������� FreeRTOS

unsigned char read_page(unsigned short pageNum, unsigned char *buf);
unsigned char write_page(unsigned short pageNum, unsigned char *buf);
void erase_chip();

#endif /* SPI_FLASH_H_ */
