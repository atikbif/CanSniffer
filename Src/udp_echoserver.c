/**
  ******************************************************************************
  * @file    LwIP/LwIP_UDP_Echo_Server/Src/udp_echoserver.c
  * @author  MCD Application Team
  * @brief   UDP echo server
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "udp_echoserver.h"
#include "spi_flash.h"
#include "crc.h"
#include "flash_if.h"

//#include <string.h>

#include "stm32f4xx_hal.h"

#define UDP_SERVER_PORT    7   /* define the UDP local connection port */
#define PAGE_CNT		   8192

#define INCORRECT_PAGE_NUM	1
#define FRAM_IS_BUSY		2

extern RTC_HandleTypeDef hrtc;

static char answer[1024];
static unsigned short pageNum = 0;
static unsigned short reqID = 0;

volatile uint8_t *UniqueID = (uint8_t *)0x1FFF7A10;

static RTC_TimeTypeDef sTime;
static RTC_DateTypeDef sDate;
static unsigned long curTime;

const uint16_t day_offset[12] = {0, 31, 61,92, 122, 153, 184, 214, 245, 275,306, 337};
struct sDateTime
{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t date;
	uint8_t month;
	uint8_t day;
	int16_t year;
} sdt;

void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void inline send_udp_data(struct udp_pcb *upcb,const ip_addr_t *addr,u16_t port,u16_t length);

static unsigned long EncodeDateTime(struct sDateTime *dt)
{
	uint8_t a = dt->month < 3; // а = 1, если месяц январь или февраль
	if((dt->year==2000)&&(a)) {
		if(dt->month==1) return 86400*(dt->date-1) + (int)dt->hour * 3600 + (int)dt->min * 60 + dt->sec;
		else return 86400*(dt->date-1) + (unsigned long)31*86400 + (int)dt->hour * 3600 + (int)dt->min * 60 + dt->sec;
	}else {
		int16_t y = dt->year - a - 2000;  // y = отнимаем от года 1, если а = 1, а так же 2000;
		uint8_t m = dt->month + 12 * a - 3; // аналогия выражения (12 + month - 3) % 12; делаем март нулевым месяцем года.
		return 5184000 + (dt->date - 1 + day_offset[m] + y * 365 + y / 4 - y / 100 + y / 400) * 86400 +
			   (int)dt->hour * 3600 + (int)dt->min * 60 + dt->sec;
	}
}

/*static void DecodeDateTime(uint32_t idt, sDateTime *dt)
{
    sDayTime day;
    ldiv_t century, year_of_century;
    uint32_t day_of_century, a, m;
    DecodeDayTime(idt, &day);
    dt->sec = day.Sec;
    dt->min = day.Min;
    dt->hour = day.Hour;
    century = ldiv(day.Day * 4 + 3, 146097); // Вычисляем кол-во 100-летий
    day_of_century = century.rem / 4; // Остаток дней в столетии
    year_of_century = ldiv(day_of_century * 4 + 3, 1461); // Вычисляем кол-во лет в столетии
    day_of_year = year_of_century.rem / 4; // Остаток дней в году
    m = (5 * day_of_year + 2) / 153; // номер месяца, где март = 0, апрель = 1 и т.д.
    dt->date = day_of_year + 1 - day_offset[m]; // находим день в месяце
    a = m < 10;
    dt->month = m + 3 - 12 * a; // вычисляем месяц в году
    dt->year = 100 * century.quot + year_of_century.quot + 2000 + a; // вычисляем год
    dt->day = (day.Day + 3) % 7; // вычисляем день недели
}*/

void udp_echoserver_init(void)
{
   struct udp_pcb *upcb;
   err_t err;
   
   /* Create a new UDP control block  */
   upcb = udp_new();
   
   if (upcb)
   {
     /* Bind the upcb to the UDP_PORT port */
     /* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
      err = udp_bind(upcb, IP_ADDR_ANY, UDP_SERVER_PORT);
      
      if(err == ERR_OK)
      {

        /* Set a receive callback for the upcb */
        udp_recv(upcb, udp_echoserver_receive_callback, NULL);
      }
      else
      {
        udp_remove(upcb);
      }
   }
}

void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{

  unsigned char *data;
  unsigned short crc;
  unsigned char i=0;

  data = (unsigned char*)(p->payload);
  crc = GetCRC16(data,p->len);
  if(crc==0)
  {
	  reqID = (unsigned short)data[0]<<8;
	  reqID |= data[1];
	  switch(data[2]){
	  	  case 0xA0:
			  answer[0] = 0x01;	// type of device identificator
			  answer[1] = 0xAD;
			  answer[2] = 0x54;
			  answer[3] = 0x98;
			  crc = GetCRC16((unsigned char*)answer,4);
			  answer[4]=crc>>8;
			  answer[5]=crc&0xFF;
			  send_udp_data(upcb, addr, port,6);
			  break;
		  case 0xE8:
			  if((data[3]==4)&&(FLASH_If_Erase_One_Sector(data[3]))) { // correct sector erase
				  answer[0] = data[0];
				  answer[1] = data[1];
				  answer[2] = 0xE1;
				  answer[3] = 0x01;
				  crc = GetCRC16((unsigned char*)answer,4);
				  answer[4]=crc>>8;
				  answer[5]=crc&0xFF;
				  send_udp_data(upcb, addr, port,6);
			  }else {
				  answer[0] = data[0];
				  answer[1] = data[1];
				  answer[2] = 0xE1;
				  answer[3] = 0x00;
				  crc = GetCRC16((unsigned char*)answer,4);
				  answer[4]=crc>>8;
				  answer[5]=crc&0xFF;
				  send_udp_data(upcb, addr, port,6);
			  }
			  break;
		  case 0xEF:
			  SCB->AIRCR = 0x05FA0004;
			  break;
		  case 0xD0:
			  pageNum = (unsigned short)data[3]<<8;
			  pageNum |= data[4];
			  // repeat request ID
			  answer[0] = data[0];
			  answer[1] = data[1];
			  if(pageNum<PAGE_CNT) {
				  if(read_page(pageNum,(unsigned char*)&answer[2])) {
					  crc = GetCRC16((unsigned char*)answer,2 + 528);
					  answer[530]=crc>>8;
					  answer[531]=crc&0xFF;
					  send_udp_data(upcb, addr,port,532);
				  }else {
					  answer[2] = 0xD0;
					  answer[3] = FRAM_IS_BUSY;
					  crc = GetCRC16((unsigned char*)answer,4);
					  answer[4]=crc>>8;
					  answer[5]=crc&0xFF;
					  send_udp_data(upcb, addr, port,6);
				  }

			  }else {
				  answer[2] = 0xD0;
				  answer[3] = INCORRECT_PAGE_NUM;
				  crc = GetCRC16((unsigned char*)answer,4);
				  answer[4]=crc>>8;
				  answer[5]=crc&0xFF;
				  send_udp_data(upcb, addr, port,6);
			  }
			  break;
		  case 0xE0:
			  break;
		  case 0xD1:
			  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

			  sdt.sec = sTime.Seconds;
			  sdt.min = sTime.Minutes;
			  sdt.hour = sTime.Hours;
			  sdt.date = sDate.Date;
			  sdt.day = sDate.WeekDay;
			  sdt.month = sDate.Month;
			  sdt.year = sDate.Year + 2000;
			  curTime = EncodeDateTime(&sdt);

			  // repeat request ID
			  answer[0] = data[0];
			  answer[1] = data[1];
			  answer[2] = 0xD1;
			  answer[3] = (curTime>>24)&0xFF;
			  answer[4] = (curTime>>16)&0xFF;
			  answer[5] = (curTime>>8)&0xFF;
			  answer[6] = curTime & 0xFF;
			  for(i=0;i<12;i++) answer[7+i] = UniqueID[i];
			  answer[19] = 0x01; // version high
			  answer[20] = 0x00; // version low

			  crc = GetCRC16((unsigned char*)answer,7+12+2);
			  answer[21]=crc>>8;
			  answer[22]=crc&0xFF;
			  send_udp_data(upcb, addr, port,23);
			  break;
		  case 0xE1:
			  sTime.Seconds = data[3];
			  sTime.Minutes = data[4];
			  sTime.Hours = data[5];
			  sDate.Date = data[6];
			  sDate.Month = data[7];
			  sDate.Year = data[8];
			  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			  answer[0] = data[0];
			  answer[1] = data[1];
			  answer[2] = 0xE1;
			  crc = GetCRC16((unsigned char*)answer,3);
			  answer[3]=crc>>8;
			  answer[4]=crc&0xFF;
			  send_udp_data(upcb, addr, port,5);
	  }
  }

  /* Free the p buffer */
  pbuf_free(p);
   
}

void send_udp_data(struct udp_pcb *upcb,const ip_addr_t *addr,u16_t port,u16_t length) {
	struct pbuf *p_answer;
	udp_connect(upcb, addr, port);
	p_answer = pbuf_alloc(PBUF_TRANSPORT,length, PBUF_POOL);
	if (p_answer != NULL)
	{
	  pbuf_take(p_answer, answer, length);
	  udp_send(upcb, p_answer);
	  pbuf_free(p_answer);
	}
	udp_disconnect(upcb);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
