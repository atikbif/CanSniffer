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

#include <string.h>

#define UDP_SERVER_PORT    7   /* define the UDP local connection port */
#define PAGE_CNT		   8192

#define INCORRECT_PAGE_NUM	1
#define FRAM_IS_BUSY		2

static char answer[1024];
static unsigned short pageNum = 0;
static unsigned short reqID = 0;


void udp_echoserver_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void inline send_udp_data(struct udp_pcb *upcb,const ip_addr_t *addr,u16_t port,u16_t length);


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

  data = (unsigned char*)(p->payload);
  crc = GetCRC16(data,p->len);
  if(crc==0)
  {
	  reqID = (unsigned short)data[0]<<8;
	  reqID |= data[1];
	  switch(data[2]){
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
