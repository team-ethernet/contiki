/*
 * Copyright (c) 2017, Copyright Robert Olsson
 * KTH Royal Institute of Technology NSLAB KISTA STOCHOLM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 *
 * Author  : Robert Olsson roolss@kth.se
 * Created : 2017-05-22
 */

/**
 * \file
 *         A simple application showing sc16is I2C UART & GPIO
 *         Example uses avr-rss2 platform
 *         
 */

#include "contiki.h"
#include "sys/etimer.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "i2c.h"
#include "dev/leds.h"
#include "dev/sc16is/sc16is.h"
#include "gprs-a6.h"
#include "tcp-socket-gprs.h"

/*---------------------------------------------------------------------------*/
PROCESS(testapp, "IP/GPRS testapp");
AUTOSTART_PROCESSES(&testapp);

//#define APN "online.telia.se"
#define APN "4g.tele2.se"
#define IPADDR  "192.16.125.232"
//#define IPADDR  "2001:6b0:32:13:4d6b:73cf:d936:2bfe"
#define PORTNO 9999

struct etimer et;

static int err;

#define GPRS_BUFSIZ 512
struct tcp_socket_gprs socket;
uint8_t inbuf[GPRS_BUFSIZ];
uint8_t outbuf[GPRS_BUFSIZ];


static int
tcp_gprs_input(struct tcp_socket_gprs *s,
          void *ptr,
          const uint8_t *input_data_ptr,
          int input_data_len)
{
  printf("Here is tcp_gprs_input: %d @0x%x\n", input_data_len, (unsigned) input_data_ptr);
  return 0;
}

static int connstatus;

/*
 * Handles TCP events from Simple TCP
 */
static void
tcp_gprs_event(struct tcp_socket_gprs *s, void *ptr, tcp_socket_gprs_event_t event)
{
  printf("Here is tcp_gprs_event: %d\n", event);
  switch (event) {
  case TCP_SOCKET_CONNECTED:
    printf("TCP_SOCKET_CONNECTED:\n");
    break;
  case TCP_SOCKET_CLOSED:
    printf("TCP_SOCKET_CLOSED:\n");
    break;
  case TCP_SOCKET_TIMEDOUT:
    printf("TCP_SOCKET_TIMEDOUT:\n");
    break;
  case TCP_SOCKET_ABORTED:
    printf("TCP_SOCKET_ABORTED:\n");
    break;
  case TCP_SOCKET_DATA_SENT:
    printf("TCP_SOCKET_DATA_SENT:\n");
    break;
  }
  connstatus = event;
  process_poll(&testapp);
}

PROCESS_THREAD(testapp, ev, data)
{
  err = 0;
  PROCESS_BEGIN();

  printf("Here is testapp. Wait for gprs\n");

  PROCESS_WAIT_EVENT_UNTIL(ev == a6at_gprs_init);
  printf("Here is testapp proc again\n");

  while (1) {
    tcp_socket_gprs_register(&socket, NULL, inbuf, sizeof(inbuf), outbuf, sizeof(outbuf), tcp_gprs_input, tcp_gprs_event);
    while (1) 
      {
        connstatus = -1;      
        err = tcp_socket_gprs_connect_strhost(&socket, IPADDR, PORTNO);
        PROCESS_YIELD_UNTIL(connstatus != -1);
        if (connstatus == TCP_SOCKET_CONNECTED)
          break;
      }
    printf("CONNECTED\n");
    tcp_socket_gprs_send(&socket, (uint8_t *) "heja data\r\n", sizeof("heja data\r\n")-1);
    PROCESS_YIELD_UNTIL(connstatus == TCP_SOCKET_DATA_SENT || connstatus == TCP_SOCKET_CLOSED || connstatus == TCP_SOCKET_TIMEDOUT);
    if (connstatus != TCP_SOCKET_DATA_SENT)
      goto clear;
    etimer_set(&et, CLOCK_SECOND*30);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    tcp_socket_gprs_close(&socket);
    PROCESS_YIELD_UNTIL(connstatus == TCP_SOCKET_CLOSED || connstatus == TCP_SOCKET_TIMEDOUT);
  clear:
    tcp_socket_gprs_unregister(&socket);
  }
  PROCESS_END();
}
