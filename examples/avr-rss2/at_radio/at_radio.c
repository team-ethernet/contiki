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
#include <string.h>
#include "i2c.h"
#include "dev/leds.h"
#include "dev/sc16is/sc16is.h"

#define GP0 1
#define GP1 2
#define GP2 4
#define G_RESET GP0
#define G_SLEEP  GP1
#define G_PWR_KEY GP2

process_event_t sc16is_input_event;
process_event_t at_match_event;

/*---------------------------------------------------------------------------*/
PROCESS(sc16is_process, "I2C UART/GPIO process");
PROCESS(sc16is_reader, "I2C UART input process");
PROCESS(sc16is_at, "I2C UART AT emitter");
AUTOSTART_PROCESSES(&sc16is_reader, &sc16is_at);
//AUTOSTART_PROCESSES(&sc16is_process);
uint32_t baud;

uint8_t at[] = {'A', 'T', 0xd };
uint8_t atcgatt[] = {'A', 'T', 'C', 'G', 'A', 'T', 'T', '=', '1', 0xd };

int len;
uint8_t buf[120];

static unsigned char *waitstr;
static int waitpos;

static void startwait(char *str) {
  waitstr = (unsigned char *)str;
  waitpos = 0;
}

static unsigned char
*matchwait(unsigned char *bytes) {
  int pos = 0;

  if (waitstr) {
    while (bytes[pos] != 0) {
      if (bytes[pos++] == waitstr[waitpos++]) {
        if (waitstr[waitpos] == 0) {
          /* Found it */
          return waitstr;
        }
      }
      else {
        waitpos = 0;
      }
    }
  }
  return NULL;
}
static void
stopwait() {
  waitstr = NULL;
}

static void
dumpstr(unsigned char *str) {
  unsigned char *s = str;

  printf("    ");
  while (*s) {
    if (*s == '\n')
      printf("\n    ");
    else
      printf("%c", *s);
    s++;
  }
  printf("\n");
}

PROCESS_THREAD(sc16is_reader, ev, data)
{
  
  PROCESS_BEGIN();

  if( i2c_probed & I2C_SC16IS ) {
    sc16is_init();
    baud = 115200;
    sc16is_uart_set_speed(baud);
   
    /* GPIO set output */
    sc16is_gpio_set_dir(G_RESET|G_PWR_KEY|G_SLEEP);
  }
  sc16is_input_event = process_alloc_event();
  at_match_event = process_alloc_event();  

  leds_init();

  /* Fix baudrate  */
  sc16is_tx(at, sizeof(at));

  while(1) {
    PROCESS_PAUSE();
    if( i2c_probed & I2C_SC16IS ) {
      len = sc16is_rx(buf, sizeof(buf));
      if(len) {
        static unsigned char *match;

        buf[len] = 0;
        dumpstr(buf);
#if 1
        if((match = matchwait(buf))) {
          stopwait();
          /* Tell other processes there was a match */
          if (process_post(PROCESS_BROADCAST, at_match_event, match) == PROCESS_ERR_OK) {
            PROCESS_WAIT_EVENT_UNTIL(ev == at_match_event);
          }
        }
#endif
        PROCESS_PAUSE();
      }
    }
  }
  PROCESS_END();
}

static struct etimer et;

static void
sendstr(char *str) {
  printf("%s\n", str);
  sc16is_tx((unsigned char *) str, strlen(str)); 
}

#define ATWAIT(atstr, waitstr, delay)              \
  sendstr(atstr); \
  startwait(waitstr); \
  etimer_set(&et, delay); \
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et) || \
                           ev == at_match_event); \
  if(etimer_expired(&et)) { \
  res = NULL; \
  } \
  else if(ev == at_match_event) { \
    etimer_stop(&et); \
    res = (unsigned char *) data;                \
  }

PROCESS_THREAD(sc16is_at, ev, data)
{
  unsigned char *res;
  char str[80];
  PROCESS_BEGIN();

  etimer_set(&et, CLOCK_SECOND*10);
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

 again:
  ATWAIT("ATE1\r", "OK", CLOCK_SECOND*10); /* Echo on */
  ATWAIT("AT\r", "OK", CLOCK_SECOND*10);

  sprintf(str, "AT+CSTT=\"%s\", \"\", \"\"\r", "online.telia.se");
  ATWAIT(str, "OK", CLOCK_SECOND*5);

  ATWAIT("AT+CGATT=1\r", "OK", CLOCK_SECOND*5);
  ATWAIT("AT+CGATT?\r", "OK", CLOCK_SECOND*5);

  sprintf(str, "AT+CGDCONT=1,\"IP\",%s\r", "mobile.telia.se");
  ATWAIT(str, "OK", CLOCK_SECOND*5);
  ATWAIT("AT+CGACT=1,1\r", "OK", CLOCK_SECOND*30);

  ATWAIT("AT+CIPSTATUS?\r", "OK", CLOCK_SECOND*5);
  ATWAIT("AT+CCID\r", "OK", CLOCK_SECOND*5);
  ATWAIT("AT+CEER\r", "OK", CLOCK_SECOND*5);
  ATWAIT("AT+CSQ\r", "OK", CLOCK_SECOND*5);

  ATWAIT("AT\r", "KOKO", CLOCK_SECOND*30); /* delay */
  goto again;

  PROCESS_END();
}

PROCESS_THREAD(sc16is_process, ev, data)
{
  PROCESS_BEGIN();

  if( i2c_probed & I2C_SC16IS ) {
    sc16is_init();
    baud = 115200;
    sc16is_uart_set_speed(baud);
   
/* GPIO set output */
    sc16is_gpio_set_dir(G_RESET|G_PWR_KEY|G_SLEEP);
 }

  leds_init();

/* Fix baudrate  */
  sc16is_tx(at, sizeof(at));
  etimer_set(&et, CLOCK_SECOND * 10);

while(1) {
  PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
  if( i2c_probed & I2C_SC16IS ) {
    len = sc16is_rx(buf, sizeof(buf));
    if(len) {
      buf[len] = 0;
      printf("%s\n", buf);
    }

      printf("GPIO=0x%02x\n", sc16is_gpio_get());
      if(sc16is_gpio_get() & G_SLEEP) {
	sc16is_gpio_set(0);
	sc16is_tx(at, sizeof(at)); /* Just for demo */
      }
      else
	sc16is_gpio_set(G_SLEEP);
  }
  etimer_reset(&et);
 }
 PROCESS_END();
}

