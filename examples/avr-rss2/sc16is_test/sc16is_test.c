/*
 * Copyright (c) 2015, Copyright Robert Olsson KTH Stockholm
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
#include "i2c.h"
#include "dev/leds.h"
#include "dev/sc16is/sc16is.h"
/*---------------------------------------------------------------------------*/
PROCESS(sc16is_process, "I2C UART/GPIO process");
AUTOSTART_PROCESSES(&sc16is_process);

static struct etimer et;
int len;

/* PM preamble bytes */
#define PRE1 0x42
#define PRE2 0x4d
/* Valid values for body length field */
#define PMSMINBODYLEN 20
#define PMSMAXBODYLEN 28
/* Buffer holds frame body plus preamble (two bytes)
 * and length field (two bytes) */
#define PMSBUFFER (PMSMAXBODYLEN + 4)
uint8_t buf[PMSBUFFER];

/* Frame assembly statistics */
static uint32_t invalid_frames, valid_frames;

/* Sensor configured on? */
static uint8_t configured_on = 0;
/* When sensor entered current power save mode, in clock_seconds()*/
static unsigned long when_mode;

/* Last readings of sensor data */
struct {
  uint16_t s1, s2_5, s10;
  uint16_t a1, a2_5, a10;
  uint16_t db0_3, db0_5, db1, db2_5, db5, db10;
  unsigned long ts;
} pm;

/**
 * Validate frame by checking preamble, length field and checksum.
 * Return 0 if invalid frame, otherwise 1.
 */
static int
check_pmsframe(uint8_t *buf)
{
  int sum, pmssum;
  int i;
  int len;

  if(buf[0] != PRE1 || buf[1] != PRE2) {
    return 0;
  }
  /* len is length of frame not including preamble and checksum */
  len = (buf[2] << 8) + buf[3];
  if(len < PMSMINBODYLEN || len > PMSMAXBODYLEN) {
    return 0;
  }
  /* Sum data bytewise, including preamble but excluding checksum */
  sum = 0;
  for(i = 0; i < len + 2; i++) {
    sum += buf[i];
  }
  /* Compare with received checksum last in frame*/
  pmssum = (buf[len + 2] << 8) + buf[len + 3];
  return pmssum == sum;
}

static void
printpm()
{
  printf("%5d", pm.ts);
  printf(" valid=%-lu/%-lu",
         valid_frames, invalid_frames);

  printf(" PM STD %4d %4d %4d", pm.s1, pm.s2_5, pm.s10);
  printf(" ATM %4d  %4d  %4d ",
         pm.a1, pm.a2_5, pm.a10);
  printf(" DB %4d %4d %4d %4d %4d %4d", pm.db0_3, pm.db0_5, pm.db1, pm.db2_5, pm.db5, pm.db10);
  printf("\n");
}

 /* Frame received from PMS sensor. Validate and update sensor data.
  * Return 1 if valid frame, otherwise 0
 */
static int
pmsframe(uint8_t *buf)
{
  if(check_pmsframe(buf)) {
    pm.ts = clock_seconds();
    valid_frames++;
    /* Update sensor readings */
    pm.s1 = (buf[4] << 8) | buf[5];
    pm.s2_5 = (buf[6] << 8) | buf[7];
    pm.s10 = (buf[8] << 8) | buf[9];
    pm.a1 = (buf[10] << 8) | buf[11];
    pm.a2_5 = (buf[12] << 8) | buf[13];
    pm.a10 = (buf[14] << 8) | buf[15];
    pm.db0_3 = (buf[16] << 8) | buf[17];
    pm.db0_5 = (buf[18] << 8) | buf[19];
    pm.db1 = (buf[20] << 8) | buf[21];
    pm.db2_5 = (buf[22] << 8) | buf[23];
    pm.db5 = (buf[24] << 8) | buf[25];
    pm.db10 = (buf[26] << 8) | buf[27];
#ifdef DEBUG
    printpm();
#endif /* DEBUG */
    return 1;
  } else {
    invalid_frames++;
#ifdef DEBUG
    printpm();
#endif /* DEBUG */
    return 0;
  }
}

PROCESS_THREAD(sc16is_process, ev, data)
{
  PROCESS_BEGIN();

  if( i2c_probed & I2C_SC16IS ) {
    sc16is_init();
    sc16is_uart_set_speed(9600);
  }

  leds_init();

  etimer_set(&et, CLOCK_SECOND * 1);

  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    if( i2c_probed & I2C_SC16IS ) {
	  len = sc16is_rx(buf, sizeof(buf));
	  buf[len] = 0;
	  if( check_pmsframe(buf) ) {
	    pmsframe(buf);
	    printpm();
	  }
    }
    etimer_reset(&et);
  }
  PROCESS_END();
}

