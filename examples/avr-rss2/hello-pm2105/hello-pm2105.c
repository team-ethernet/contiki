/*
 * Copyright (c) 2015, Copyright Robert Olsson / Radio Sensors AB  
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
 * Author  : Robert Olsson robert@radio-sensors.com
 * Created : 2015-11-22
 */

/**
 * \file
 *         A simple application for airsensor reading on RSS2 mote
 */

#include "contiki.h"
#include "pt.h"
#include "sys/etimer.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "adc.h"
#include "i2c.h"
#include "dev/leds.h"
#include "dev/battery-sensor.h"
#include "dev/temp-sensor.h"
#include "dev/temp_mcu-sensor.h"
#include "dev/light-sensor.h"
#include "dev/pulse-sensor.h"
#include "dev/bme280/bme280-sensor.h"
#include "dev/co2_sa_kxx-sensor.h"
#include "dev/pm2105/pm2105.h"
#include "dev/pm2105/pm2105-sensor.h"
#include "i2c.h"


PROCESS(hello_airsensors_process, "Hello sensor process");
AUTOSTART_PROCESSES(&hello_airsensors_process);

SENSORS(&pm2105_sensor);

static struct etimer et;

unsigned char buf[70];

int now = 0;
uint8_t status, setup = 1;
uint16_t mode, cal;

static struct pt read_values_pt;

PT_THREAD(read_values(struct pt *pt))
{
  PT_BEGIN(pt);  

  if(0 && i2c_probed & I2C_PM2105 ) {
    if (1 || pm2105_sensor.value(PM2105_SENSOR_TIMESTAMP) != 0) {
      printf(" PMS1S=%-d", pm2105_sensor.value(PM2105_SENSOR_PM1));
    }
  }

  if(setup)  {
    setup = 0;
      buf[0] = 0x16;
      buf[1] = 3; /* Cont. Mode */
      buf[1] = 4; /* Time Mode -- Single */
      buf[1] = 5; /* Dynamic Mode */
      buf[1] = 7; /* Warm Mode */

      buf[1] = 4; /* Time Mode -- Single */
      buf[2] = 7;

      if(buf[1] == 3) {
	buf[3] = 0xFF;
	buf[4] = 0xFF;
      }
      buf[3] = 0;
      buf[4] = 60; //0xFF;
      buf[5] = 0; /* Reserved */
      buf[6] = (buf[0]^buf[1]^buf[2]^buf[3]^buf[4]^buf[5]); /* Checksum */
      i2c_write_mem_block(I2C_PM2105_ADDR, buf, 7);
  }

  if(i2c_probed & I2C_PM2105 ) {

    if(now)  {
      buf[0] = 0x16;
      if(now == 1) {  /* Close */
	buf[1] = 1; 
      }
      if(now == 2) {  /* Open */
	buf[1] = 2; 
      }
      buf[2] = 7;
      buf[3] = 0;
      buf[4] = 60; //0xFF;
      buf[5] = 0; /* Reserved */
      buf[6] = (buf[0]^buf[1]^buf[2]^buf[3]^buf[4]^buf[5]); /* Checksum */
      i2c_write_mem_buf(I2C_PM2105_ADDR, buf, 7);
    }

    memset(buf, 0, sizeof(buf));
    i2c_read_mem_buf(I2C_PM2105_ADDR, 0, buf, 32);
    status = buf[2];
    mode = (buf[3] << 8) + buf[4];

    /* Cont. Mode */
    if(mode == 3) {
      if(status == 0x80 ) {
	now = 1;
      }
      if(status == 2 ) {
	now = 1;
      }
      if(status == 1 ) {
	now = 2;
      }
    }

    /* Time Mode */
    if(mode == 4) {
      if(status == 0x80 ) {
	printf("**DR-Close: ");
	now = 1;
      }
      if(status == 1) {
	static struct etimer timer;
	etimer_set(&timer, 30 * CLOCK_SECOND);
	PT_WAIT_UNTIL(pt, etimer_expired(&timer));
	now = 2;
      }
    }

    /* Dynamic Mode */
    if(mode == 5) {
      if(status == 0x80 ) {
	printf("**DR: ");
      }
    }

    /* Warm Mode */
    if(mode == 7) {
      if(status == 0x80 ) {
	printf("** WARM: ");
      }
    }

#if 0
    for(i = 0; i < sizeof(buf) ; i++)  {
      printf(" %02x", buf[i]);
    }
    printf("\n");
#endif
  }
  PT_END(pt);  
}

PROCESS_THREAD(hello_airsensors_process, ev, data)
{
  PROCESS_BEGIN();

  if(i2c_probed & I2C_PM2105 ) {
    //SENSORS_ACTIVATE(pm2105_sensor);
  }
  PT_INIT(&read_values_pt);
  leds_init(); 
 
  etimer_set(&et, CLOCK_SECOND * 5);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    read_values(&read_values_pt);
    pmsframe(buf);
    etimer_reset(&et);
  }
  PROCESS_END();
}

