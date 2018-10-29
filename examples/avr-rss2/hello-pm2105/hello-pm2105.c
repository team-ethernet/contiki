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
#include "sys/etimer.h"
#include <stdio.h>
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


void
i2c_write_mem_block(uint8_t addr, uint8_t buf[], uint8_t bytes)
{
  uint8_t i = 0;
  i2c_start(addr | I2C_WRITE);
  i2c_readAck();
  for(i = 0; i < bytes; i++) {
      i2c_write(buf[i]);
      i2c_readAck();
  }
  i2c_stop();
}

void
i2c_read_mem_my(uint8_t addr, uint8_t reg, uint8_t buf[], uint8_t bytes)
{
  uint8_t i = 0;
  i2c_start(addr | I2C_READ);
  for(i = 0; i < bytes; i++) {
    if(i == bytes - 1) {
      buf[i] = i2c_readNak();
    } else {
      buf[i] = i2c_readAck();
    }
  }
  i2c_stop();
}

static void
read_values(void)
{
  int i;
  if(0 && i2c_probed & I2C_PM2105 ) {
    if (1 || pm2105_sensor.value(PM2105_SENSOR_TIMESTAMP) != 0) {
      printf(" PMS1S=%-d", pm2105_sensor.value(PM2105_SENSOR_PM1));
    }
  }

  if(i2c_probed & I2C_PM2105 ) {
    /* Set measuring open */
    if(1)  {
      buf[0] = 0x16;
      buf[1] = 0x07;
      buf[2] = 0x07;
      buf[3] = 0; //0xFF;
      buf[4] = 20; //0xFF;
      buf[5] = 0x00; /* Reserved */
      buf[6] = (buf[0]^buf[1]^buf[2]^buf[3]^buf[4]^buf[5]); /* Checksum */
      i2c_write_mem_block(I2C_PM2105_ADDR, buf, 7);
    }
#include <string.h>
    memset(buf, 0, sizeof(buf));
    i2c_read_mem_my(I2C_PM2105_ADDR, 0, buf, 32);
#if 0
    for(i = 0; i < sizeof(buf) ; i++)  {
      printf(" %02x", buf[i]);
    }
    printf("\n");
#endif
  }
}

PROCESS_THREAD(hello_airsensors_process, ev, data)
{
  PROCESS_BEGIN();


  if(i2c_probed & I2C_PM2105 ) {
    //SENSORS_ACTIVATE(pm2105_sensor);
  }

  leds_init(); 
 
    etimer_set(&et, CLOCK_SECOND * 5);
  while(1) {
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
    read_values();

    pmsframe(buf);
    etimer_reset(&et);
  }

  PROCESS_END();
}

