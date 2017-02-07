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

#ifndef PMS5003_H
#define PMS5003_H

/* AVR configuration for controlling dust sensor */
#define SET_PMS_DDR  DDRD   /* Data Direction Register: Port B */
#define SET_PMS_PORT PORTD  /* Serial Peripheral Interface */
#define PMS_SET      6      /* PD6: OW2_PIN, Chip Select */

/* How often sensor process runs (secs) */
#define PMS_PROCESS_PERIOD	15
/* How often sensor data is collected (secs) */
#define PMS_SAMPLE_PERIOD       30
/* Warmup time before sensor data can be read (secs) */
#define PMS_STARTUP_INTERVAL    10

#define PMS_CONF_SERIAL_I2C 1
#ifdef PMS_CONF_SERIAL_I2C
#define PMS_SERIAL_I2C PMS_CONF_SERIAL_I2C
#else
#define PMS_SERIAL_I2C 1
#endif
#define PMS_CONF_SERIAL_UART 1
#ifdef PMS_CONF_SERIAL_UART
#define PMS_SERIAL_UART PMS_CONF_SERIAL_UART
#else
#define PMS_SERIAL_UART 1
#endif

/* Event to signal presense of new data */
process_event_t pms5003_event;

void pms5003_init();
void pms5003_off();

uint16_t pms5003_pm1();
uint16_t pms5003_pm2_5();
uint16_t pms5003_pm10();
uint16_t pms5003_pm1_atm();
uint16_t pms5003_pm2_5_atm();
uint16_t pms5003_pm10_atm();

#endif /* PMS5003_H */
