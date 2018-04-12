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
 * Created : 2018-04-08
 */

/**
 * \file
 *         A test program for sc16is I2C UART & GPIO
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
#include "sc16is-common.h"

PROCESS(blink, "Test board GPIO");
AUTOSTART_PROCESSES(&blink);

static struct etimer et;

int on;
uint8_t at[] = {'A', 'T', 0xd };

int
module_init(uint32_t baud)
{
  if( i2c_probed & I2C_SC16IS ) {

    sc16is_init();
    sc16is_gpio_set_dir(G_RESET|G_PWR|G_U_5V_CTRL|G_SET|G_LED_YELLOW|G_LED_RED|G_GPIO7);
    sc16is_gpio_set(G_LED_RED);
    sc16is_uart_set_speed(baud);
    //sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_FCR, SC16IS_FCR_FIFO_BIT);
    sc16is_tx(at, sizeof(at));
    return 1;
  }
  return 0;
}

PROCESS_THREAD(blink, ev, data)
{
  unsigned char s;
  PROCESS_BEGIN();
  module_init(115200);

  while (1) {
    etimer_set(&et, CLOCK_SECOND*4);
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

    s = sc16is_gpio_get();
    toggle_bit(&s, G_LED_RED);
    toggle_bit(&s, G_LED_YELLOW);
    toggle_bit(&s, G_GPIO7);
    toggle_bit(&s, G_PWR);
    toggle_bit(&s, G_SET);

    /* Warning on connect not to exceed UART Max 4.5/5.5 */
    /* TEST 5V */
    set_board_5v(on++ & 0x1);

    /* Needs pull-up fo test */
    toggle_bit(&s, G_RESET);
    sc16is_gpio_set(s);
  }
  PROCESS_END();
}
