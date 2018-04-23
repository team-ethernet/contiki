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
 * Implementation for NXP SC16IS7XX chip for bridge I2C/SPI to USRT/GPIO
 * I2C support only
 * REF: NXP Datasheet SC16IS740/750/760  Rev. 06 - 13 May 2008
 *
 * Author  : Robert Olsson roolss@kth.se
 * Created : 2017-05-22
 */

#include "contiki.h"
#include <stdio.h>
#include <string.h>
#include "sc16is.h"
#include "sc16is-arch.h"

uint32_t sc16is_xtal = SC16IS_XTAL;

uint8_t
sc16is_gpio_get(void)
{
  uint8_t val;
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_IOSTATE, &val, 1);
  return val;
}
void
sc16is_gpio_set(uint8_t set)
{
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_IOSTATE, set);
  return;
}
void
sc16is_gpio_set_dir(uint8_t set)
{

  /* 0 input */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_IODIR, set);
  return;
}
uint8_t
sc16is_gpio_get_dir(void)
{
  uint8_t val;
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_IODIR, &val, 1);
  return val;
}
void
sc16is_gpio_set_irq(uint8_t set)
{
  /* 1 gives irq */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_IOINTENA, set);
  return;
}
uint8_t
sc16is_gpio_get_irq(void)
{
  uint8_t val;
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_IOINTENA, &val, 1);
  return val;
}
uint8_t
sc16is_tx(uint8_t *buf, int len)
{
  uint8_t i, maxtx;

  /* Read TX FIFO depth */
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_TXLVL, &maxtx, 1);

  for(i = 0; i < len; i++) {
    if(i < maxtx) {
      sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_THR, buf[i]);
    } else {
      break;
    }
  }
  return i;
}
uint8_t
sc16is_rx(uint8_t *buf, uint8_t maxlen)
{
  uint8_t i;
  uint8_t lsr; /* rx */

  /* sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_RXLVL, &rx, 1); */
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LSR, &lsr, 1);

  for(i = 0; lsr &SC16IS_LSR_DR_BIT; i++) {
    /* for(i=0; rx != 0; i++) { */
    if(i == maxlen) {
      break;
    }
    sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_RHR, &buf[i], 1);
    sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LSR, &lsr, 1);
    /* sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_RXLVL, &rx, 1); */
  }
  return i;
}
void
sc16is_echo_test(void)
{
  uint8_t len, buf[20];

  len = sc16is_rx(buf, sizeof(buf));
  len = sc16is_tx(buf, len);
}
void
sc16is_uart_set_speed(uint32_t baud)
{
  uint32_t div;
  uint8_t lcr, val, prescale = 0;

  div = sc16is_xtal / (uint32_t)baud;
  div = div / 16;

  if(div > 0xFFFF) {
    prescale = SC16IS_MCR_CLKSEL_BIT;
    div = div >> 4;
  }

  /* Sleep mode should not be used when sett DLL/DLH registers */
  sc16is_sleep_mode(0);

  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LCR, &lcr, 1);
  /* Open the LCR divisors for configuration */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR, SC16IS_LCR_CONF_MODE_B);
  /* Enaable enchancfed features */
#ifdef HW_FLOW_NONE
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_EFR, SC16IS_EFR_ENABLE_BIT);
#else
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_EFR, SC16IS_EFR_ENABLE_BIT | SC16IS_EFR_AUTORTS_BIT | SC16IS_EFR_AUTOCTS_BIT);
#endif
  /* LCR to Normal mode */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR, lcr);
  /* Prescaler */
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_MCR, &val, 1);
  val |= prescale;
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_MCR, val);
  /* Open LCR for config */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR, SC16IS_LCR_CONF_MODE_A);
  /* Divisor and reminder */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_DLH, div >> 8);
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_DLL, (div & 0xFF));
  /* Restore mode */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR, lcr);

  /* Sleep mode should not be used when sett DLL/DLH registers */
  sc16is_sleep_mode(1);
}
uint8_t
sc16is_tx_fifo(void)
{
  uint8_t maxtx;

  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_TXLVL, &maxtx, 1);
  return maxtx;
#if 0
  uint8_t lvl, lsr;
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_TXLVL, &lvl, 1);
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LSR, &lsr, 1);
  return (lsr & SC16IS_LSR_THRE_BIT) && !lvl;
#endif
}
void
sc16is_sleep_mode(uint8_t sleep)
{
  uint8_t val;

  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_IER, &val, 1);
  if(sleep) {
    val |= SC16IS_IER_SLEEP_BIT;
  } else {
    val &= ~SC16IS_IER_SLEEP_BIT;
  }

  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_IER, val);
}
int
sc16is_init(void)
{
  uint8_t val;

  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_IOCONTROL, (1 << 3)); /* Software reset */
  clock_delay_usec(10);

  /*  We use chip FIFO mode  */
  /* Reset FIFOs */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_FCR, (SC16IS_FCR_RXRESET_BIT | SC16IS_FCR_TXRESET_BIT));
  clock_delay_usec(5);
  /* Enable FIFO */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_FCR, SC16IS_FCR_FIFO_BIT);
  /* Enable EFR */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR, SC16IS_LCR_CONF_MODE_B);
  /* Enaable enchancfed features */
#ifdef HW_FLOW_NONE
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_EFR, SC16IS_EFR_ENABLE_BIT);
#else
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_EFR, SC16IS_EFR_ENABLE_BIT | SC16IS_EFR_AUTORTS_BIT | SC16IS_EFR_AUTOCTS_BIT);
#endif
  /* Enable TCL/TLR */
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_MCR, &val, 1);
  val |= SC16IS_MCR_TCRTLR_BIT;
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_MCR, val);
  /* Flow control levels */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_TCR, (SC16IS_TCR_RX_HALT(48) | SC16IS_TCR_RX_RESUME(24)));
  /* Initialize UART for 8 bit */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR, SC16IS_LCR_WORD_LEN_8);
  /* Enable RX & TX FIFO -- Clear disable bits*/
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_EFCR, &val, 1);
  val &= ~(SC16IS_EFCR_RXDISABLE_BIT | SC16IS_EFCR_TXDISABLE_BIT);
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_EFCR, val);
#ifdef SC66IS_CONF_INTERRUPT
  /* Enable RX, TX, CTS change interrupts */
  sc16is_arch_i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_IER, (SC16IS_IER_RDI_BIT | SC16IS_IER_THRI_BIT | SC16IS_IER_CTSI_BIT));
#endif
  sc16is_gpio_set(0);
  return 1;
}
void
sc16is_debug_register(void)
{
  uint8_t val;

  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_TXLVL, &val, 1);
  printf("TXFIFO=%2d", val);
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_RXLVL, &val, 1);
  printf(" RXFIFO=%2d", val);
  printf(" GPIO=0x%02X", sc16is_gpio_get());
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LCR, &val, 1);
  printf(" LCR=0x%02X", val);
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_MCR, &val, 1);
  printf(" MCR=0x%02X", val);
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_IER, &val, 1);
  printf(" IER=0x%02X", val);
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_IIR, &val, 1);
  printf(" IIR=0x%02X", val);
  sc16is_arch_i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LSR, &val, 1);
  printf(" LSR=0x%02X %c\n", val, val);
}
