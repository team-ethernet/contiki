/* Robert Olsson 2017-05-21 KTH */

#include "contiki.h"
#include <stdio.h>
#include <string.h>
#include <dev/i2c.h>
#include "sc16is.h"

static int debug = 1;
uint32_t sc16is_xtal = 14745600ul;

uint8_t
sc16is_gpio_get(void)
{
  uint8_t val;
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_IOSTATE, &val, 1);
  return val;
}
void
sc16is_gpio_set(uint8_t set)
{
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_IOSTATE, set);
  return;
}
uint8_t
sc16is_tx(uint8_t *buf, int len)
{
  uint8_t i, maxtx;

  /* Read TX FIFO depth */
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_TXLVL, &maxtx, 1);

  for(i = 0; i < len; i++) {
    if(i < maxtx) {
      i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_THR, buf[i]);
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

  /* i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_RXLVL, &rx, 1); */
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LSR, &lsr, 1);

  for(i = 0; lsr &SC16IS_LSR_DR_BIT; i++) {
    /* for(i=0; rx != 0; i++) { */
    if(i == maxlen) {
      break;
    }
    i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_RHR, &buf[i], 1);
    i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LSR, &lsr, 1);
    /* i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_RXLVL, &rx, 1); */
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
static void
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

  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LCR, &lcr, 1);
  /* Open the LCR divisors for configuration */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR, SC16IS_LCR_CONF_MODE_B);
  /* Enaable enchancfed features */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_EFR, SC16IS_EFR_ENABLE_BIT);
  /* LCR to Normal mode */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR, lcr);
  /* Prescaler */
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_MCR, &val, 1);
  val |= prescale;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_MCR, val);
  /* Open LCR for config */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR, SC16IS_LCR_CONF_MODE_A);
  /* Divisor and reminder */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_DLH, div >> 8);
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_DLL, (div & 0xFF));
  /* Restore mode */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR, lcr);
}
static uint8_t
tx_empty(void)
{
  uint8_t lvl, lsr;

  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_TXLVL, &lvl, 1);
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LSR, &lsr, 1);
  return (lsr & SC16IS_LSR_THRE_BIT) && !lvl;
}
int
sc16is_init(void)
{
  uint8_t val;

  /*  We use chip FIFO mode  */
  /* Reset FIFOs */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_FCR, (SC16IS_FCR_RXRESET_BIT | SC16IS_FCR_TXRESET_BIT));
  clock_delay_usec(5);
  /* Enable FIFO */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_FCR, SC16IS_FCR_FIFO_BIT);
  /* Enable EFR */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR, SC16IS_LCR_CONF_MODE_B);
  /* Enaable enchancfed features */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_EFR, SC16IS_EFR_ENABLE_BIT);
  /* Enable TCL/TLR */
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_MCR, &val, 1);
  val |= SC16IS_MCR_TCRTLR_BIT;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_MCR, val);
  /* Flow control levels */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_TCR, (SC16IS_TCR_RX_HALT(48) | SC16IS_TCR_RX_RESUME(24)));
  /* Initialize UART for 8 bit */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR, SC16IS_LCR_WORD_LEN_8);
  /* Enable RX & TX FIFO -- Clear disable bits*/
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_EFCR, &val, 1);
  val &= ~(SC16IS_EFCR_RXDISABLE_BIT | SC16IS_EFCR_TXDISABLE_BIT);
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_EFCR, val);
  /* Enable RX, TX, CTS change interrupts */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_IER, (SC16IS_IER_RDI_BIT | SC16IS_IER_THRI_BIT | SC16IS_IER_CTSI_BIT));
  sc16is_uart_set_speed(115200);
  return 1;
}
void
sc16is_debug_register(void)
{
  uint8_t val;

  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_TXLVL, &val, 1);
  printf("TXFIFO=%2d", val);
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_RXLVL, &val, 1);
  printf(" RXFIFO=%2d", val);
  printf(" GPIO=0x%02X", sc16is_gpio_get());
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LCR, &val, 1);
  printf(" LCR=0x%02X", val);
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_MCR, &val, 1);
  printf(" MCR=0x%02X", val);
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_IER, &val, 1);
  printf(" IER=0x%02X", val);
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_IIR, &val, 1);
  printf(" IIR=0x%02X", val);
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LSR, &val, 1);
  printf(" LSR=0x%02X %c\n", val, val);
}
