#include "contiki.h"
#include <stdio.h>
#include <string.h>
#include <dev/i2c.h>
#include "sc16is.h"
#include "lib/sensors.h"

int debug = 1;

unsigned int 
sc16is_gpio_in(void)
{
  uint8_t val;  
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_IOSTATE<<3, &val, 1);
  return val;
}

void
sc16is_tx(uint8_t *buf, int len)
{
  int i;
  for(i= 0; i <len; i++) {
    i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_THR<<3, buf[i]);
  }
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_THR<<3, '\n');
}

void
sc16is_echo_test(void)
{
  uint8_t val;

  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_RHR<<3, &val, 1);  

  for( ; val; ) {
    i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_THR<<3, val);
    i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_RHR<<3, &val, 1);  
  }
}

void
uart_speed(uint32_t baud)
{
  uint32_t div ;
  uint8_t lcr, val, prescale = 0;


  div = 14745600/(uint32_t) baud;
  div = div/16;

  if(div > 0xFFFF) {
    printf("Doing UART div\n");
    prescale = SC16IS_MCR_CLKSEL_BIT;
    div = div>>4;
  }

  if(debug) 
    printf("prescale=%d div=%lu %lu %lu\n", prescale, div, div/256, div % 256);

  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LCR<<3, &lcr, 1);

  /* Open the LCR divisors for configuration */
  val =  SC16IS_LCR_CONF_MODE_B;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR<<3, val);
  
  /* Enaable enchancfed features */
  val =  SC16IS_EFR_ENABLE_BIT;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_EFR<<3, val);

  /* LCR to Normal mode */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR<<3, lcr);

  /* Prescaler */
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_MCR<<3, &val, 1);
  val |= prescale;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_MCR<<3, val);

  /* Open LCR for config */
  val =  SC16IS_LCR_CONF_MODE_A;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR<<3, val);

  /* Divisor */
  
  val = div/256;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_DLH<<3, val);
  
  val = div % 256;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_DLL<<3, val);

  /* Restore mode */
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR<<3, lcr);
}

int
sc16is_init(uint8_t mode)
{
  uint8_t val;  
  char buf[4]="hej";

  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LCR<<3, &val, 1);
  printf("LCR=0x%02X\n", val);

  /* Reset FIFOs*/
  val = SC16IS_FCR_RXRESET_BIT | SC16IS_FCR_TXRESET_BIT;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_FCR<<3, val);

  clock_delay_usec(5);

  val = SC16IS_FCR_FIFO_BIT;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_FCR<<3, val);

  /* Enable EFR */
  val = SC16IS_LCR_CONF_MODE_B;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR<<3, val);

  /* Enaable enchancfed features */
  val =  SC16IS_EFR_ENABLE_BIT;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_EFR<<3, val);


  /* Enable TCL/TLR */
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_MCR<<3, &val, 1);
  val |= SC16IS_MCR_TCRTLR_BIT;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_MCR<<3, val);

#if 1
  /* Flow control levels */
  val = SC16IS_TCR_RX_HALT(48) | SC16IS_TCR_RX_RESUME(24);
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_TCR<<3, val);
#endif


  /* Initialize  UART */
  val = SC16IS_LCR_WORD_LEN_8;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_LCR<<3, val);
  
#if 0
  /* Enable RX & TX FIFO */
  val = SC16IS_EFCR_RXDISABLE_BIT | SC16IS_EFCR_TXDISABLE_BIT;
  i2c_write_mem(I2C_SC16IS_ADDR, SC16IS_EFCR<<3, val);
#endif  

  //CHANGE INTERRUPTS

  //sc16is7xx_port_write(port, SC16IS7XX_FCR_REG, SC16IS7XX_FCR_FIFO_BIT);

  //i2c_write_mem(SC16IS_ADDR, uint8_t reg, uint8_t value)
  //i2c_read_mem(SC16IS_ADDR, uint8_t reg, uint8_t buf[], uint8_t bytes)

  uart_speed(115200);
   return 1;
}

void
sc16is_status(void)
{
  uint8_t val;  
  char buf[4]="hej";

  printf("GPIO IN=0x%02X", sc16is_gpio_in());
  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_LCR<<3, &val, 1);
  printf(" LCR=0x%02X", val);

  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_MCR<<3, &val, 1);
  printf(" MCR=0x%02X", val);

  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_IIR<<3, &val, 1);
  printf(" IIR=0x%02X", val);

  i2c_read_mem(I2C_SC16IS_ADDR, SC16IS_RHR<<3, &val, 1);
  printf(" RHR=0x%02X %c", val, val);

  printf("\n");
  //sc16is_tx(&buf, 4);
  sc16is_echo_test();

}
      
