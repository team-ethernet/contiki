#ifndef SC16IS_H
#define SC16IS_H

#ifdef I2C_SC16IS_CONF_ADDR
#define I2C_SC16IS_ADDR  I2C_SC16IS_CONF_ADDR
#else
#define I2C_SC16IS_ADDR  (0x9A) /* A0 & A1 to GND */
#endif

extern int sc16is_init(void);
extern void sc16is_uart_set_speed(uint32_t baud);
extern uint8_t sc16is_rx(uint8_t *buf, uint8_t maxlen);
extern uint8_t sc16is_tx(uint8_t *buf, int len);
extern uint8_t sc16is_tx_fifo(void);
extern uint8_t sc16is_gpio_get(void);
extern void sc16is_gpio_set(uint8_t set);
extern void sc16is_gpio_set_dir(uint8_t set);
extern uint8_t sc16is_gpio_get_dir(void);
extern void sc16is_gpio_set_irq(uint8_t set);
extern uint8_t sc16is_gpio_get_irq(void);
extern void sc16is_sleep_mode(uint8_t sleep);
extern void sc16is_echo_test(void);

#ifdef SC66IS_CONF_XTAL
#define SC16IS_XTAL SC16IS_CONF_XTAL
#else
#define SC16IS_XTAL 14745600UL
#endif

#define RG(x) (x << 3)
#define SC16IS_RHR    RG(0x00)
#define SC16IS_THR    RG(0x00)
#define SC16IS_IER    RG(0x01)
#define SC16IS_IIR    RG(0x02)
#define SC16IS_FCR    RG(0x02)
#define SC16IS_LCR    RG(0x03)
#define SC16IS_MCR    RG(0x04)
#define SC16IS_LSR    RG(0x05)
#define SC16IS_MSR    RG(0x06)
#define SC16IS_SPR    RG(0x07)
#define SC16IS_TXLVL  RG(0x08)
#define SC16IS_RXLVL  RG(0x09)
#define SC16IS_IODIR  RG(0x0a)
#define SC16IS_IOSTATE  RG(0x0b)
#define SC16IS_IOINTENA RG(0x0c)
#define SC16IS_IOCONTROL RG(0x0e)
#define SC16IS_EFCR   RG(0x0f)
#define SC16IS_TCR    RG(0x06)
#define SC16IS_TLR    RG(0x07)
#define SC16IS_DLL    RG(0x00)
#define SC16IS_DLH    RG(0x01)
#define SC16IS_EFR    RG(0x02)
#define SC16IS_XON1   RG(0x04)
#define SC16IS_XON2   RG(0x05)
#define SC16IS_XOFF1  RG(0x06)
#define SC16IS_XOFF2  RG(0x07)

#define SC16IS_FCR_FIFO_BIT (1 << 0)
#define SC16IS_FCR_RXRESET_BIT (1 << 1)
#define SC16IS_FCR_TXRESET_BIT (1 << 2)
#define SC16IS_FCR_RXLVLL_BIT (1 << 6)
#define SC16IS_FCR_RXLVLH_BIT (1 << 7)
#define SC16IS_LCR_DLAB_BIT (1 << 7)
#define SC16IS_LCR_WORD_LEN_8 (0x03)
#define SC16IS_LCR_CONF_MODE_A  SC16IS_LCR_DLAB_BIT
#define SC16IS_LCR_CONF_MODE_B  0xBF
#define SC16IS_MCR_TCRTLR_BIT (1 << 2)
#define SC16IS_MCR_CLKSEL_BIT (1 << 7)
#define SC16IS_EFCR_RXDISABLE_BIT (1 << 1)
#define SC16IS_EFCR_TXDISABLE_BIT (1 << 2)
#define SC16IS_EFR_AUTORTS_BIT (1 << 6)
#define SC16IS_EFR_AUTOCTS_BIT (1 << 7)
#define SC16IS_EFR_ENABLE_BIT (1 << 4)

#define SC16IS_TCR_RX_HALT(words) ((((words) / 4) & 0x0f) << 0)
#define SC16IS_TCR_RX_RESUME(words) ((((words) / 4) & 0x0f) << 4)

#define SC16IS_LSR_DR_BIT   (1 << 0)
#define SC16IS_IER_SLEEP_BIT  (1 << 4)

#endif /* SC16IS_H */
