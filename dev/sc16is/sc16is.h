#ifndef SC16IS_H
#define SC16IS_H

#ifdef I2C_SC16IS_CONF_ADDR
#define I2C_SC16IS_ADDR     I2C_SC16IS_CONF_ADDR
#else
#define I2C_SC16IS_ADDR     (0x9A) /* A0 & A1 to GND */
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

#define SC16IS_RHR    RG(0x00) /* RX FIFO */
#define SC16IS_THR    RG(0x00) /* TX FIFO */
#define SC16IS_IER    RG(0x01) /* Interrupt enable */
#define SC16IS_IIR    RG(0x02) /* Interrupt Identification */
#define SC16IS_FCR    RG(0x02) /* FIFO control */
#define SC16IS_LCR    RG(0x03) /* Line Control */
#define SC16IS_MCR    RG(0x04) /* Modem Control */
#define SC16IS_LSR    RG(0x05) /* Line Status */
#define SC16IS_MSR    RG(0x06) /* Modem Status */
#define SC16IS_SPR    RG(0x07) /* Scratch Pad */
#define SC16IS_TXLVL    RG(0x08) /* TX FIFO level */
#define SC16IS_RXLVL    RG(0x09) /* RX FIFO level */
#define SC16IS_IODIR    RG(0x0a) /* I/O Direction - only on 75x/76x */
#define SC16IS_IOSTATE    RG(0x0b) /* I/O State - only on 75x/76x */
#define SC16IS_IOINTENA   RG(0x0c) /* I/O Interrupt Enable  - only on 75x/76x */
#define SC16IS_IOCONTROL  RG(0x0e) /* I/O Control  - only on 75x/76x */
#define SC16IS_EFCR   RG(0x0f) /* Extra Features Control */

/* TCR/TLR Register set: Only if ((MCR[2] == 1) && (EFR[4] == 1)) */
#define SC16IS_TCR    RG(0x06) /* Transmit control */
#define SC16IS_TLR    RG(0x07) /* Trigger level */

/* Special Register set: Only if ((LCR[7] == 1) && (LCR != 0xBF)) */
#define SC16IS_DLL    RG(0x00) /* Divisor Latch Low */
#define SC16IS_DLH    RG(0x01) /* Divisor Latch High */

/* Enhanced Register set: Only if (LCR == 0xBF) */
#define SC16IS_EFR    RG(0x02) /* Enhanced Features */
#define SC16IS_XON1   RG(0x04) /* Xon1 word */
#define SC16IS_XON2   RG(0x05) /* Xon2 word */
#define SC16IS_XOFF1    RG(0x06) /* Xoff1 word */
#define SC16IS_XOFF2    RG(0x07) /* Xoff2 word */

/* FCR register bits */
#define SC16IS_FCR_FIFO_BIT (1 << 0) /* Enable FIFO */
#define SC16IS_FCR_RXRESET_BIT  (1 << 1) /* Reset RX FIFO */
#define SC16IS_FCR_TXRESET_BIT  (1 << 2) /* Reset TX FIFO */
#define SC16IS_FCR_RXLVLL_BIT (1 << 6) /* RX Trigger level LSB */
#define SC16IS_FCR_RXLVLH_BIT (1 << 7) /* RX Trigger level MSB */

/* LCR bits */

/* Bitlen:  00 -> 5,  01 -> 6,  10 -> 7,  11 -> 8 bits */
#define SC16IS_LCR_LENGTH0_BIT  (1 << 0)
#define SC16IS_LCR_LENGTH1_BIT  (1 << 1)
#define SC16IS_LCR_STOPLEN_BIT  (1 << 2)
/* stopbits:  0 -> 1, 1 -> 1 1/2 or 2 bits */
#define SC16IS_LCR_PARITY_BIT (1 << 3) /* Parity bit enable */
#define SC16IS_LCR_EVENPARITY_BIT  (1 << 4) /* Even parity bit enable */
#define SC16IS_LCR_FORCEPARITY_BIT  (1 << 5) /* 9-bit multidrop parity */
#define SC16IS_LCR_TXBREAK_BIT  (1 << 6) /* TX break enable */
#define SC16IS_LCR_DLAB_BIT (1 << 7) /* Divisor Latch enable */
#define SC16IS_LCR_WORD_LEN_5 (0x00)
#define SC16IS_LCR_WORD_LEN_6 (0x01)
#define SC16IS_LCR_WORD_LEN_7 (0x02)
#define SC16IS_LCR_WORD_LEN_8 (0x03)
#define SC16IS_LCR_CONF_MODE_A  SC16IS_LCR_DLAB_BIT /* Special reg */
#define SC16IS_LCR_CONF_MODE_B  0xBF                 /* Enhanced reg */

/* MCR  bits */
#define SC16IS_MCR_DTR_BIT      (1 << 0) /* DTR complement only on 75x/76x   */
#define SC16IS_MCR_RTS_BIT  (1 << 1) /* RTS complement */
#define SC16IS_MCR_TCRTLR_BIT (1 << 2) /* TCR/TLR register enable */
#define SC16IS_MCR_LOOP_BIT (1 << 4) /* Enable loopback test mode */
#define SC16IS_MCR_XONANY_BIT (1 << 5) /* Enable Xon Any  write enabled if (EFR[4] == 1) */
#define SC16IS_MCR_IRDA_BIT (1 << 6) /* Enable IrDA mode  write enabled if (EFR[4] == 1) */
#define SC16IS_MCR_CLKSEL_BIT (1 << 7) /* Divide clock by 4  write enabled (EFR[4] == 1)  */

/* EFCR register bits */
#define SC16IS_EFCR_9BIT_MODE_BIT  (1 << 0) /* Enable 9-bit or Multidrop mode (RS485) */
#define SC16IS_EFCR_RXDISABLE_BIT  (1 << 1) /* Disable receiver */
#define SC16IS_EFCR_TXDISABLE_BIT  (1 << 2) /* Disable transmitter */
#define SC16IS_EFCR_AUTO_RS485_BIT (1 << 4) /* Auto RS485 RTS direction */
#define SC16IS_EFCR_RTS_INVERT_BIT (1 << 5) /* RTS output inversion */
#define SC16IS_EFCR_IRDA_MODE_BIT  (1 << 7) /* IrDA mode  0 = rate upto 115.2 kbit/s - Only 750/760
                                             * 1 = rate upto 1.152 Mbit/s
                                             *   - Only 760
                                             */
/* EFR register bits */
#define SC16IS_EFR_AUTORTS_BIT  (1 << 6) /* Auto RTS flow ctrl enable */
#define SC16IS_EFR_AUTOCTS_BIT  (1 << 7) /* Auto CTS flow ctrl enable */
#define SC16IS_EFR_XOFF2_DETECT_BIT (1 << 5) /* Enable Xoff2 detection */
#define SC16IS_EFR_ENABLE_BIT (1 << 4) /* Enable enhanced functions  and writing to IER[7:4],
                                        * FCR[5:4], MCR[7:5]  */
#define SC16IS_EFR_SWFLOW3_BIT  (1 << 3) /* SWFLOW bit 3 */
#define SC16IS_EFR_SWFLOW2_BIT  (1 << 2) /* SWFLOW bit 2
                                          *
                                          * SWFLOW bits 3 & 2 table:
                                          * 00 -> no transmitter flow
                                          *       control
                                          * 01 -> transmitter generates
                                          *       XON2 and XOFF2
                                          * 10 -> transmitter generates
                                          *       XON1 and XOFF1
                                          * 11 -> transmitter generates
                                          *       XON1, XON2, XOFF1 and
                                          *       XOFF2
                                          */
#define SC16IS_EFR_SWFLOW1_BIT  (1 << 1) /* SWFLOW bit 2 */
#define SC16IS_EFR_SWFLOW0_BIT  (1 << 0) /* SWFLOW bit 3
                                          *
                                          * SWFLOW bits 3 & 2 table:
                                          * 00 -> no received flow
                                          *       control
                                          * 01 -> receiver compares
                                          *       XON2 and XOFF2
                                          * 10 -> receiver compares
                                          *       XON1 and XOFF1
                                          * 11 -> receiver compares
                                          *       XON1, XON2, XOFF1 and
                                          *       XOFF2
                                          */

/*
 * TCR register bits
 * TCR trigger levels are available from 0 to 60 characters with a granularity
 * of four.
 * The programmer must program the TCR such that TCR[3:0] > TCR[7:4]. There is
 * no built-in hardware check to make sure this condition is met. Also, the TCR
 * must be programmed with this condition before auto RTS or software flow
 * control is enabled to avoid spurious operation of the device.
 */
#define SC16IS_TCR_RX_HALT(words) ((((words) / 4) & 0x0f) << 0)
#define SC16IS_TCR_RX_RESUME(words) ((((words) / 4) & 0x0f) << 4)

/* LSR bits */
#define SC16IS_LSR_DR_BIT   (1 << 0) /* Receiver data ready */
#define SC16IS_LSR_OE_BIT   (1 << 1) /* Overrun Error */
#define SC16IS_LSR_PE_BIT   (1 << 2) /* Parity Error */
#define SC16IS_LSR_FE_BIT   (1 << 3) /* Frame Error */
#define SC16IS_LSR_BI_BIT   (1 << 4) /* Break Interrupt */
#define SC16IS_LSR_BRK_ERROR_MASK 0x1E     /* BI, FE, PE, OE bits */
#define SC16IS_LSR_THRE_BIT   (1 << 5) /* TX holding register empty */
#define SC16IS_LSR_TEMT_BIT   (1 << 6) /* Transmitter empty */
#define SC16IS_LSR_FIFOE_BIT    (1 << 7) /* Fifo Error */

/* IER register bits */
#define SC16IS_IER_RDI_BIT    (1 << 0) /* Enable RX data interrupt */
#define SC16IS_IER_THRI_BIT   (1 << 1) /* Enable TX holding register
                                        * interrupt */
#define SC16IS_IER_RLSI_BIT   (1 << 2) /* Enable RX line status
                                        * interrupt */
#define SC16IS_IER_MSI_BIT    (1 << 3) /* Enable Modem status
                                        * interrupt */
/* IER register bits - write only if (EFR[4] == 1) */
#define SC16IS_IER_SLEEP_BIT    (1 << 4) /* Enable Sleep mode */
#define SC16IS_IER_XOFFI_BIT    (1 << 5) /* Enable Xoff interrupt */
#define SC16IS_IER_RTSI_BIT   (1 << 6) /* Enable nRTS interrupt */
#define SC16IS_IER_CTSI_BIT   (1 << 7) /* Enable nCTS interrupt */

#endif /* SC16IS_H */
