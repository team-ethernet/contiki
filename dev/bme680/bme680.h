/*
 * Copyright (c) 2015, Copyright Robert Olsson
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
 * Author  : Robert Olsson rolss@kth.se/robert@radio-sensors.com
 *
 * Created : 2016-09-14
 */

/**
 * \file
 *         Definitions for the Bosch BME680 based on datasheet Rev 1.0
 */

#ifndef BME680_H
#define BME680_H

uint8_t bme680_init(void);
void bme680_read(void);

#ifdef BME680_CONF_ADDR
#define BME680_ADDR         BME680_CONF_ADDR
#else
#define BME680_ADDR              (0x77 << 1) /* Alternative 0x76 */
#endif

#define BME680_COEFF_ADDR1_LEN    25
#define BME680_COEFF_ADDR2_LEN    16
#define BME680_FIELD_ADDR_OFFSET  17

#define BME680_ADDR_RES_HEAT_VAL_ADDR 0x00
#define BME680_ADDR_RES_HEAT_RANGE_ADDR 0x02
#define BME680_ADDR_RANGE_SW_ERR_ADDR 0x04
#define BME680_ADDR_SENS_CONF_START 0x5A
#define BME680_ADDR_GAS_CONF_START  0x64
#define BME680_FIELD0_ADDR    0x1d
#define BME680_RES_HEAT0_ADDR   0x5a
#define BME680_GAS_WAIT0_ADDR   0x64

#define BME680_GAS_RANGE_MSK          0x0f

#define BME680_CONF_HEAT_CTRL_ADDR    0x70
#define BME680_CONF_ODR_RUN_GAS_NBC_ADDR  0x71
#define BME680_CONF_OS_H_ADDR     0x72
#define BME680_MEM_PAGE_ADDR      0xf3
#define BME680_CONF_T_P_MODE_ADDR   0x74
#define BME680_CONF_ODR_FILT_ADDR   0x75
#define BME680_COEFF_ADDR1      0x89
#define BME680_COEFF_ADDR2  0xe1

#define BME680_RESET_TIME                10

#define BME680_STATUS                   0x73
#define BME680_RESET                    0xE0
#define BME680_CONFIG                   0x75
#define BME680_CNTL_MEAS                0x74
#define BME680_CNTL_HUM                 0x72
#define BME680_GAS_1                    0x71
#define BME680_GAS_0                    0x70
#define BME680_GAS_WAIT_9               0x6D
#define BME680_GAS_WAIT_0               0x64
#define BME680_RES_HEAT_9               0x63
#define BME680_RES_HEAT_0               0x5A
#define BME680_IDAC_HEAT_9              0x59
#define BME680_IDAC_HEAT_0              0x50
#define BME680_R_LSB                    0x2B
#define BME680_R_MSB                    0x2A

#define BME680_HUM_LSB                  0x26
#define BME680_HUM_MSB                  0x25
#define BME680_TEMP_XLSB                0x24
#define BME680_TEMP_LSB                 0x23
#define BME680_TEMP_MSB                 0x22
#define BME680_PRES_XLSB                0x21
#define BME680_PRES_LSB                 0x20
#define BME680_PRES_MSB                 0x1F
#define BME680_MEAS_STATUS_0            0x1D

#define BME680_HEAT           0x70
#define BME680_ODR_RUN_GAS_NBC    0x71

#define BME680_FIELD_LENGTH   15
#define BME680_FIELD_ADDR_OFFSET  17

#define BME680_HUM_REG_SHIFT_VAL  4
#define BME680_BIT_H1_DATA_MSK         0x0F

/* Diffrent BOSCH chip id's */
#define BMP085_CHIP_ID                 0x55  /* And also BMP180 */
#define BMP280_CHIP_ID                 0x58
#define BME280_CHIP_ID                 0x60
#define BME680_CHIP_ID                 0x61

/** Array Index to Field data mapping for Calibration Data*/
#define BME680_T2_LSB_REG (1)
#define BME680_T2_MSB_REG (2)
#define BME680_T3_REG   (3)
#define BME680_P1_LSB_REG (5)
#define BME680_P1_MSB_REG (6)
#define BME680_P2_LSB_REG (7)
#define BME680_P2_MSB_REG (8)
#define BME680_P3_REG   (9)
#define BME680_P4_LSB_REG (11)
#define BME680_P4_MSB_REG (12)
#define BME680_P5_LSB_REG (13)
#define BME680_P5_MSB_REG (14)
#define BME680_P7_REG   (15)
#define BME680_P6_REG   (16)
#define BME680_P8_LSB_REG (19)
#define BME680_P8_MSB_REG (20)
#define BME680_P9_LSB_REG (21)
#define BME680_P9_MSB_REG (22)
#define BME680_P10_REG    (23)
#define BME680_H2_MSB_REG (25)
#define BME680_H2_LSB_REG (26)
#define BME680_H1_LSB_REG (26)
#define BME680_H1_MSB_REG (27)
#define BME680_H3_REG   (28)
#define BME680_H4_REG   (29)
#define BME680_H5_REG   (30)
#define BME680_H6_REG   (31)
#define BME680_H7_REG   (32)
#define BME680_T1_LSB_REG (33)
#define BME680_T1_MSB_REG (34)
#define BME680_GH2_LSB_REG  (35)
#define BME680_GH2_MSB_REG  (36)
#define BME680_GH1_REG    (37)
#define BME680_GH3_REG    (38)

#define BME280_MAX_WAIT                300 /* ms. Forced mode max wait */

#define BME680_RSERROR_MSK  0xf0
#define BME680_RHRANGE_MSK  0x30
#define BME680_GASM_VALID_MSK 0x20
#define BME680_HEAT_STAB_MSK  0x10

/** Over-sampling settings */
#define BME680_OS_NONE		0
#define BME680_OS_1X		1
#define BME680_OS_2X		2
#define BME680_OS_4X		3
#define BME680_OS_8X		4
#define BME680_OS_16X		5

/** IIR filter settings */
#define BME680_FILTER_SIZE_0	0
#define BME680_FILTER_SIZE_1	1
#define BME680_FILTER_SIZE_3	2
#define BME680_FILTER_SIZE_7	3
#define BME680_FILTER_SIZE_15	4
#define BME680_FILTER_SIZE_31	5
#define BME680_FILTER_SIZE_63	6
#define BME680_FILTER_SIZE_127	7

struct {
  int32_t temp;
  uint32_t hum;
  uint32_t pres;

  struct {
    uint32_t res; /* Ohm */
    uint32_t iaq; /* Indoor Air Qualty Index */
    int16_t heater_temp; /* Celsius */
    int16_t heater_dur;  /* ms */
  } gas;

  uint8_t os_temp;
  uint8_t os_hum;
  uint8_t os_pres;
  uint8_t filter; /* 3bit, 0-7 Corresponds 0 - 127  fot temp and P */
} bme680;

#endif /* BME680_H */
