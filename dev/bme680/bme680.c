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
 * Created : 2016-09-14
 */

/**
 * \file
 *        Basic functions for Bosch BME680 based on datasheet Rev 1.0
 */

#include "contiki.h"
#include <string.h>
#include "bme680.h"
#include "bme680-arch.h"
#include "lib/sensors.h"
#include <stdio.h>

static struct {
  uint16_t t1;
  int16_t t2;
  int16_t t3;
  uint16_t p1;
  int16_t p2;
  int8_t p3;
  int16_t p4;
  int16_t p5;
  int8_t p6;
  int8_t p7;
  int16_t p8;
  int16_t p9;
  uint8_t p10;
  uint16_t h1;
  uint16_t h2;
  int8_t h3;
  int8_t h4;
  int8_t h5;
  uint8_t h6;
  int8_t h7;
  int32_t t_fine;
  uint8_t mode;
  int8_t gh1;
  int16_t gh2;
  int8_t gh3;

  uint8_t res_heat_range;
  /*! Variable to store heater resistance value */
  int8_t res_heat_val;
  /*! Variable to store error range */
  int8_t range_sw_err;

  /*! Ambient temperature in Degree C*/
  int8_t amb_temp; //BETTER PLACE

} cal;

uint32_t lookupTable1[16] = { 
  2147483647, 2147483647, 2147483647, 2147483647, 2147483647, 
  2126008810, 2147483647, 2130303777, 2147483647, 2147483647, 
  2143188679, 2136746228, 2147483647, 2126008810, 2147483647, 2147483647 };
/**Look up table for the possible gas range values */
uint32_t lookupTable2[16] = { 
  4096000000ul, 2048000000, 1024000000, 512000000, 255744255, 
  127110228,    64000000,   32258064, 16016016,    8000000,  
    4000000,     2000000,    1000000,   500000,     250000, 125000 };

static 
int32_t calc_t(uint32_t temp_adc)
{
  int64_t var1, var2, var3;
  int32_t temp;
  
  var1 = ((int32_t) temp_adc / 8) - ((int32_t) cal.t1 * 2);
  var2 = (var1 * (int32_t) cal.t2) / 2048;
  var3 = ((var1 / 2) * (var1 / 2)) / 4096;
  var3 = ((var3) * ((int32_t) cal.t3 * 16)) / 16384;
  cal.t_fine = (int32_t) (var2 + var3);
  //temp = (int32_t) (var2 + var3);
  temp = (int16_t) (((cal.t_fine * 5) + 128) / 256);
  
  return temp;
}

static uint32_t calc_p(uint32_t pres_adc)
{
  int64_t var1, var2, var3;
  int32_t calc_pres;
	
  var1 = (((int32_t) cal.t_fine) / 2) - 64000;
  var2 = ((var1 / 4) * (var1 / 4)) / 2048;
  var2 = ((var2) * (int32_t) cal.p6) / 4;
  var2 = var2 + ((var1 * (int32_t) cal.p5) * 2);
  var2 = (var2 / 4) + ((int32_t) cal.p4 * 65536);
  var1 = ((var1 / 4) * (var1 / 4)) / 8192;
  var1 = (((var1) * ((int32_t) cal.p3 * 32)) / 8) + (((int32_t) cal.p2 * var1) / 2);
  var1 = var1 / 262144;
  var1 = ((32768 + var1) * (int32_t) cal.p1) / 32768;
  calc_pres = (int32_t) (1048576 - pres_adc);
  calc_pres = (int32_t) ((calc_pres - (var2 / 4096)) * (3125));
  calc_pres = ((calc_pres / var1) * 2);
  var1 = ((int32_t) cal.p9 * (int32_t) (((calc_pres / 8) * (calc_pres / 8)) / 8192)) / 4096;
  var2 = ((int32_t) (calc_pres / 4) * (int32_t) cal.p8) / 8192;
  var3 = ((int32_t) (calc_pres / 256) * (int32_t) (calc_pres / 256) * (int32_t) (calc_pres / 256)
	  * (int32_t) cal.p10) / 131072;
  calc_pres = (int32_t) (calc_pres) + ((var1 + var2 + var3 + ((int32_t) cal.p7 * 128)) / 16);
  
  return (uint32_t) calc_pres;
}

static 
uint32_t calc_h(uint16_t hum_adc)
{
  int32_t var1, var2, var3, var4, var5, var6;
  int32_t temp_scaled, calc_hum;
	
  temp_scaled = (((int32_t) cal.t_fine * 5) + 128) / 256;
  var1 = (int32_t) (hum_adc - ((int32_t) ((int32_t) cal.h1 * 16)))
    - (((temp_scaled * (int32_t) cal.h3) / ((int32_t) 100)) / 2);
  var2 = ((int32_t) cal.h2
	  * (((temp_scaled * (int32_t) cal.h4) / ((int32_t) 100))
	     + (((temp_scaled * ((temp_scaled * (int32_t) cal.h5) / ((int32_t) 100))) / 64)
		/ ((int32_t) 100)) + (int32_t) (1 * 16384))) / 1024;
  var3 = var1 * var2;
  var4 = (int32_t) cal.h6 * 128;
  var4 = ((var4) + ((temp_scaled * (int32_t) cal.h7) / ((int32_t) 100))) / 16;
  var5 = ((var3 / 16384) * (var3 / 16384)) / 1024;
  var6 = (var4 * var5) / 2;
  calc_hum = (((var3 + var6) / 1024) * ((int32_t) 1000)) / 4096;
  
  if (calc_hum > 100000) /* Cap at 100%rH */
    calc_hum = 100000;
  else if (calc_hum < 0)
    calc_hum = 0;
  
  return (uint32_t) calc_hum;
}

static uint32_t calc_gas_res(uint16_t gas_res_adc, uint8_t gas_range)
{
  int64_t var1;
  uint64_t var2;
  int64_t var3;
  uint32_t calc_gas_res;

  var1 = (int64_t) ((1340 + (5 * (int64_t) cal.range_sw_err)) * ((int64_t) lookupTable1[gas_range])) / 65536;
  var2 = (((int64_t) ((int64_t) gas_res_adc * 32768) - (int64_t) (16777216)) + var1);
  var3 = (((int64_t) lookupTable2[gas_range] * (int64_t) var1) / 512);
  calc_gas_res = (uint32_t) ((var3 + ((int64_t) var2 / 2)) / (int64_t) var2);

  return calc_gas_res;
}
static uint8_t calc_heater_res(uint16_t temp)
{
  uint8_t heatr_res;
  int32_t var1, var2, var3, var4, var5;
  int32_t heatr_res_x100;

  if (temp < 200) /* Cap temperature */
    temp = 200;
  else if (temp > 400)
    temp = 400;

  var1 = (((int32_t) cal.amb_temp * cal.gh3) / 1000) * 256;
  var2 = (cal.gh1 + 784) * (((((cal.gh2 + 154009) * temp * 5) / 100) + 3276800) / 10);
  var3 = var1 + (var2 / 2);
  var4 = (var3 / (cal.res_heat_range + 4));
  var5 = (131 * cal.res_heat_val) + 65536;
  heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);
  heatr_res = (uint8_t) ((heatr_res_x100 + 50) / 100);

  return heatr_res;
}


/*   calculate the Heat duration value. */
static uint8_t calc_heater_dur(uint16_t dur)
{
  uint8_t factor = 0;
  uint8_t durval;

  if (dur >= 0xfc0) {
    durval = 0xff; /* Max duration*/
  } else {
    while (dur > 0x3F) {
      dur = dur / 4;
      factor += 1;
    }
    durval = (uint8_t) (dur + (factor * 64));
  }

  return durval;
}


uint8_t buf[BME680_COEFF_ADDR1_LEN+BME680_COEFF_ADDR2_LEN];

uint8_t
bme680_init(void)
{
  uint16_t i;
  uint8_t tmp;

  /* Do not mess with other chips */
  bme680_arch_i2c_read_mem(BME680_ADDR, 0xD0, buf, 1);

  if(buf[0] != BME680_CHIP_ID) {
    return 0;
  }

  bme680_arch_i2c_write_mem(BME680_ADDR, BME680_RESET, 0xB6);

  for(i = 0; i < BME680_RESET_TIME; i++) {
    clock_delay_usec(1000);
  }

  bme680_arch_i2c_init();
  memset(buf, 0, sizeof(buf));

  /* Burst read of all calibration part 1 */
  bme680_arch_i2c_read_mem(BME680_ADDR, BME680_COEFF_ADDR1, buf, BME680_COEFF_ADDR1_LEN);
  /* Burst read of all calibration part 2 */
  bme680_arch_i2c_read_mem(BME680_ADDR, BME680_COEFF_ADDR2, &buf[BME680_COEFF_ADDR1_LEN], 
			   BME680_COEFF_ADDR2_LEN);

  /* Temp coeff */
  cal.t1 = ((uint16_t)buf[BME680_T1_MSB_REG] << 8) | buf[BME680_T1_LSB_REG];
  cal.t2 = ((int16_t)buf[BME680_T2_MSB_REG] << 8) | buf[BME680_T2_LSB_REG];
  cal.t3 = (uint16_t)buf[BME680_T3_REG];
  /* Pressure coeff */
  cal.p1 = (uint16_t) (buf[BME680_P1_MSB_REG]<<8) | buf[BME680_P1_LSB_REG];
  cal.p2 = (int16_t) (buf[BME680_P2_MSB_REG]<<8) | buf[BME680_P2_LSB_REG];
  cal.p3 = (int8_t) buf[BME680_P3_REG];
  cal.p4 = (int16_t) (buf[BME680_P4_MSB_REG]<<8) | buf[BME680_P4_LSB_REG];
  cal.p5 = (int16_t) (buf[BME680_P5_MSB_REG]<<8) | buf[BME680_P5_LSB_REG];
  cal.p6 = (int8_t) (buf[BME680_P6_REG]);
  cal.p7 = (int8_t) (buf[BME680_P7_REG]);
  cal.p8 = (int16_t) (buf[BME680_P8_MSB_REG]<<8) | buf[BME680_P8_LSB_REG];
  cal.p9 = (int16_t) (buf[BME680_P9_MSB_REG]<<8) | buf[BME680_P9_LSB_REG];
  cal.p10 = (uint8_t) (buf[BME680_P10_REG]);

  /* RH coeff */
  cal.h1 = (uint16_t) (((uint16_t) buf[BME680_H1_MSB_REG] << BME680_HUM_REG_SHIFT_VAL) | 
		       (buf[BME680_H1_LSB_REG] & BME680_BIT_H1_DATA_MSK));
  cal.h2 = (uint16_t) (((uint16_t) buf[BME680_H2_MSB_REG] << BME680_HUM_REG_SHIFT_VAL) | 
		       ((buf[BME680_H2_LSB_REG]) >> BME680_HUM_REG_SHIFT_VAL));
  cal.h3 = (int8_t) buf[BME680_H3_REG];
  cal.h4 = (int8_t) buf[BME680_H4_REG];
  cal.h5 = (int8_t) buf[BME680_H5_REG];
  cal.h6 = (uint8_t) buf[BME680_H6_REG];
  cal.h7 = (int8_t) buf[BME680_H7_REG];

  /* Heater coeff */
  cal.gh1 = (int8_t) buf[BME680_GH1_REG];
  cal.gh2 = (int16_t) buf[BME680_GH2_MSB_REG]<<8 | buf[BME680_GH2_LSB_REG];
  cal.gh3 = (int8_t) buf[BME680_GH3_REG];

  bme680_arch_i2c_read_mem(BME680_ADDR, BME680_ADDR_RES_HEAT_RANGE_ADDR, &tmp, 1);
  cal.res_heat_range = ( tmp & BME680_RHRANGE_MSK) / 16;
  bme680_arch_i2c_read_mem(BME680_ADDR, BME680_ADDR_RES_HEAT_VAL_ADDR, &tmp, 1);
  cal.res_heat_val = tmp;
  bme680_arch_i2c_read_mem(BME680_ADDR, BME680_ADDR_RANGE_SW_ERR_ADDR, &tmp, 1);
  cal.range_sw_err = ( tmp & BME680_RSERROR_MSK) / 16;

  return 1;
}
void
bme680_read(void)
{
  uint32_t ut, up;
  uint16_t uh, adc_gas_res;
  uint8_t gas_range;
  
  uint16_t l1, l2;

  uint8_t ght, ghd;

  uint8_t sleep;
  uint16_t i;
  memset(buf, 0, sizeof(buf));

  ut = uh = up = 0;
  l1 = l2 = 0;

  /* Humidity oversampling *1 */
  bme680_arch_i2c_write_mem(BME680_ADDR, BME680_CNTL_HUM, 0x01);
  
  /*  00100111  0x27 oversampling *1 for t and p plus normal mode */
  /* 0.5 ms -- no filter -- no SPI */
  bme680_arch_i2c_write_mem(BME680_ADDR, BME680_CONFIG, 0x00); // KOLLA
  
  // GAS heater off 
  //bme680_arch_i2c_write_mem(BME680_ADDR, BME680_GAS_0, 0x04);

  // 00100101 Temp and P oversampling * 1 + FORCE MODE
  // bme680_arch_i2c_write_mem(BME680_ADDR, BME680_CNTL_MEAS, 0x25);

  ght = calc_heater_res(320);
  ghd = calc_heater_dur(100);
  printf("gdt=%d, ghd=%d\n", ght, ghd);
  bme680_arch_i2c_write_mem(BME680_ADDR, BME680_GAS_WAIT_0, ghd);
  bme680_arch_i2c_write_mem(BME680_ADDR, BME680_RES_HEAT_0, ght);
  // Select the heater  0 above start 0x10
  bme680_arch_i2c_write_mem(BME680_ADDR, BME680_GAS_1, 0x10);

  /* Gas measurements */
  bme680_arch_i2c_write_mem(BME680_ADDR, BME680_GAS_0, 0x00);


  /* Start gas measuement */
  //bme680_arch_i2c_write_mem(BME680_ADDR, BME680_CNTL_MEAS, 0x1);
  bme680_arch_i2c_write_mem(BME680_ADDR, BME680_CNTL_MEAS, 0x25);

  /* Wait to get into sleep mode == measurement done */
  for(i = 0; i < BME280_MAX_WAIT; i++) {
    l1++;
    bme680_arch_i2c_read_mem(BME680_ADDR, BME680_MEAS_STATUS_0, &sleep, 1);
    sleep = sleep& 0x10; // BIT 5
    if(sleep == 0) {
      break;
    } else {
      clock_delay_usec(1000);
    }
  }
  if(i == BME280_MAX_WAIT) {
    printf("TPH MAX--WAIT\n");
    return; /* error  wait*/
  }

  /* Burst read of all measurements */
  bme680_arch_i2c_read_mem(BME680_ADDR, BME680_FIELD0_ADDR , buf, BME680_FIELD_LENGTH);
  //data->status = buff[0] & BME680_NEW_DATA_MSK;
  //data->gas_index = buff[0] & BME680_GAS_INDEX_MSK;
  //data->meas_index = buff[1];

  /* Set sleep mode */
  bme680_arch_i2c_write_mem(BME680_ADDR, BME680_CNTL_MEAS, 0x0);

  /* read the raw data from the sensor */
  up = (uint32_t) (((uint32_t) buf[2] * 4096) | ((uint32_t) buf[3] * 16) | ((uint32_t) buf[4] / 16));
  ut = (uint32_t) (((uint32_t) buf[5] * 4096) | ((uint32_t) buf[6] * 16) | ((uint32_t) buf[7] / 16));
  uh = (uint16_t) (((uint32_t) buf[8] * 256) | (uint32_t) buf[9]);
  bme680_mea.t_overscale100 = calc_t(ut);
  bme680_mea.h_overscale1024 = calc_h(uh);
  bme680_mea.p = calc_p(up);


#if 0
  for(i = 0; i < BME280_MAX_WAIT; i++) {
    l2++;
    bme680_arch_i2c_read_mem(BME680_ADDR, BME680_MEAS_STATUS_0, &sleep, 1);
    sleep = sleep& 0x20; // BIT 6
    if(sleep == 0) {
      break;
    } else {
      clock_delay_usec(1000);
    }
  }
  if(i == BME280_MAX_WAIT) {
    printf("GAS MAX--WAIT\n");
    return; /* error  wait*/
  }
#endif


  adc_gas_res = (uint16_t) ((uint32_t) buf[13] * 4 | (((uint32_t) buf[14]) / 64));
  gas_range = buf[14] & BME680_GAS_RANGE_MSK;


  printf("GAS=%lu %u %lu l1=%d, l2=%d\n", adc_gas_res, gas_range, calc_gas_res(adc_gas_res, gas_range), l1, l2);

#if TEST
  printf("T_BME680=%5.2f", (double)bme680_mea.t_overscale100 / 100.);
  printf(" RH_BME680=%5.2f ", (double)bme680_mea.h_overscale1024 / 1024.);
#ifdef BME680_64BIT
  printf(" P_BME680=%5.2f\n", (double)bme680_mea.p_overscale256 / 256.);
#else
  printf(" P_BME680=%5.2f\n", (double)bme680_mea.p);
#endif
#endif
}
