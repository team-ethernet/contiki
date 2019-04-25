#include "contiki.h"
#include "sys/etimer.h"
#include <stdio.h>
#include "adc.h"
#include "i2c.h"
#include "lib/sensors.h"
#include "rss2.h"
#include "dev/noise-sensor.h"

/*	PROJECT: Kth 2019 Internet of Noisy things
*	
*	
*	
*	This file contains the driver for the external noise sensor
*	Digital sound level meter version 1358-EN-00
*	

*/	

double read_noise_value(void)
{
  return ((adc_read_a1()*100)); //Multiply voltage value with
								//100 as the voltage is linearly proportional 
								//with the dB value but 100 times smaller
}