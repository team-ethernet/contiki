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

//const struct sensors_sensor noise_sensor;

int OLDMICvalue(int type)
{
  return ((int)(adc_read_a1()*100)); //Multiply voltage value with
								//100 as the voltage is linearly proportional 
								//with the dB value but 100 times smaller
}
/*
*	---------------------------------------------------------------------------
*	static int status(int type)
*	{
* 	 return 0;
*	}
*	---------------------------------------------------------------------------
*	static int configure(int type, int c)
*	{
*	  return 0;
*	}
*	---------------------------------------------------------------------------
*	SENSORS_SENSOR(noise_sensor, "Noise", value, configure, status);
*/
