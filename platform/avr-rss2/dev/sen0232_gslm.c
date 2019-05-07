#include "contiki.h"
#include "lib/sensors.h"
#include "rss2.h"
#include "pwr.c"

#define multiply_value 57

const struct sensors_sensor sen0232_gslm;
	/*
	* Multiply voltage value with
	* a constant as the voltage is linearly proportional 
	* with the dB value but a number of times smaller
	* Through testing this number is found to be around or 
	* slightly above 55 for our microphone
	* Testing will need to be done for other microphones
	*
	* Turn power on 175 ticks before reading
	* clock_wait(time) waits time*8 ms
	*/
static int value(int type)
{
	pwr_1_on();
	clock_wait(175);
	int dBVal = ((int)(adc_read_a1()*multiply_value)); 
	pwr_1_off();
	return ((int)dBVal);
}
static int status(int type)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static int configure(int type, int c)
{
  return 0;
}

SENSORS_SENSOR(sen0232_gslm, "Noise", value, configure, status);