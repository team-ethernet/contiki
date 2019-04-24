#include "contiki.h"
#include "sys/etimer.h"
#include <stdio.h>
#include "adc.h"
#include "i2c.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "dev/noise-sensor.h"
#include "lib/sensors.h"
#include "rss2.h"

static double read_noise_value(void)
{
  return ((adc_read_a1()*100)+4);
}