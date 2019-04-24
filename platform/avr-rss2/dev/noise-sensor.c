#include "contiki.h"
#include "lib/sensors.h"
#include "dev/light-sensor.h"
#include "rss2.h"
#include "adc.h"
#include "lib/sensors.h"

double value(void)
{
  return ((((double)adc_read(A1)) * V_IN_FACTOR)*100)+4;
}

static int status(int type)
{
  return 0;
}

static int configure(int type, int c)
{
  DDRF &= ~(1 << A1); /* Noise sensor */
  DDRF &= ~(1 << A1_PWR);

  PORTF |= (1 << A1_PWR); /* Noise sensor */
  return 0;
}

SENSORS_SENSOR(noise_sensor, "Noise", value, configure, status);