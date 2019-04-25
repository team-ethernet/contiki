#ifndef NOISE_SENSOR_H_
#define NOISE_SENSOR_H_

#include "lib/sensors.h"

double read_noise_value(void);
extern const struct sensors_sensor noise_sensor;

#endif