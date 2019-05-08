#ifndef SEN0232_GSLM_H_
#define SEN0232_GSLM_H_
#include "lib/sensors.h"

extern const struct sensors_sensor sen0232_gslm;
void sen0232_init(void);
void sen0232_disable(void);
int value(int type);

#endif