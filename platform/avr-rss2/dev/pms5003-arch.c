#include "contiki.h"
#include "i2c.h"
#include "watchdog.h"
#include "dev/pms5003.h"
#include "pms5003-arch.h"

static uint8_t standbymode;

void
pms5003_set_standby_mode(uint8_t mode) {
  SET_PMS_DDR |= (1 << PMS_SET);
  if (mode == STANDBY_MODE_OFF)
    SET_PMS_PORT |= (1 << PMS_SET);
  else if (mode == STANDBY_MODE_ON)
    SET_PMS_PORT &= ~(1 << PMS_SET);
  standbymode = mode;
}

uint8_t
pms5003_get_standby_mode(void) {
  return standbymode;
}

uint8_t
pms5003_i2c_probe(void) {
  watchdog_periodic();
  if(!i2c_start(I2C_PMS5003_ADDR)) {
    i2c_stop();
    return 1;
  }
  return 0;
}
