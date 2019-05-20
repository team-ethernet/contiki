#include "contiki.h"
#include "sys/etimer.h"
#include <stdio.h>
#include "adc.h"
#include "i2c.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
//#include "dev/noise-sensor.h"
#include "dev/noise-sensor.c"

#include "dev/sen0232_gslm.h"
#include "dev/sen0232_gslm.c"

PROCESS(noise_sensors_process, "Noise sensor process");
AUTOSTART_PROCESSES(&noise_sensors_process);

static struct etimer et;

static void read_values (void)
{
	//printf("%d\n", sen0232_gslm.value(0));
	printf("%d\n", OLDMICvalue(0));
	
}

  PROCESS_THREAD(noise_sensors_process, ev, data)
  {
    PROCESS_BEGIN();

    SENSORS_ACTIVATE(button_sensor);
	//SENSORS_ACTIVATE(noise_sensor);
	
    leds_init();
    leds_on(LEDS_RED);
    leds_on(LEDS_YELLOW);
	SENSORS_ACTIVATE(sen0232_gslm);
	pwr_1_init();

    /*
     * Delay 5 sec
     * Gives a chance to trigger some pulses
     */

      etimer_set(&et, CLOCK_SECOND * 3);
    while(1) {
      PROCESS_YIELD();
      //PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

      if (ev == sensors_event && data == &button_sensor) {
        leds_on(LEDS_YELLOW);
        printf("Button pressed\n");
      }

      read_values();
      etimer_reset(&et);
    }

    PROCESS_END();
  }
