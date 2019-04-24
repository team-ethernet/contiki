#include "contiki.h"
#include "sys/etimer.h"
#include <stdio.h>
#include "adc.h"
#include "i2c.h"
#include "dev/button-sensor.h"
#include "dev/noise-sensor.h"


PROCESS(noise_sensors_process, "Noise sensor process");
AUTOSTART_PROCESSES(&noise_sensors_process);

static struct etimer et;

static void read_values (void)
{

char serial[16];

int i;

i2c_at24mac_read((char *) &serial, 0);
printf("128_bit_ID=");

for(i=0; i < 15; i++){
  printf("%02x", serial[i]);
}
  printf("%02x\n", serial[15]);

  printf("NOICE=%-d dB", noise_sensor.value(0));


}

double read_noise_sensor(void)
{
  return ((((double)adc_read(A1)) * V_IN_FACTOR)*100)+4;
}
  PROCESS_THREAD(noise_sensors_process, ev, data)
  {
    PROCESS_BEGIN();

    SENSORS_ACTIVATE(button_sensor);
	SENSORS_ACTIVATE(noise_sensor);

    /*
     * Delay 5 sec
     * Gives a chance to trigger some pulses
     */

      etimer_set(&et, CLOCK_SECOND * 5);
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
