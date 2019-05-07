#include "contiki.h"
#include "sys/etimer.h"
#include <stdio.h>
#include "dev/leds.h"
#include "senml-api.h"
#include "senml-json-formatter.h"
static char app_buffer[1024];

PROCESS(senml_api_process, "Noise sensor process");
AUTOSTART_PROCESSES(&senml_api_process);

static struct etimer et;



  PROCESS_THREAD(senml_api_process, ev, data)
  {
    PROCESS_BEGIN();
	
    leds_init();
    leds_on(LEDS_RED);
    leds_on(LEDS_YELLOW);

    /*
     * Delay 1 sec
     */

      etimer_set(&et, CLOCK_SECOND * 1);
    while(1) {
      PROCESS_YIELD();

      char* buf_ptr = app_buffer;

      init_senml(buf_ptr, 1024, senml_json_formatter);
      add_record(BASE_UNIT, "u/u", BASE_UNIT, "u/u", BASE_UNIT, "u/u", BASE_UNIT, "u/u", BASE_UNIT, "u/u", END);
      add_record(BASE_NAME, "test_name", BASE_TIME, 0.0, BASE_UNIT, "u/u", END);
      add_record(BASE_NAME, "test_name",
	BASE_TIME, 0.0,
	BASE_UNIT, "u/u",
	BASE_VALUE, 1.0,
	BASE_SUM, 2.0,
	BASE_VERSION, 3.0,
	NAME, "name",
	UNIT, "unit",
	VALUE, 4.12,
	STRING_VALUE, "strv",
	BOOLEAN_VALUE, 1,
	DATA_VALUE, "dataval",
	SUM, 5.0,
	TIME, 6.0,
	UPDATE_TIME, 7.0,
	END);
      end_senml();
      printf("senml: %s\n", app_buffer);
      etimer_reset(&et);
    }

    PROCESS_END();
  }
