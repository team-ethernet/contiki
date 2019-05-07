#include "contiki.h"
#include "unit-test.h"

UNIT_TEST_REGISTER(json_empty_string, "empty string");
UNIT_TEST_REGISTER(json_noise_sensor, "noise sensor example");

PROCESS(unit_testing, "Unit Testing");
AUTOSTART_PROCESSES(&test_process);

PROCESS_THREAD(unit_testing, ev, data)
{
  PROCESS_BEGIN();

  UNIT_TEST_RUN(json_empty_string);
  UNIT_TEST_RUN(json_noise_sensor);

  PROCESS_END();
}

char * app_buffer[1024];

//Run with json file included.
UNIT_TEST(json_empty_string) {
  UNIT_TEST_BEGIN();

  init(app_buffer, 1024);
  end();

  UNIT_TEST_ASSERT(*app_buffer == "[]");

  UNIT_TEST_END();
}


//Run with json file included.
Test(json_noise_sensor) {
  UNIT_TEST_BEGIN();

  init(app_buffer, 1024);
  add_record(BASE_NAME, "urn:dev:mac:fcc23d0000003790", UNIT, "dB", VALUE, "50.00", NULL);
  end();

  UNIT_TEST_ASSERT(*app_buffer == "[{\"bn\":\"urn:dev:mac:fcc23d0000003790\",\"u\":\"dB\",\"v\":50.00}]");

  UNIT_TEST_END();
}


//Add test cases
//add_start() {
//add_end()   }


//Add test cases for CBOR
