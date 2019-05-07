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

static char * buffer_pointer[1024];

//Run with json file included.
UNIT_TEST(json_empty_string) {
  UNIT_TEST_BEGIN();

  init_senml(buffer_pointer, 1024);
  end_senml();

  UNIT_TEST_ASSERT(*buffer_pointer == "[]");

  UNIT_TEST_END();
}


//Run with json file included.
Test(json_noise_sensor) {
  UNIT_TEST_BEGIN();

  init_senml(buffer_pointer, 1024);
  add_record(BASE_NAME, "urn:dev:mac:fcc23d0000003790", UNIT, "dB", VALUE, 50.00, NULL);
  end_senml();

  UNIT_TEST_ASSERT(*buffer_pointer == "[{\"bn\":\"urn:dev:mac:fcc23d0000003790\",\"u\":\"dB\",\"v\":50.00}]");

  UNIT_TEST_END();
}

Test(cbor_empty_string){
  UNIT_TEST_BEGIN();

  init_senml(buffer_pointer, 1024);
  end_senml();

  UNIT_TEST_ASSERT(*buffer_pointer == "80");

  UNIT_TEST_END();

}
Test(cbor_noise_sensor){
  UNIT_TEST_BEGIN();

  init_senml(buffer_pointer, 1024);
  add_record(BASE_NAME, "urn:dev:mac:fcc23d0000003790", UNIT, "dB", VALUE, 50.00, NULL);
  end_senml();

  UNIT_TEST_ASSERT(*buffer_pointer == "81A362626E781C75726E3A6465763A6D61633A6663633233643030303030303337393061756264426176F95240");

  UNIT_TEST_END();
}

//Add test cases


//Add test cases for CBOR
