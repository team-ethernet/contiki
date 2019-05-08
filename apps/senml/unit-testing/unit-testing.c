#include "contiki.h"
#include "unit-test.h"

UNIT_TEST_REGISTER(json_empty_string, "json empty string");
UNIT_TEST_REGISTER(json_noise_sensor, "json noise sensor example");
UNIT_TEST_REGISTER(json_many_parameters, "json many parameters");
UNIT_TEST_REGISTER(json_multiple_records, "json many records");
UNIT_TEST_REGISTER(cbor_empty_string, "cbor empty string");
UNIT_TEST_REGISTER(cbor_noise_sensor, "cbor noise sensor example");
UNIT_TEST_REGISTER(cbor_many_parameters, "cbor many parameters");
UNIT_TEST_REGISTER(cbor_multiple_records, "cbor multiple records");


PROCESS(unit_testing, "Unit Testing");
AUTOSTART_PROCESSES(&test_process);

PROCESS_THREAD(unit_testing, ev, data){
  PROCESS_BEGIN();

  UNIT_TEST_RUN(json_empty_string);
  UNIT_TEST_RUN(json_noise_sensor);
  UNIT_TEST_RUN(json_many_parameters);
  UNIT_TEST_RUN(json_multiple_records);
  UNIT_TEST_RUN(cbor_empty_string);
  UNIT_TEST_RUN(cbor_noise_sensor);
  UNIT_TEST_RUN(cbor_many_parameters);
  UNIT_TEST_RUN(cbor_multiple_records);

  PROCESS_END();
}

static char * buffer_pointer[1024];

//Run with json file included.
UNIT_TEST(json_empty_string){
  UNIT_TEST_BEGIN();

  init_senml(buffer_pointer, 1024);
  end_senml();

  UNIT_TEST_ASSERT(*buffer_pointer == "[]");

  UNIT_TEST_END();
}


//Run with json file included.
Test(json_noise_sensor){
  UNIT_TEST_BEGIN();

  init_senml(buffer_pointer, 1024);
  add_record(BASE_NAME, "urn:dev:mac:fcc23d0000003790", UNIT, "dB", VALUE, 50.00, NULL);
  end_senml();

  UNIT_TEST_ASSERT(*buffer_pointer == "[{\"bn\":\"urn:dev:mac:fcc23d0000003790\",\"u\":\"dB\",\"v\":50.00}]");

  UNIT_TEST_END();
}

Test(json_many_parameters){
  UNIT_TEST_BEGIN();

  init_senml(buffer_pointer, 1024);
  add_record(BASE_NAME, "urn:dev:mac:fcc23d0000003790", BASE_TIME, 123456789, BASE_UNIT, "Volt", BASE_VERSION, 2, VALUE, -1, SUM, 0, TIME, 643, UPDATE_TIME, 000, NULL);
  end_senml();

  UNIT_TEST_ASSERT(*buffer_pointer == "[{\"bn\":\"urn:dev:mac:fcc23d0000003790\",\"bt\":123456789,\"bu\":\"Volt\",\"bv\":2,\"v\":-1,\"s\":0,\"t\":643,\"tu\":000}]")

}

Test(json_multiple_records){
  UNIT_TEST_BEGIN();

  init_senml(buffer_pointer, 1024);
  add_record(BASE_NAME, "urn:dev:mac:fcc23d0000003790", UNIT, "dB", VALUE, 50.00, NULL);
  add_record(BASE_NAME, "urn:dev:mac:fcc23d0000003788", UNIT, "w", VALUE, 0.00, NULL);
  add_record(BASE_NAME, "urn:dev:mac:fcc23d0000013290", UNIT, "m", VALUE, 35.25, NULL);
  end_senml();

  UNIT_TEST_ASSERT(*buffer_pointer == "[{\"bn\":\"urn:dev:mac:fcc23d0000003790\",\"u\":\"dB\",\"v\":50.00},{\"bn\":\"urn:dev:mac:fcc23d0000003788\",\"u\":\"w\",\"v\":0.00},{\"bn\":\"fcc23d0000013290\",\"u\":\"m\",\"v\":35.25}]");

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

Test(cbor_many_paramters){
  UNIT_TEST_BEGIN();

  init_senml(buffer_pointer, 1024);
  add_record(BASE_NAME, "urn:dev:mac:fcc23d0000003790", BASE_TIME, 123456789, BASE_UNIT, "Volt", BASE_VERSION, 2, VALUE, -1, SUM, 0, TIME, 643, UPDATE_TIME, 000, NULL);
  end_senml();
  UNIT_TEST_ASSERT(*buffer_pointer == "81A862626E781C75726E3A6465763A6D61633A666363323364303030303030333739306262741A075BCD1562627564566F6C74646276657202617620617300617419028362757400")

  UNIT_TEST_END();
}

Test(cbor_multiple_records) {
  UNIT_TEST_BEGIN();

  init_senml(buffer_pointer, 1024);
  add_record(BASE_NAME, "urn:dev:mac:fcc23d0000003790", UNIT, "dB", VALUE, 50.00, NULL);
  add_record(BASE_NAME, "urn:dev:mac:fcc23d0000003788", UNIT, "w", VALUE, 0.00, NULL);
  add_record(BASE_NAME, "urn:dev:mac:fcc23d0000013290", UNIT, "m", VALUE, 35.25, NULL);
  end_senml();

  UNIT_TEST_ASSERT(*buffer_pointer == "83A362626E781C75726E3A6465763A6D61633A6663633233643030303030303337393061756264426176F95240A362626E781C75726E3A6465763A6D61633A66636332336430303030303033373838617561776176F90000A362626E70666363323364303030303031333239306175616D6176F95068");

  UNIT_TEST_END();
}
