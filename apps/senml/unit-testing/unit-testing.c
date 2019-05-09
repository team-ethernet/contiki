#include "contiki.h"
#include "unit-test.h"
#include "senml-json.h"
#include <string.h>
#include <stdio.h>

UNIT_TEST_REGISTER(json_empty_string, "json empty string");
UNIT_TEST_REGISTER(json_noise_sensor, "json noise sensor example");
UNIT_TEST_REGISTER(json_many_parameters, "json many parameters");
UNIT_TEST_REGISTER(json_multiple_records, "json many records");
/*UNIT_TEST_REGISTER(cbor_empty_string, "cbor empty string");
UNIT_TEST_REGISTER(cbor_noise_sensor, "cbor noise sensor example");
UNIT_TEST_REGISTER(cbor_many_parameters, "cbor many parameters");
UNIT_TEST_REGISTER(cbor_multiple_records, "cbor multiple records");*/


//All json tests should be run wiht json file included
UNIT_TEST(json_empty_string){
char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_JSON(buffer_pointer, 1024);
  END_SENML();

  printf("==> %s", buffer_pointer);

  //UNIT_TEST_ASSERT(strcmp(buffer_pointer, "[]") == 0);

  UNIT_TEST_END();
}


UNIT_TEST(json_noise_sensor){
char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_JSON(buffer_pointer, 1024);
  ADD_RECORD(BASE_NAME, "urn:dev:mac:fcc23d0000003790", UNIT, "dB", VALUE, 50.00);
  END_SENML();

  UNIT_TEST_ASSERT(strcmp(buffer_pointer, "[{\"bn\":\"urn:dev:mac:fcc23d0000003790\",\"u\":\"dB\",\"v\":50.000000}]") == 0);

  UNIT_TEST_END();
}

UNIT_TEST(json_many_parameters){
char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_JSON(buffer_pointer, 1024);
  ADD_RECORD(BASE_NAME, "urn:dev:mac:fcc23d0000003790", BASE_TIME, 123456789.00, BASE_UNIT, "Volt", BASE_VERSION, 2.00, VALUE, -1.00, SUM, 0.00, TIME, 643.00, UPDATE_TIME, 0.00);
  END_SENML();

  UNIT_TEST_ASSERT(strcmp(buffer_pointer, "[{\"bn\":\"urn:dev:mac:fcc23d0000003790\",\"bt\":123456789.000000,\"bu\":\"Volt\",\"bver\":2.000000,\"v\":-1.000000,\"s\":0.000000,\"t\":643.000000,\"ut\":0.000000}]") == 0);

  UNIT_TEST_END();
}

UNIT_TEST(json_multiple_records){
char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_JSON(buffer_pointer, 1024);
  ADD_RECORD(BASE_NAME, "urn:dev:mac:fcc23d0000003790", UNIT, "dB", VALUE, 50.00);
  ADD_RECORD(BASE_NAME, "urn:dev:mac:fcc23d0000003788", UNIT, "w", VALUE, 0.00);
  ADD_RECORD(BASE_NAME, "urn:dev:mac:fcc23d0000013290", UNIT, "m", VALUE, 35.25);
  END_SENML();

  UNIT_TEST_ASSERT(strcmp(buffer_pointer, "[{\"bn\":\"urn:dev:mac:fcc23d0000003790\",\"u\":\"dB\",\"v\":50.000000},{\"bn\":\"urn:dev:mac:fcc23d0000003788\",\"u\":\"w\",\"v\":0.000000},{\"bn\":\"urn:dev:mac:fcc23d0000013290\",\"u\":\"m\",\"v\":35.250000}]") == 0);

  UNIT_TEST_END();
}
/*
UNIT_TEST(cbor_empty_string){
char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_CBOR(buffer_pointer, 1024);
  END_SENML();

  UNIT_TEST_ASSERT(strcmp(buffer_pointer, "80") == 0);

  UNIT_TEST_END();

}

UNIT_TEST(cbor_noise_sensor){
char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_CBOR(buffer_pointer, 1024);
  ADD_RECORD(BASE_NAME, "urn:dev:mac:fcc23d0000003790", UNIT, "dB", VALUE, 50.00);
  END_SENML();

  UNIT_TEST_ASSERT(strcmp(buffer_pointer, "81A362626E781C75726E3A6465763A6D61633A6663633233643030303030303337393061756264426176F95240") == 0);

  UNIT_TEST_END();
}

UNIT_TEST(cbor_many_parameters){
char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_CBOR(buffer_pointer, 1024);
  ADD_RECORD(BASE_NAME, "urn:dev:mac:fcc23d0000003790", BASE_TIME, 123456789, BASE_UNIT, "Volt", BASE_VERSION, 2, VALUE, -1, SUM, 0, TIME, 643, UPDATE_TIME, 000);
  END_SENML();
  UNIT_TEST_ASSERT(strcmp(buffer_pointer, "81A862626E781C75726E3A6465763A6D61633A666363323364303030303030333739306262741A075BCD1562627564566F6C74646276657202617620617300617419028362757400") == 0);

  UNIT_TEST_END();
}

UNIT_TEST(cbor_multiple_records) {
char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_CBOR(buffer_pointer, 1024);
  ADD_RECORD(BASE_NAME, "urn:dev:mac:fcc23d0000003790", UNIT, "dB", VALUE, 50.00);
  ADD_RECORD(BASE_NAME, "urn:dev:mac:fcc23d0000003788", UNIT, "w", VALUE, 0.00);
  ADD_RECORD(BASE_NAME, "urn:dev:mac:fcc23d0000013290", UNIT, "m", VALUE, 35.25);
  END_SENML();

  UNIT_TEST_ASSERT(strcmp(buffer_pointer, "83A362626E781C75726E3A6465763A6D61633A6663633233643030303030303337393061756264426176F95240A362626E781C75726E3A6465763A6D61633A66636332336430303030303033373838617561776176F90000A362626E70666363323364303030303031333239306175616D6176F95068") == 0);

  UNIT_TEST_END();
}*/

//Equivalent to the main() and is where the unit-testing starts.
PROCESS(unit_testing, "Unit Testing");
AUTOSTART_PROCESSES(&unit_testing);

PROCESS_THREAD(unit_testing, ev, data){
  PROCESS_BEGIN();

  UNIT_TEST_RUN(json_empty_string);
  UNIT_TEST_RUN(json_noise_sensor);
  UNIT_TEST_RUN(json_many_parameters);
  UNIT_TEST_RUN(json_multiple_records);
  /*UNIT_TEST_RUN(cbor_empty_string);
  UNIT_TEST_RUN(cbor_noise_sensor);
  UNIT_TEST_RUN(cbor_many_parameters);
  UNIT_TEST_RUN(cbor_multiple_records);*/

  PROCESS_END();
}

