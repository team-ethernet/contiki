#include "contiki.h"
#include "unit-test.h"
#include "senml-json.h"
#include "senml-cbor.h"
#include <string.h>
#include <stdio.h>

UNIT_TEST_REGISTER(json_empty_string, "JSON: Adding 0 records should return \"[]\"");
UNIT_TEST_REGISTER(json_noise_sensor, "JSON: Noise sensor example");
UNIT_TEST_REGISTER(json_many_parameters, "JSON: Many parameters");
UNIT_TEST_REGISTER(json_multiple_records, "JSON: Multiple records");
UNIT_TEST_REGISTER(cbor_empty_string, "CBOR: Adding 0 records should return \"[]\"");
UNIT_TEST_REGISTER(cbor_noise_sensor, "CBOR: Noise sensor example");
UNIT_TEST_REGISTER(cbor_many_parameters, "CBOR: Many parameters");
UNIT_TEST_REGISTER(cbor_multiple_records, "CBOR: Multiple records");

int strcmpn(char * str1, char * str2, int n) {
  int i;
  for (i = 0; i < n; i++)
  {
    if(str1[i] - str2[i] != 0) return str1[i] - str2[i];
  }
  return 0;
}

UNIT_TEST(json_empty_string){
	char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_JSON(buffer_pointer, 1024);
  END_SENML();

  UNIT_TEST_ASSERT(strcmp(buffer_pointer, "[]") == 0);

  UNIT_TEST_END();
}


UNIT_TEST(json_noise_sensor){
	char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_JSON(buffer_pointer, 1024);
  ADD_RECORD(BASE_NAME, "urn:dev:mac:fcc23d0000003790", UNIT, "dB", BOOLEAN_VALUE, 0, VALUE, 50.00);
  END_SENML();

  UNIT_TEST_ASSERT(strcmp(buffer_pointer, "[{\"bn\":\"urn:dev:mac:fcc23d0000003790\",\"u\":\"dB\",\"vb\":false,\"v\":50.000000}]") == 0);

  UNIT_TEST_END();
}

UNIT_TEST(json_many_parameters){
	char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_JSON(buffer_pointer, 1024);
  ADD_RECORD(BASE_NAME, "urn:dev:mac:fcc23d0000003790", BASE_TIME, 123456789.00, BASE_UNIT, "Volt", BASE_VERSION, 2, VALUE, -1.00, SUM, 0.00, TIME, 643.00, UPDATE_TIME, 0.00);
  END_SENML();
	
  UNIT_TEST_ASSERT(strcmp(buffer_pointer, "[{\"bn\":\"urn:dev:mac:fcc23d0000003790\",\"bt\":123456789.000000,\"bu\":\"Volt\",\"bver\":2,\"v\":-1.000000,\"s\":0.000000,\"t\":643.000000,\"ut\":0.000000}]") == 0);

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

UNIT_TEST(cbor_empty_string){
	char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_CBOR(buffer_pointer, 1024);
  END_SENML();

	char str[] = {0x9F, 0xFF, 0x00};


  UNIT_TEST_ASSERT(strcmpn(buffer_pointer, str, sizeof(str) / sizeof(char)) == 0);

  UNIT_TEST_END();

}

UNIT_TEST(cbor_noise_sensor){
	char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_CBOR(buffer_pointer, 1024);
  ADD_RECORD(BASE_NAME, "urn:mac:fcc2948375028573", UNIT, "dB", VALUE, 59.23);
  END_SENML();
	char str[] = {0x9F, 0xBF, 0x21, 0x78, 0x18, 0x75, 0x72, 0x6E, 0x3A, 0x6D, 0x61, 0x63, 0x3A, 0x66, 0x63, 0x63, 0x32, 0x39, 0x34, 0x38, 0x33, 0x37, 0x35, 0x30, 0x32, 0x38, 0x35, 0x37, 0x33, 0x01, 0x62, 0x64, 0x42, 0x02, 0xFB, 0x40, 0x4D, 0x9D, 0x70, 0xA3, 0xD7, 0x0A, 0x3D, 0xFF, 0xFF, 0x00};

  UNIT_TEST_ASSERT(strcmpn(buffer_pointer, str, sizeof(str) / sizeof(char)) == 0);

  UNIT_TEST_END();
}

UNIT_TEST(cbor_multiple_records){
	char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_CBOR(buffer_pointer, 1024);
  ADD_RECORD(BASE_NAME, "urn:mac:fcc2948375028573", UNIT, "dB", VALUE, 8923.223);
  ADD_RECORD(BOOLEAN_VALUE, 0, BASE_TIME, 1176020076.001);
  END_SENML();
	char str[] = {0x9F, 0xBF, 0x21, 0x78, 0x18, 0x75, 0x72, 0x6E, 0x3A, 0x6D, 0x61, 0x63,0x3A, 0x66, 0x63, 0x63, 0x32, 0x39, 0x34, 0x38, 0x33, 0x37, 0x35, 0x30, 0x32, 0x38, 0x35, 0x37, 0x33, 0x01, 0x62, 0x64, 0x42, 0x02, 0xFB, 0x40, 0xC1, 0x6D, 0x9C, 0x8B, 0x43, 0x95, 0x81, 0xFF, 0xBF, 0x04, 0xF4, 0x22, 0xFB, 0x41, 0xD1, 0x86, 0x29, 0x1B, 0x00, 0x10, 0x62, 0xFF, 0xFF, 0x00};

  UNIT_TEST_ASSERT(strcmpn(buffer_pointer, str, sizeof(str) / sizeof(char)) == 0);

  UNIT_TEST_END();
}

UNIT_TEST(cbor_many_parameters) {
	char buffer_pointer[1024];
  UNIT_TEST_BEGIN();

  INIT_SENML_CBOR(buffer_pointer, 1024);
  ADD_RECORD(BASE_VERSION, 5, UNIT, "m/s", VALUE, 12.608765);
  END_SENML();
	
	char str[] = {0x9F, 0xBF, 0x20, 0x05, 0x01, 0x63, 0x6D, 0x2F, 0x73, 0x02, 0xFB, 0x40, 0x29, 0x37, 0xB0, 0x0B, 0xCB, 0xE6, 0x1D, 0xFF, 0xFF, 0x00};
  UNIT_TEST_ASSERT(strcmpn(buffer_pointer, str, sizeof(str) / sizeof(char)) == 0);

  UNIT_TEST_END();
}

PROCESS(unit_testing, "Unit Testing");

AUTOSTART_PROCESSES(&unit_testing);

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
