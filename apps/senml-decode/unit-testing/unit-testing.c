#include "contiki.h"
#include "unit-test.h"
#include <string.h>
#include <stdio.h>
#include "senml-decode.h"
#include "jsonparse.h"

UNIT_TEST_REGISTER(add_msg_and_read_label, "Add msg and read label");
UNIT_TEST_REGISTER(add_msg_and_read_value_str, "Add msg and read str value");
UNIT_TEST_REGISTER(add_msg_and_read_value_int, "Add msg and read int value");
UNIT_TEST_REGISTER(read_add_read_label, "Read, add, read label");
UNIT_TEST_REGISTER(read_add_read_value_str, "read, add, read str value");
UNIT_TEST_REGISTER(read_add_read_value_int," read, add, read int value");
UNIT_TEST_REGISTER(read_one_label, "read one label");
int strcmpn(char * str1, char * str2, int n) {
  int i;
  for (i = 0; i < n; i++)
  {
    if(str1[i] - str2[i] != 0) return str1[i] - str2[i];
  }
  return 0;
}

UNIT_TEST(read_one_label){
  UNIT_TEST_BEGIN();
  init_json_decoder("[{\"bn\": \"urn:mac:testID\"]");
  struct pair lv;
  read_next_token(&lv);
  UNIT_TEST_ASSERT(strcmp(lv.label, "bn") == 0);
  UNIT_TEST_END();
}

UNIT_TEST(add_msg_and_read_label){
  UNIT_TEST_BEGIN();
  init_json_decoder("[{\"bn\": \"urn:mac:testID\"},");
  add_new_msg("{\"u\": \"dB\"},");
  struct pair lv;
  read_next_token(&lv);
  read_next_token(&lv);
  UNIT_TEST_ASSERT(strcmp(lv.label, "u") == 0);
  UNIT_TEST_END();
}
 
UNIT_TEST(add_msg_and_read_value_str){
  UNIT_TEST_BEGIN();
  init_json_decoder("[{\"bn\": \"urn:mac:testID\"},");
  add_new_msg("{\"u\": \"dB\"},");
  struct pair lv;
  read_next_token(&lv);
  read_next_token(&lv);
 
  UNIT_TEST_ASSERT(strcmp(lv.value, "dB") == 0);
  UNIT_TEST_END();
}
 
UNIT_TEST(add_msg_and_read_value_int){
  UNIT_TEST_BEGIN();
  init_json_decoder("[{\"v\": 10},");
  add_new_msg("{\"v\": 10},");
  struct pair lv;
  read_next_token(&lv);
  read_next_token(&lv);
  UNIT_TEST_ASSERT(strcmp(lv.value, "10") == 0);
  UNIT_TEST_END();
}
 
UNIT_TEST(read_add_read_label){
  UNIT_TEST_BEGIN();
  init_json_decoder("[{\"bn\": \"urn:mac:testID\"},");
  struct pair lv;
  read_next_token(&lv);
  add_new_msg("{\"v\": 10},");
  read_next_token(&lv);
 
  UNIT_TEST_ASSERT(strcmp(lv.label, "v") == 0);
  UNIT_TEST_END();
}
 
UNIT_TEST(read_add_read_value_str){
  UNIT_TEST_BEGIN();
  init_json_decoder("[{\"bn\": \"urn:mac:testID\"},");
  struct pair lv;
  read_next_token(&lv);
  add_new_msg("{\"u\": \"dB\"},");
  read_next_token(&lv);
  UNIT_TEST_ASSERT(strcmp(lv.value, "dB") == 0);
  UNIT_TEST_END();
}
 
UNIT_TEST(read_add_read_value_int){
  UNIT_TEST_BEGIN();
  init_json_decoder("[{\"bn\": \"urn:mac:testID\"},");
  struct pair lv;
  read_next_token(&lv);
  add_new_msg("{\"v\": 10},");
  read_next_token(&lv);
  UNIT_TEST_ASSERT(strcmp(lv.value, "10") == 0);
  UNIT_TEST_END();
}

PROCESS(unit_testing, "Unit Testing");

AUTOSTART_PROCESSES(&unit_testing);

PROCESS_THREAD(unit_testing, ev, data){
  PROCESS_BEGIN();
	UNIT_TEST_RUN(read_one_label);
  UNIT_TEST_RUN(add_msg_and_read_label);
  UNIT_TEST_RUN(add_msg_and_read_value_str);
  UNIT_TEST_RUN(add_msg_and_read_value_int);
  UNIT_TEST_RUN(read_add_read_label);
  UNIT_TEST_RUN(read_add_read_value_str);
  UNIT_TEST_RUN(read_add_read_value_int);
  PROCESS_END();
}
