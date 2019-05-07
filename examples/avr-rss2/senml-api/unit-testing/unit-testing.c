#include "contiki.h"
#include "unit-test.h"
#include <stdio.h>

/* Register two unit tests that will be executed by using 
   the UNIT_TEST_RUN macro. */
UNIT_TEST_REGISTER(arithmetic, "Arith ops");
UNIT_TEST_REGISTER(string, "String ops");

/* arithmetic: Demonstrates a test that succeeds. The exit point will be 
   the line where UNIT_TEST_END is called. */
UNIT_TEST(arithmetic)
{
  int a, b;

  UNIT_TEST_BEGIN();

  a = 1;
  b = 2;

  UNIT_TEST_ASSERT(a + b == 3);

  UNIT_TEST_END();
}

/* string: Demonstrates a test that fails. The exit point will be 
   the line where the call to UNIT_TEST_ASSERT fails. */
UNIT_TEST(string)
{
  char str1[] = "A";

  UNIT_TEST_BEGIN();

  UNIT_TEST_ASSERT(str1[0] == 'B');

  UNIT_TEST_END();
}

PROCESS(test_process, "Unit testing");
AUTOSTART_PROCESSES(&test_process);

PROCESS_THREAD(test_process, ev, data)
{
  PROCESS_BEGIN();

  UNIT_TEST_RUN(arithmetic);
  UNIT_TEST_RUN(string);

  PROCESS_END();
}
