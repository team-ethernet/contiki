# Unit testing for SenML API
The unit tests to run are defined in **unit-testing.c**. The print function for the unit tests is also defined there.

## How to run
```
make
./unit-testing.native
```

## Common errors


**make can't find curses.h**

`sudo apt-get install libncurses5-dev libncursesw5-dev`

## Creating tests
### Registering new tests

```UNIT_TEST_REGISTER(name, "description")```

In the ```PROCESS_THREAD``` method:

```UNIT_TEST_RUN(name);```

### Example test design

```
UNIT_TEST(name) {
  int a, b;
  UNIT_TEST_BEGIN();
  a = 1;
  b = 2;
  UNIT_TEST_ASSERT(a+b===3);
  UNIT_TEST_END();
}
```
This test will pass since 1+2=3
