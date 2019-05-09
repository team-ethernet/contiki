# senml app

## senml Code

apps/senml/senml-api.c  
apps/senml/senml-json-formatter.c


## Project code needs

```c
#include "senml-json.h"
```  
and/or  
```c
#include "senml-cbor.h"
```  
depending on desired format

## Build

To include in project:
in Makefile add

```
APPS += senml
```

## Use

API is used through three different macros:  
INIT_SENML_JSON or INIT_SENML_CBOR, ADD_RECORD, and END_RECORD.  

The SenML message is written into the buffer given as an argument to INIT.

ADD_RECORD takes field name-value pairs as arguments. The supported fields are:

| Field name    | Data type |
| ------------- |:---------:|
| BASE_NAME     | char *    |
| BASE_TIME     | double    |
| BASE_UNIT     | char *    |
| BASE_VALUE    | double    |
| BASE_SUM      | double    |
| BASE_VERSION  | double    |
| NAME          | char *    |
| UNIT          | char *    |
| VALUE         | double    |
| STRING_VALUE  | char *    |
| BOOLEAN_VALUE | int (*)   |
| DATA_VALUE    | char *    |
| SUM           | double    |
| TIME          | double    |
| UPDATE_TIME   | double    |

(*) 0 = false, all other values = true


### Macros:
```c
// Creates and begins new SenML message in JSON format, given a buffer and its size
INIT_SENML_JSON(char* buffer_pointer, int size)

// Creates and begins new SenML message in CBOR format
INIT_SENML_CBOR(char* buffer_pointer, int size)

// Adds a record with the given fields
// For example 
// ADD_RECORD(BASE_NAME, "name", BASE_UNIT, "unit", VALUE, 4.6)
// adds a record with the fields bn = name, bu = unit, v = 4.6
ADD_RECORD(...)

// Ends the SenML message
END_SENML()
```

## Example usage
```c
static char buffer[1024];
char* buf_ptr = buffer;
INIT_SENML_JSON(buf_ptr, 1024);
ADD_RECORD(BASE_NAME, "name", BASE_TIME, );
ADD_RECORD(BASE_NAME, "test_name"
```


## Authors
TODO