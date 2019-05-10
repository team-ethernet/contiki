# senml app
The senml app is an API for creating SenML messages in JSON or CBOR format.  
It handles the formatting of the SenML pack conforming to [RFC 8428](https://tools.ietf.org/html/rfc8428).

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
| BASE_VERSION  | int       |
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

Maximum int value and string length is 65535 in CBOR formatter (can be extended if needed).

### Macros
```c
// Creates and begins new SenML message in JSON format, given a buffer and its size
INIT_SENML_JSON(char* buffer_pointer, int size)
```
```c
// Creates and begins new SenML message in CBOR format
INIT_SENML_CBOR(char* buffer_pointer, int size)
```
```c
// Adds a record with the given fields
// For example 
// ADD_RECORD(BASE_NAME, "name", BASE_UNIT, "unit", VALUE, 4.6)
// adds a record with the fields bn = name, bu = unit, v = 4.6
ADD_RECORD(...)
```
```c
// Ends the SenML message
END_SENML()
```

## Example usage
```c
static char buffer[1024];
char* buf_ptr = buffer;

INIT_SENML_JSON(buf_ptr, 1024);
ADD_RECORD(BASE_NAME, "urn:dev:ow:10e2073a01080063", NAME, "voltage", UNIT, "V", VALUE, 120.1);
ADD_RECORD(NAME, "current", UNIT, "A", VALUE, 1.2);
END_RECORD();

printf(buffer);
```
Should print
```json
[{"bn":"urn:dev:ow:10e2073a01080063:","n":"voltage","u":"V","v":120.100000},{"n":"current","u":"A","v":1.200000}]
```
## Code structure
The different lables are defined in `label.h`.  
The main code that handles the different labels is in `senml-api.c`.  
Formatting is handled in `senml-cbor-fomatter.c` and `senml-json-formatter.c`, both implementing the functions defined in `senml-formatter.h`  
User macros defined in `senml-json.h` and `senml-cbor.h` provide a formatter to the main functions in `senml-api.c`.  

## TODO
Implementation of XML and EXI.  

## Authors
Anton Bothin  
Erik Flink  
Nelly Friman  
Jacob Klasmark  
Valter Lundegårdh  
Isak Olsson  
Andreas Sjödin  
Carina Wickström