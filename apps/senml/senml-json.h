#ifndef SENML_JSON_H_
#define SENML_JSON_H_
#include "senml-json-formatter.h"
#include "senml-api.h"
#define INIT_SENML_JSON(buffer_pointer, size) init_senml(buffer_pointer, size, senml_json_formatter)
#endif