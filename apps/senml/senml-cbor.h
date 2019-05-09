#ifndef SENML_CBOR_H_
#define SENML_CBOR_H_
#include "senml-cbor-formatter.h"
#include "senml-api.h"
#define INIT_SENML_CBOR(buffer_pointer, size) init_senml(buffer_pointer, size, senml_cbor_formatter)
#endif