#include "senml-json-formatter.h"
#include "senml-api.h"

#define INIT_SENML(buffer_pointer, size, frmttr) init_senml(buffer_pointer, size, frmttr)

#define END_SENML() end_senml()

#define ADD_RECORD(...) add_record(__VA_ARGS__, END)

