#include <stdarg.h>
#include "label.h"
#include "senml-json.h"
#include "senml-json.c"

int len = 0;
int remaining = 0;
char * buf_ptr;

// #define PUTFMT(...) { \
// 		len = snprintf(buf_ptr, remaining, __VA_ARGS__);	\
// 		if (len < 0 || len >= remaining) { \
// 			printf("SenML: Buffer too short. Have %d, need %d + \\0", remaining, len); \
// 			return; \
// 		} \
// 		remaining -= len; \
// 		buf_ptr += len; \
// 	}

#define BUFFER(len) { \
		if (len < 0 || len >= remaining) { \
			printf("SenML: Buffer too short. Have %d, need %d + \\0", remaining, len); \
			return; \
		} \
		remaining -= len; \
		buf_ptr += len; \
	}

void init_pack(char * buffer_pointer, int size){
	buf_ptr = buffer_pointer;
	remaining = size;
	BUFFER(senml_json.start_pack(buf_ptr, remaining));
}

void end_pack(){
	BUFFER(senml_json.end_pack(buf_ptr, remaining));
}

void add_record(Label label, ...) {
	va_list args;
	va_start(args, label);
	BUFFER(senml_json.start_record(buf_ptr, remaining));
	while(label != END) {
		switch(label) {
			case BASE_NAME:
			case BASE_UNIT:
			case UNIT:
			case NAME:
			case STRING_VALUE:
			case DATA_VALUE:
			{
				char * str = va_arg(args, char *);
				BUFFER(senml_json.append_str_field(buf_ptr, remaining, label, str));
			}
			    break;
			case BASE_TIME:
			case BASE_VALUE:
			case BASE_SUM:
			case BASE_VERSION:
			case VALUE:
			case SUM:
			case TIME:
			case UPDATE_TIME:
			{
				double dbl = va_arg(args, double);
				BUFFER(senml_json.append_dbl_field(buf_ptr, remaining, label, dbl));
			}
			  	break;
			case BOOLEAN_VALUE:
			{
				int bol = va_arg(args, int);
				BUFFER(senml_json.append_bol_field(buf_ptr, remaining, label, bol));
			}
				break;
			default:
				break;
		}
		
		label = va_arg(args, Label);

	}
	BUFFER(senml_json.end_record(buf_ptr, remaining));
}
