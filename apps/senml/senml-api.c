#include <stdarg.h>
#include <stdio.h>
#include "label.h"
#include "senml-formatter.h"

int len = 0;
int remaining = 0;
char * buf_ptr;

struct senml_formatter formatter;

#define BUFFER(len) { \
	if (len < 0 || len >= remaining) { \
		printf("SenML: Buffer too short. Have %d, need %d + \\0", remaining, len); \
		return; \
	} \
	remaining -= len; \
	buf_ptr += len; \
}

void init_senml(char * buffer_pointer, int size, struct senml_formatter frmttr){
	buf_ptr = buffer_pointer;
	remaining = size;
	formatter = frmttr;
	BUFFER(formatter.start_pack(buf_ptr, remaining));
}

void end_senml(){
	BUFFER(formatter.end_pack(buf_ptr, remaining));
}

void add_record(Label label, ...) {
	va_list args;
	va_start(args, label);
	BUFFER(formatter.start_record(buf_ptr, remaining));
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
				BUFFER(formatter.append_str_field(buf_ptr, remaining, label, str));
			}
			    break;
			case BASE_TIME:
			case BASE_VALUE:
			case BASE_SUM:
			case VALUE:
			case SUM:
			case TIME:
			case UPDATE_TIME:
			{
				double dbl = va_arg(args, double);
				BUFFER(formatter.append_dbl_field(buf_ptr, remaining, label, dbl));
			}
			  	break;
			case BOOLEAN_VALUE:
			{
				int b = va_arg(args, int);
				BUFFER(formatter.append_bool_field(buf_ptr, remaining, label, b));
			}
				break;
			case BASE_VERSION:
			{
				int i = va_arg(args, int);
				BUFFER(formatter.append_int_field(buf_ptr, remaining, label, i));
			}
				break;
			default:
				break;
		}
		
		label = va_arg(args, Label);

	}
	BUFFER(formatter.end_record(buf_ptr, remaining));
}
