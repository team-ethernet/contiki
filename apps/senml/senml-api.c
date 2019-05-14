#include <stdarg.h>
#include <stdio.h>
#include "label.h"
#include "senml-formatter.h"

struct senml_formatter formatter;

void init_senml(struct senml_formatter frmttr){
	formatter = frmttr;
}

int start_senml_pack_stream(char * buf_ptr, int buf_len){
	return formatter.start_pack(buf_ptr, buf_len);
}

int end_senml_pack_stream(char * buf_ptr, int buf_len){
	return formatter.end_pack(buf_ptr, buf_len);
}

int add_record(char * buf_ptr, int buf_len, Label label, ...) {
	uint8_t len = 0;
	va_list args;
	va_start(args, label);
	len += formatter.start_record(buf_ptr, buf_len);
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
				len += formatter.append_str_field(&buf_ptr[len], buf_len - len, label, str);
			}
			case BASE_TIME:
			case BASE_VALUE:
			case BASE_SUM:
			case VALUE:
			case SUM:
			case TIME:
			case UPDATE_TIME:
			{
				double dbl = va_arg(args, double);
				len += formatter.append_dbl_field(&buf_ptr[len], buf_len - len, label, dbl);
			}
			case BOOLEAN_VALUE:
			{
				int b = va_arg(args, int);
				len += formatter.append_bool_field(&buf_ptr[len], buf_len - len, label, b);
			}
			case BASE_VERSION:
			{
				int i = va_arg(args, int);
				len += formatter.append_int_field(&buf_ptr[len], buf_len - len, label, i);
			}
			default:
				break;
		}
		
		label = va_arg(args, Label);

	}
	len += formatter.end_record(&buf_ptr[len], buf_len - len);
	return len;
}
