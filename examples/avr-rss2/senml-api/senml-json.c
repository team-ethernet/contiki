#include "label.h"
#include "senml-json.h"
#include <stdio.h>

static const char * label_strings[] = { 
	"bn", 
	"bt", 
	"bu", 
	"bv", 
	"bs", 
	"bver", 
	"n", 
	"u", 
	"v", 
	"vs", 
	"vb", 
	"vd", 
	"s", 
	"t", 
	"ut"
};

int start_pack(char * buf_ptr, int remaining) {
    return snprintf(buf_ptr, remaining, "[");
}

int end_pack(char * buf_ptr, int remaining) {
	buf_ptr -= sizeof(char);
    return snprintf(buf_ptr, remaining, "]") - 1;
}

int start_record(char * buf_ptr, int remaining) {
    return snprintf(buf_ptr, remaining, "{");
}

int end_record(char * buf_ptr, int remaining) {
	buf_ptr -= sizeof(char);
    return snprintf(buf_ptr, remaining, "},") - 1;
}

int append_str_field(char * buf_ptr, int remaining, Label label, char * str) {
    return snprintf(buf_ptr, remaining, "\"%s\":\"%s\",", label_strings[label], str);
}

int append_dbl_field(char * buf_ptr, int remaining, Label label, double dbl){
    return snprintf(buf_ptr, remaining, "\"%s\":\"%f\",", label_strings[label], dbl);
}

int append_bol_field(char * buf_ptr, int remaining, Label label, int bol){
	if(bol) {

		return snprintf(buf_ptr, remaining, "\"%s\":\"true\",", label_strings[label]);
	}
	else {
		return snprintf(buf_ptr, remaining, "\"%s\":\"false\",", label_strings[label]);
	}
}

const struct senml_output senml_json = { 
    start_record, 
    end_record, 
	start_pack,
	end_pack,
    append_str_field, 
    append_dbl_field, 
    append_bol_field
};