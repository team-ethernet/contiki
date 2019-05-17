#ifndef SENML_API_H_
#define SENML_API_H_
#include "label.h"
#include "senml-formatter.h"

#define END_SENML_PACK_STREAM(buf_ptr, buf_len) end_senml_pack_stream(buf_ptr, buf_len) 
#define START_SENML_PACK_STREAM(buf_ptr, buf_len) start_senml_pack_stream(buf_ptr, buf_len) 

#define ADD_RECORD(buf_ptr, buf_len, ...) add_record(buf_ptr, buf_len, __VA_ARGS__, END)

void init_senml(struct senml_formatter frmttr);
int end_senml_pack_stream(char * buf_ptr, int buf_len);
int start_senml_pack_stream(char * buf_ptr, int buf_len);
int add_record(char* buf_ptr, int buf_len, Label label, ...);
#endif