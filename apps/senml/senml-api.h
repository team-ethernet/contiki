#ifndef SENML_API_H_
#define SENML_API_H_
#include "label.h"
#include "senml-formatter.h"

#define SENML_START_PACK(buf_ptr, buf_len) senml_start_pack(buf_ptr, buf_len)
#define SENML_END_PACK(buf_ptr, buf_len) senml_end_pack(buf_ptr, buf_len)

#define SENML_ADD_RECORD(buf_ptr, buf_len, ...) senml_add_record(buf_ptr, buf_len, __VA_ARGS__, END)

void senml_init(struct senml_formatter frmttr);
int senml_end_pack(char *buf_ptr, int buf_len);
int senml_start_pack(char *buf_ptr, int buf_len);
int senml_add_record(char *buf_ptr, int buf_len, Label label, ...);
#endif