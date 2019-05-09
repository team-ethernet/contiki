#ifndef SENML_API_H_
#define SENML_API_H_
#include "label.h"
#include "senml-formatter.h"

#define END_SENML() end_senml()

#define ADD_RECORD(...) add_record(__VA_ARGS__, END)

void init_senml(char * buffer_pointer, int size, struct senml_formatter frmttr);
void end_senml();
void add_record(Label label, ...);
#endif