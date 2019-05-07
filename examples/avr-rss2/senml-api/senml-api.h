#ifndef SENML_API_H_
#define SENML_API_H_
#include "label.h"

void init_pack(char * buffer_pointer, int size);
void end_pack();
void add_record(Label label, ...);
#endif