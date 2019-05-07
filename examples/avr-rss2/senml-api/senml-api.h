#ifndef SENML_API_H_
#define SENML_API_H_
#include "label.h"

void init_senml(char * buffer_pointer, int size);
void end_senml();
void add_record(Label label, ...);
#endif