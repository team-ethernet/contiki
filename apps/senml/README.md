senml app
=======
TODO

senml Code
--------
apps/senml/senml-api.c
apps/senml/senml-json-formatter.c


Project code needs
------------------
#include "senml-api.h"
#include "senml-json-formatter.h"

Build
-----
To include in project:
in Makefile add

APPS += senml

Use
---
void init_senml(char * buffer_pointer, int size, struct senml_formatter frmttr);
void end_senml();
void add_record(Label label, ...);
TODO

Authors
--------
TODO