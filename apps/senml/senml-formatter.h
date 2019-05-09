#ifndef SENML_OUTPUT_H_
#define SENML_OUTPUT_H_
#include "label.h"

struct senml_formatter
{
    int    (* start_record)         (char * buf_ptr, int remaining);
    int    (* end_record)           (char * buf_ptr, int remaining);
    int    (* start_pack)           (char * buf_ptr, int remaining);
    int    (* end_pack)             (char * buf_ptr, int remaining);
    int    (* append_str_field)     (char * buf_ptr, int remaining, Label label, char * str);
    int    (* append_dbl_field)     (char * buf_ptr, int remaining, Label label, double dbl);
    int    (* append_bol_field)     (char * buf_ptr, int remaining, Label label, int bol);
    int    (* append_int_field)     (char * buf_ptr, int remaining, Label label, int i);
};
#endif