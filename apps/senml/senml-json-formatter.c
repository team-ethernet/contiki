#include "label.h"
#include <stdio.h>
#include "senml-formatter.h"

static const char *label_strings[] = {
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

int
start_pack_json(char *buf_ptr, int remaining)
{
  return snprintf(buf_ptr, remaining, "[");
}
int
end_pack_json(char *buf_ptr, int remaining)
{
  buf_ptr -= sizeof(char);

  if(buf_ptr[0] == ',') {
    return snprintf(buf_ptr, remaining, "]") - 1;
  } else {
    buf_ptr += sizeof(char);
    return snprintf(buf_ptr, remaining, "]");
  }
}
int
start_record_json(char *buf_ptr, int remaining)
{
  return snprintf(buf_ptr, remaining, "{");
}
int
end_record_json(char *buf_ptr, int remaining)
{
  buf_ptr -= sizeof(char);
  return snprintf(buf_ptr, remaining, "},") - 1;
}
int
append_str_field_json(char *buf_ptr, int remaining, Label label, char *str)
{
  return snprintf(buf_ptr, remaining, "\"%s\":\"%s\",", label_strings[label], str);
}
int
append_dbl_field_json(char *buf_ptr, int remaining, Label label, float flt)
{
  return snprintf(buf_ptr, remaining, "\"%s\":%g,", label_strings[label], flt);
}
int
append_bool_field_json(char *buf_ptr, int remaining, Label label, int b)
{
  if(b) {

    return snprintf(buf_ptr, remaining, "\"%s\":true,", label_strings[label]);
  } else {
    return snprintf(buf_ptr, remaining, "\"%s\":false,", label_strings[label]);
  }
}
int
append_int_field_json(char *buf_ptr, int remaining, Label label, int i)
{
  return snprintf(buf_ptr, remaining, "\"%s\":%d,", label_strings[label], i);
}
const struct senml_formatter senml_json_formatter = {
  start_record_json,
  end_record_json,
  start_pack_json,
  end_pack_json,
  append_str_field_json,
  append_dbl_field_json,
  append_bool_field_json,
  append_int_field_json
};