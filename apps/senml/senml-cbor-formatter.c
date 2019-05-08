#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "label.h"

// typedef enum {
// 	BASE_NAME,
// 	BASE_TIME,
// 	BASE_UNIT,
// 	BASE_VALUE,
// 	BASE_SUM,
// 	BASE_VERSION,
//   NAME,
// 	UNIT,
// 	VALUE,
// 	STRING_VALUE,
// 	BOOLEAN_VALUE,
// 	DATA_VALUE,
// 	SUM,
// 	TIME,
// 	UPDATE_TIME//,
// 	//CUSTOM
// }label;

// translated labels from string to CBOR according to RFC8428 page 19
static const unsigned char label_cbor[] = {0x21, 0x22, 0x23, 0x24, 0x25, 0x20, 0x00, 0x01, 0x02, 0x03, 0x04, 0x08, 0x05, 0x06, 0x07};
 //                  { “bn”, “bt”, “bu”, “bv”, “bs”, “bver”, “n”, “u”, “v”, “vs”, “vb”, “vd”, ”s”, “t”, “ut”};

unsigned char * convert(unsigned char type,  int value)
{
  unsigned char * output;
  if (value < 24){
    *output = type | (unsigned char)value;
  }

  if (value < 256){
    *output = type | 0x18;
    //printf("%x\n",(unsigned char)*output);
    output++;
    *output = (unsigned char)value;
    //printf("%x\n",(unsigned char)*output);
    output--;
  }
  return output;

}
void append_str_field(char * buffer, int buffer_len, Label label, char * value)
{
  uint8_t len = 0;

  len += snprintf(&buffer[len], buffer_len - len,"%c", label_cbor[label]);

  int number_of_chars = 0;
  char *copy = value;
  while (*copy != '\0'){
    number_of_chars ++;
    copy ++;
  }

  unsigned char * text_string = convert(0x60, number_of_chars);
  len += snprintf(&buffer[len], buffer_len - len, "%s", text_string);

  while (*value != '\0'){
      len += snprintf(&buffer[len], buffer_len - len, "%c", *value);
      value ++;
  }
  return;
}

void append_dbl_field(char * buffer, int remaining_buffer, Label label, double value)
{


}

void append_bool_field(char * buffer, int buffer_len, Label label, int value)
{
  uint8_t len = 0;
    printf("label_cbor: %02x\n", label_cbor[label] );

  len += snprintf(&buffer[len], buffer_len - len,"%c", label_cbor[label]);

  if (value == 0){
    snprintf(&buffer[len], buffer_len - len,"%c", 0xF4);
  } else {
      snprintf(&buffer[len], buffer_len - len,"%c", 0xF5);
  }
  return;

}

int main(int argc, char const *argv[]) {

  char buffer [500];
  printf("base_name: \n");
  append_str_field(buffer, sizeof(buffer), BASE_NAME, "123456789abcdef123456789abcdef");
  for (int i = 0; i < 31; i++) {
    printf("%02x", (unsigned char)buffer[i]);  //unsigned otherwise FF becomes ffffff

  }
  printf("\n");

  char buffer2 [8];

  append_bool_field( buffer2, sizeof(buffer2), BOOLEAN_VALUE, 1);

  printf("bool_value: \n");
  for (int i = 0; i < 2; i++) {
    printf("%02x", buffer2[i] & 0xFF);
  }

  printf("\n");

  char buffer3 [8];
  printf("string_value: \n");
  append_str_field( buffer3, sizeof(buffer3), STRING_VALUE, "hello");
  for (int i = 0; i < 20; i++) {
    printf("%02x", (unsigned char)buffer3[i]);  //unsigned otherwise FF becomes ffffff

  }
  printf("\n");



  return 0;
}
