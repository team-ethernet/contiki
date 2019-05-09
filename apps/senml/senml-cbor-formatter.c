#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "label.h"


// translated labels from string to CBOR according to RFC8428 page 19
static const unsigned char label_cbor[] = {
      0x21,
      0x22,
      0x23,
      0x24,
      0x25,
      0x20,
      0x00,
      0x01,
      0x02,
      0x03,
      0x04,
      0x08,
      0x05,
      0x06,
      0x07
  };
 //                  { “bn”, “bt”, “bu”, “bv”, “bs”, “bver”, “n”, “u”, “v”, “vs”, “vb”, “vd”, ”s”, “t”, “ut”};

// 0x9F stands for start of indefinite length array
int start_pack(char * buffer, int buffer_len){
  return snprintf(buffer, buffer_len,"%c", 0x9F);
}
// 0xFF stands for "break" indefinite length array
int end_pack(char * buffer, int buffer_len){
  return snprintf(buffer, buffer_len,"%c", 0xFF);;
}
// 0xBF stands for start indefinite length map
int start_record(char * buffer, int buffer_len){
  uint8_t len = 0;
  return snprintf(buffer, buffer_len,"%c", 0xBF);
}
// 0xFF stands for "break" indefinite length map
int end_record(char * buffer, int buffer_len){
  return snprintf(buffer, buffer_len,"%c", 0xFF);
}

int convert(char * buffer, int buffer_len, unsigned char type,  int value)
{
  uint8_t len = 0;

  if (value < 24){
    len += snprintf(buffer, buffer_len, "%c",  type | (unsigned char)value);
  }
  else if (value < 256){
    len += snprintf(buffer, buffer_len, "%c", type | 0x18 );
    len += snprintf(&buffer[len], buffer_len - len, "%c", (unsigned char)value);
  }
  return len;

}
int append_str_field(char * buffer, int buffer_len, Label label, char * value)
{
  uint8_t len = 0;

  len += snprintf(&buffer[len], buffer_len - len,"%c", label_cbor[label]);

  int number_of_chars = 0;
  char *copy = value;
  while (*copy != '\0'){
    number_of_chars ++;
    copy ++;
  }
  // 0x60 for text
  len += convert(&buffer[len], buffer_len - len, 0x60, number_of_chars);

  while (*value != '\0'){
      len += snprintf(&buffer[len], buffer_len - len, "%c", *value);
      printf("%02x\n",*value );
      value ++;
  }
  return len;
}

int append_dbl_field(char * buffer, int remaining_buffer, Label label, double value)
{
  uint8_t len = 0;

  return len;
}

int append_bool_field(char * buffer, int buffer_len, Label label, int value)
{
  uint8_t len = 0;

  len += snprintf(&buffer[len], buffer_len - len,"%c", label_cbor[label]);

  if (value == 0){
    len += snprintf(&buffer[len], buffer_len - len,"%c", 0xF4);
  } else {
    len += snprintf(&buffer[len], buffer_len - len,"%c", 0xF5);
  }
  return len;
}

int main(int argc, char const *argv[]) {

  uint8_t len = 0;
  char buffer [500];

  len += start_pack(buffer, sizeof(buffer));

  printf("len start pack %d\n", len);
  for (int i = 0; i < len; i++) {
    printf("%02x", (unsigned char)buffer[i]);  //unsigned otherwise FF becomes ffffff

  }
  len += start_record(&buffer[len], sizeof(buffer) - len);
    printf("len start record %d\n", len);
  for (int i = 0; i < len; i++) {
    printf("%02x", (unsigned char)buffer[i]);  //unsigned otherwise FF becomes ffffff

  }
    printf("\n");
  //printf("base_name: \n");
  len += append_str_field(&buffer[len], sizeof(buffer) - len, BASE_NAME, "123456789abcdef123456789abcdef");
  printf("len app str %d\n", len);
  for (int i = 0; i < 31; i++) {
    printf("%02x", (unsigned char)buffer[i]);  //unsigned otherwise FF becomes ffffff

  }
  printf("\n");

  for (int i = 0; i < len; i++) {
    printf("%02x", (unsigned char)buffer[i]);  //unsigned otherwise FF becomes ffffff

  }
    printf("\n");
  len += append_bool_field(&buffer[len], sizeof(buffer) - len, BOOLEAN_VALUE, 1);
  printf("len app bool %d\n", len);

  for (int i = 0; i < len; i++) {
    printf("%02x", (unsigned char)buffer[i]);  //unsigned otherwise FF becomes ffffff

  }
    printf("\n");
  len += end_record(&buffer[len], sizeof(buffer) - len);
  printf("len end record %d\n", len);
  for (int i = 0; i < len; i++) {
    printf("%02x", (unsigned char)buffer[i]);  //unsigned otherwise FF becomes ffffff

  }
    printf("\n");
  len += start_record(&buffer[len], sizeof(buffer) - len);
  printf("len start record %d\n", len);

  for (int i = 0; i < len; i++) {
    printf("%02x", (unsigned char)buffer[i]);  //unsigned otherwise FF becomes ffffff

  }
    printf("\n");
  // printf("bool_value: \n");
  // for (int i = 0; i < 2; i++) {
  //   printf("%02x", buffer2[i] & 0xFF);
  // }
  //
  // printf("\n");


  printf("string_value: \n");
  len += append_str_field(&buffer[len], sizeof(buffer) - len, STRING_VALUE, "hello");
  printf("len app str %d\n", len);
  for (int i = 0; i < len; i++) {
    printf("%02x", (unsigned char)buffer[i]);  //unsigned otherwise FF becomes ffffff

  }
    printf("\n");
  len += end_record(&buffer[len], sizeof(buffer) - len);
    printf("len end record %d\n", len);
  for (int i = 0; i < len; i++) {
    printf("%02x", (unsigned char)buffer[i]);  //unsigned otherwise FF becomes ffffff

  }
    printf("\n");
  len += end_pack(&buffer[len], sizeof(buffer) - len);
    printf("len end pack %d\n", len);

  for (int i = 0; i < len; i++) {
    printf("%02x", (unsigned char)buffer[i]);  //unsigned otherwise FF becomes ffffff

  }
  printf("\n");

  for (int i = 0; i < len; i++) {
    printf("%c", buffer[i]);  //unsigned otherwise FF becomes ffffff

  }


  return 0;
}
