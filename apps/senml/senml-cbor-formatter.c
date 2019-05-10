#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "label.h"
#include "senml-formatter.h"

// translated SenML labels to CBOR labels according to RFC8428 page 19
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
// 0x9F stands for start of indefinite length array
int start_pack_cbor(char * buffer, int buffer_len){
        return snprintf(buffer, buffer_len,"%c", 0x9F);
}
// 0xFF stands for "break" indefinite length array
int end_pack_cbor(char * buffer, int buffer_len){
        return snprintf(buffer, buffer_len,"%c", 0xFF);;
}
// 0xBF stands for start indefinite length map
int start_record_cbor(char * buffer, int buffer_len){
        return snprintf(buffer, buffer_len,"%c", 0xBF);
}
// 0xFF stands for "break" indefinite length map
int end_record_cbor(char * buffer, int buffer_len){
        return snprintf(buffer, buffer_len,"%c", 0xFF);
}
// max int value or length of string is 65535. Can be extended if needed.
int data_type_cbor_convert(char * buffer, int buffer_len, unsigned char type,  int value)
{
        uint8_t len = 0;

        if (value < 24) {
                len += snprintf(buffer, buffer_len, "%c",  type | (unsigned char)value);
        }
        else if (value < 256) {
                len += snprintf(buffer, buffer_len, "%c", type | 0x18 );
                len += snprintf(&buffer[len], buffer_len - len, "%c", (unsigned char)value);
        }
        else if (value < 65536) {
                len += snprintf(buffer, buffer_len, "%c", type | 0x19 );

                int i;
                for (i = 0; i < 2; i++) {
                        len += snprintf(&buffer[len], buffer_len - len, "%c", (uint8_t)((value >> 8*(1 - i)) & 0xFF));
                }
        }
        return len;
}

int append_str_field_cbor(char * buffer, int buffer_len, Label label, char * value)
{
        uint8_t len = 0;

        len += snprintf(&buffer[len], buffer_len - len,"%c", label_cbor[label]);

        int number_of_chars = 0;
        while (*value != '\0') {
                number_of_chars++;
                value++;
        }
        // 0x60 for text
        len += data_type_cbor_convert(&buffer[len], buffer_len - len, 0x60, number_of_chars);

        value = value - number_of_chars;
        while (*value != '\0') {
                len += snprintf(&buffer[len], buffer_len - len, "%c", *value);
                value++;
        }
        return len;
}

int append_int_field_cbor(char * buffer, int buffer_len, Label label, int value)
{
        uint8_t len = 0;

        len += snprintf(&buffer[len], buffer_len - len,"%c", label_cbor[label]);

        // 0x00 for positive/unsigned int
        len += data_type_cbor_convert(&buffer[len], buffer_len - len, 0x00, value);

        return len;
}

int append_dbl_field_cbor(char * buffer, int buffer_len, Label label, double value)
{
        uint8_t len = 0;

        len += snprintf(&buffer[len], buffer_len - len,"%c", label_cbor[label]);
        // 0xFB for start of double value
        len += snprintf(&buffer[len], buffer_len - len, "%c", 0xFB);

        union {
                double d_val;
                uint64_t u_val;
        } u64;

        u64.d_val = value;

        int i;
        for (i = 0; i < 8; i++) {
                len += snprintf(&buffer[len], buffer_len - len, "%c", (uint8_t)((u64.u_val >> 8*(7 - i)) & 0xFF));
        }
        return len;
}

int append_bool_field_cbor(char * buffer, int buffer_len, Label label, int value)
{
        uint8_t len = 0;

        len += snprintf(&buffer[len], buffer_len - len,"%c", label_cbor[label]);

        if (value == 0) {
                len += snprintf(&buffer[len], buffer_len - len,"%c", 0xF4);
        } else {
                len += snprintf(&buffer[len], buffer_len - len,"%c", 0xF5);
        }
        return len;
}

const struct senml_formatter senml_cbor_formatter = {
        start_record_cbor,
        end_record_cbor,
        start_pack_cbor,
        end_pack_cbor,
        append_str_field_cbor,
        append_dbl_field_cbor,
        append_bool_field_cbor,
        append_int_field_cbor
};
