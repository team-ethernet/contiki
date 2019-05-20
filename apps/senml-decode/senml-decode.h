#ifndef SENML_DECODE_H_
#define SENML_DECODE_H_

/* Label-value pair struct */
struct pair {
	char* label;
	void* value;
};

/* Initializes state struct and returns it, use it when calling read and add */
void init_json_decoder(char* msg);

/* Reads and returns the next label-value pair as a struct */
void read_next_token(struct pair *token);

/* Adds msg to the end of the JSON string. Stream friendly */
void add_new_msg(char* msg);

#endif /* JSONPARSE_H_ */