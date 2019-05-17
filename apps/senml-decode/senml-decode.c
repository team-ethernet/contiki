#include "jsonparse.h"
#include <string.h>

/* REQUIRES JSON APP - MAKE SURE TO INCLUDE IT IN YOUR PROJECT ASWELL */

char* concat(const char* s1, const char* s2)
{
	int lens1 = length(s1);
	int lens2 = length(s2);
	char returnbuffer[lens1+lens2];
	int currpos = 0;
	for(int i = 0; i < lens1; i++){
		returnbuffer[i] = s1[i];
		currpos++;
	}
	for(int i = 0; i < lens2; i++){
		returnbuffer[currpos] = s2[i];
		currpos++;
	}
	return returnbuffer;
}

int length(char* string){
	int i;
	for(i = 0; string[i] != '\0'; ++i);
    return i;
}

struct pair{
	char* label;
	void* value;
};

struct jsonparse_state init_json_decoder(char* msg){
	struct jsonparse_state state = {.json = msg, .pos = 2, .len = length(msg)};
	jsonparse_setup(&state, state.json, state.len);
	return state;
}

struct pair read_next_token(struct jsonparse_state *state){
	
	if(jsonparse_next(&state) != 125){
		int elem_len = jsonparse_get_len(&state);
		char* label_buf[elem_len];
		jsonparse_copy_value(&state, label_buf, elem_len);
		jsonparse_next(&state);
		elem_len = jsonparse_get_len(&state);
		char* value_buf[elem_len];
		jsonparse_copy_value(&state, value_buf, elem_len);
		struct pair lv = {.label = label_buf, .value = value_buf};
		jsonparse_next(&state);
		return lv;
	}
	else {
		struct pair lvnull = {.label = NULL, .value = NULL};
		return lvnull;
	}
}

void add_new_msg(struct jsonparse_state *state, char* msg){
	char* jsonmsg = state->json;
	state->json = concat(jsonmsg, msg);
	state->len = state->len + length(msg);
}