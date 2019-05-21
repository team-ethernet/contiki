#include <stdlib.h>
#include <string.h>
#include "jsonparse.h"
#include "senml-decode.h"

/* REQUIRES JSON APP - MAKE SURE TO INCLUDE IT IN YOUR PROJECT ASWELL */

struct jsonparse_state state;

char *label_buf;
char *value_buf;

char *oldLabelAddr = 0;
void *oldValueAddr = 0;
char *oldAddBufAddr = 0;

int length(char* string){
	int i;
	for(i = 0; string[i] != '\0'; ++i);
    return i;
}

void init_json_decoder(char* msg){
	state.json = msg;
	state.pos = 0;
	state.len = length(msg);
	jsonparse_setup(&state, state.json, state.len);
	jsonparse_next(&state);
	jsonparse_next(&state);
}

void read_next_token(struct pair *token){
	if(oldLabelAddr != 0){
		free(oldLabelAddr);
	}
	if(oldValueAddr != 0){
		free(oldValueAddr);
	}
	int parseNextNumber = jsonparse_next(&state);
	if(parseNextNumber == 44){ //Reads comma
		jsonparse_next(&state);
		int elem_len = jsonparse_get_len(&state);
		int totlen = elem_len;
		elem_len++;
		char* label_buf = malloc(sizeof(char)*elem_len);
		jsonparse_copy_value(&state, label_buf, elem_len);
		
		jsonparse_next(&state);
		elem_len = jsonparse_get_len(&state);
		totlen = totlen+elem_len;
		elem_len++;
		char* value_buf = malloc(sizeof(char)*elem_len);
		jsonparse_copy_value(&state, value_buf, elem_len);
		
		token->label = label_buf;
		token->value = value_buf;
		oldLabelAddr = label_buf;
		oldValueAddr = value_buf;
		state.json = state.json+state.pos;
		state.len = state.len-state.pos;
		state.pos = 0;
	} else if(parseNextNumber == 125){ //Reads }
		if(jsonparse_next(&state) == 93){ //If ]
			token->label = NULL;
			token->value = NULL;
		} else{ 	//If not ] then it has to be comma 
			jsonparse_next(&state);
			jsonparse_next(&state);
			int elem_len = jsonparse_get_len(&state);
			int totlen = elem_len;
			elem_len++;
			char* label_buf = malloc(sizeof(char)*elem_len);
			jsonparse_copy_value(&state, label_buf, elem_len);
			
			jsonparse_next(&state);
			elem_len = jsonparse_get_len(&state);
			totlen = totlen+elem_len;
			elem_len++;
			char* value_buf = malloc(sizeof(char)*elem_len);
			jsonparse_copy_value(&state, value_buf, elem_len);
			
			token->label = label_buf;
			token->value = value_buf;
			oldLabelAddr = label_buf;
			oldValueAddr = value_buf;
			state.json = state.json+state.pos;
			state.len = state.len-state.pos;
			state.pos = 0;
		}
	} else{ //Reads a label+value
		int elem_len = jsonparse_get_len(&state);
		elem_len++;
		char* label_buf = malloc(sizeof(char)*elem_len);
		jsonparse_copy_value(&state, label_buf, elem_len);
		
		jsonparse_next(&state);
		elem_len = jsonparse_get_len(&state);
		elem_len++;
		char* value_buf = malloc(sizeof(char)*elem_len);
		jsonparse_copy_value(&state, value_buf, elem_len);
		
		token->label = label_buf;
		token->value = value_buf;
		oldLabelAddr = label_buf;
		oldValueAddr = value_buf;
		state.json = state.json+state.pos;
		state.len = state.len-state.pos;
		state.pos = 0;
	}
  }

void add_new_msg(char* msg){
	if(oldAddBufAddr != 0){
		free(oldAddBufAddr);
	}
	int stlen = state.len;
	int msglen = length(msg);
	int totlen = stlen+msglen;
	char *buf = malloc(sizeof(char)*(totlen+1));
	strcpy(buf, state.json);
	strcat(buf, msg);
	state.json = buf;
	state.len = totlen;
	oldAddBufAddr = buf;
}