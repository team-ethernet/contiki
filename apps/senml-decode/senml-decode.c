#include "jsonparse.h"
#include <string.h>
#include "senml-decode.h"
#include <stdio.h>

/* REQUIRES JSON APP - MAKE SURE TO INCLUDE IT IN YOUR PROJECT ASWELL */

struct jsonparse_state state;

char *label_buf;
char *value_buf;

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
	//printf("INIT state.json: %s\n", state.json);
}

//changes the input struct "token" to the next values
void read_next_token(struct pair *token){
	
	printf("address to token: %d\n", &token);
	
	//printf("READ state.pos: %d\n", state.pos);
	int parseNextNumber = jsonparse_next(&state);
	
	//Reads comma
	if(parseNextNumber == 44){
		jsonparse_next(&state);
		int elem_len = jsonparse_get_len(&state);
		elem_len++;
		char* label_buf = malloc(sizeof(char)*elem_len);
		//char label_buf[elem_len];
		jsonparse_copy_value(&state, label_buf, elem_len);
		
		jsonparse_next(&state);
		elem_len = jsonparse_get_len(&state);
		elem_len++;
		char* value_buf = malloc(sizeof(char)*elem_len);
		//char value_buf[elem_len];
		jsonparse_copy_value(&state, value_buf, elem_len);
		
		token->label = label_buf;
		token->value = value_buf;
		
	} else if(parseNextNumber == 125){ //Reads }
		//If ]
		if(jsonparse_next(&state) == 93){
			token->label = NULL;
			token->value = NULL;
		}else{ 	//If not ] then it has to be comma 
			jsonparse_next(&state);
			jsonparse_next(&state);
			int elem_len = jsonparse_get_len(&state);
			elem_len++;
			char* label_buf = malloc(sizeof(char)*elem_len);
			//char label_buf[elem_len];
			jsonparse_copy_value(&state, label_buf, elem_len);
			
			jsonparse_next(&state);
			elem_len = jsonparse_get_len(&state);
			elem_len++;
			char* value_buf = malloc(sizeof(char)*elem_len);
			//char value_buf[elem_len];
			jsonparse_copy_value(&state, value_buf, elem_len);

			
			token->label = label_buf;
			token->value = value_buf;
		}
	} else{ //Reads a label+value
		int elem_len = jsonparse_get_len(&state);
		elem_len++;
		char* label_buf = malloc(sizeof(char)*elem_len);
		//char label_buf[elem_len];
		jsonparse_copy_value(&state, label_buf, elem_len);
		printf("label_buf: %s\n", label_buf);
		
		jsonparse_next(&state);
		elem_len = jsonparse_get_len(&state);
		elem_len++;
		char* value_buf = malloc(sizeof(char)*elem_len);
		//char value_buf[elem_len];
		jsonparse_copy_value(&state, value_buf, elem_len);
		printf("value_buf: %s\n", value_buf);
		
		token->label = label_buf;
		token->value = value_buf;
		
		printf("---------\n");

		printf("label_buf: %s\n", label_buf);
		printf("value_buf: %s\n", value_buf);
		
		printf("label_buf: %d\n", label_buf);
		printf("value_buf: %d\n", value_buf);
	}
  }


void add_new_msg(char* msg){
	strcat(state.json, msg);
	state.len = state.len + length(msg) + 1;
}