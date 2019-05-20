#include <stdio.h>
#include <string.h>
#include "jsonparse.h"
#include "senml-decode.h"

/* REQUIRES JSON APP - MAKE SURE TO INCLUDE IT IN YOUR PROJECT ASWELL */

struct jsonparse_state state;

char *label_buf;
char *value_buf;

unsigned *oldLabelAddr = 0;
unsigned *oldValueAddr = 0;

int length(char* string){
	int i;
	for(i = 0; string[i] != '\0'; ++i);
    return i;
	printf("23\n");
}

void init_json_decoder(char* msg){
	printf("21\n");
	state.json = msg;
	state.pos = 0;
	state.len = length(msg);
	
	printf("22\n");
	jsonparse_setup(&state, state.json, state.len);
	jsonparse_next(&state);
	jsonparse_next(&state);
	//printf("INIT state.json: %s\n", state.json);
}

//changes the input struct "token" to the next values
void read_next_token(struct pair *token){
	
	//printf("old old label addr: %p\n", oldLabelAddr);
	//printf("old old value addr: %p\n", oldValueAddr);
	
	/* --- FREE --- */
	if(oldLabelAddr != 0){
		free(oldLabelAddr);
	}
	if(oldValueAddr != 0){
		free(oldValueAddr);
	}
	
	printf("1");
	int parseNextNumber = jsonparse_next(&state);
	printf("2");
	//Reads comma
	if(parseNextNumber == 44){
		printf("3");
		jsonparse_next(&state);
		int elem_len = jsonparse_get_len(&state);
		elem_len++;
		printf("4");
		char* label_buf = malloc(sizeof(char)*elem_len);
		printf("5");
		//char label_buf[elem_len];
		jsonparse_copy_value(&state, label_buf, elem_len);
		printf("6");
		jsonparse_next(&state);
		elem_len = jsonparse_get_len(&state);
		elem_len++;
		printf("7");
		char* value_buf = malloc(sizeof(char)*elem_len);
		//char value_buf[elem_len];
		jsonparse_copy_value(&state, value_buf, elem_len);
		
		token->label = label_buf;
		token->value = value_buf;
		
		oldLabelAddr = label_buf;
		oldValueAddr = value_buf;
		
		printf("newly assigned old label addr: %p\n", oldLabelAddr);
		printf("newly assigned old value addr: %p\n", oldValueAddr);
		printf("10\n");
		printf("label_buf: %s\n", label_buf);
		printf("value_buf: %s\n", value_buf);
		printf("11\n");
	} else if(parseNextNumber == 125){ //Reads }
		//If ]
		printf("12\n");
		if(jsonparse_next(&state) == 93){
			token->label = NULL;
			token->value = NULL;
		}else{ 	//If not ] then it has to be comma 
		printf("13\n");
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
			printf("14\n");
			token->label = label_buf;
			token->value = value_buf;
			
			oldLabelAddr = label_buf;
			oldValueAddr = value_buf;
			
			printf("newly assigned old label addr: %p\n", oldLabelAddr);
			printf("newly assigned old value addr: %p\n", oldValueAddr);
			printf("15\n");
		}
	} else{ //Reads a label+value
	printf("16\n");
		int elem_len = jsonparse_get_len(&state);
		elem_len++;
		char* label_buf = malloc(sizeof(char)*elem_len);
		//char label_buf[elem_len];
		jsonparse_copy_value(&state, label_buf, elem_len);
		printf("label_buf: %s\n", label_buf);
		printf("17\n");
		jsonparse_next(&state);
		elem_len = jsonparse_get_len(&state);
		elem_len++;
		char* value_buf = malloc(sizeof(char)*elem_len);
		//char value_buf[elem_len];
		jsonparse_copy_value(&state, value_buf, elem_len);
		printf("value_buf: %s\n", value_buf);
		
		token->label = label_buf;
		token->value = value_buf;
		printf("18\n");
		oldLabelAddr = label_buf;
		oldValueAddr = value_buf;
		
		printf("newly assigned old label addr: %p\n", oldLabelAddr);
		printf("newly assigned old value addr: %p\n", oldValueAddr);
		
		printf("---------\n");
		printf("19\n");
		printf("label_buf: %s\n", label_buf);
		printf("value_buf: %s\n", value_buf);
	}
	printf("20\n");
  }


void add_new_msg(char* msg){
	printf("tjo\n");
	printf("state before %s\n", state.json);
	strcat(state.json, msg);
	printf("state after %s\n", state.json);
	state.len = state.len + length(msg) + 1;
	printf("30\n");
}