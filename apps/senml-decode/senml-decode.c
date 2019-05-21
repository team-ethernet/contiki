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

unsigned *oldAddBufAddr = 0;

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
	/*
	printf("READ! beginning total json %s\n", state.json);
	printf("HEX OF state.json in ADD: %x\n",state.json);
	printf("READ! beginning total len %x\n", state.len);
	printf("READ! beginning total pos %x\n", state.pos);
	printf("ADDRESS OF state.json in START: %p\n",state.json);
	printf("ADDRESS OF oldLabelAddr in START: %p\n",oldLabelAddr);
	printf("ADDRESS OF oldValueAddr in START: %p\n",oldValueAddr);
	*/
	/* --- FREE --- */
	if(oldLabelAddr != 0){
		free(oldLabelAddr);
	}
	if(oldValueAddr != 0){
		free(oldValueAddr);
	}
	
	printf("total json: %s\n", state.json);
	
	//printf("outside state pos %d\n", state.pos);
	int parseNextNumber = jsonparse_next(&state);
	
	if(parseNextNumber == 44){ //Reads comma
		//printf("comma stateBEFORE pos: %d\n", state.pos);
		jsonparse_next(&state);
		int elem_len = jsonparse_get_len(&state);
		int totlen = elem_len;
		elem_len++;
		char* label_buf = malloc(sizeof(char)*elem_len);
		//char label_buf[elem_len];
		jsonparse_copy_value(&state, label_buf, elem_len);
		jsonparse_next(&state);
		elem_len = jsonparse_get_len(&state);
		totlen = totlen+elem_len;
		elem_len++;
		char* value_buf = malloc(sizeof(char)*elem_len);
		//char value_buf[elem_len];
		jsonparse_copy_value(&state, value_buf, elem_len);
		
		token->label = label_buf;
		token->value = value_buf;
		
		oldLabelAddr = label_buf;
		oldValueAddr = value_buf;
		
		/*printf("newly assigned old label addr: %p\n", oldLabelAddr);
		printf("newly assigned old value addr: %p\n", oldValueAddr);
		printf("label_buf: %s\n", label_buf);
		printf("value_buf: %s\n", value_buf);
		*/
		//printf("comma stateAFTER pos: %d\n", state.pos);
		
		
		//printf("comma! before json %s\n", state.json);
		state.json = state.json+state.pos;
		state.len = state.len-state.pos;
		//printf("comma! after json %s\n", state.json);
		state.pos = 0;
		
		
	} else if(parseNextNumber == 125){ //Reads }
	
		//printf("hak stateBEFORE  pos: %d\n", state.pos);
		//If ]
		if(jsonparse_next(&state) == 93){
			token->label = NULL;
			token->value = NULL;
		}else{ 	//If not ] then it has to be comma 
			jsonparse_next(&state);
			jsonparse_next(&state);
			int elem_len = jsonparse_get_len(&state);
			int totlen = elem_len;
			elem_len++;
			char* label_buf = malloc(sizeof(char)*elem_len);
			//char label_buf[elem_len];
			jsonparse_copy_value(&state, label_buf, elem_len);
			
			jsonparse_next(&state);
			elem_len = jsonparse_get_len(&state);
			totlen = totlen+elem_len;
			elem_len++;
			char* value_buf = malloc(sizeof(char)*elem_len);
			//char value_buf[elem_len];
			jsonparse_copy_value(&state, value_buf, elem_len);
			token->label = label_buf;
			token->value = value_buf;
			
			oldLabelAddr = label_buf;
			oldValueAddr = value_buf;
			
			//printf("newly assigned old label addr: %p\n", oldLabelAddr);
			//printf("newly assigned old value addr: %p\n", oldValueAddr);
			
			//printf("hak! before json %s\n", state.json);
			state.json = state.json+state.pos;
			state.len = state.len-state.pos;
			//printf("hak! after json %s\n", state.json);
			state.pos = 0;
			
			//printf(" hak stateAFTER pos: %d\n", state.pos);
			
		}
	} else{ //Reads a label+value
		
		//printf("default stateBEFORE  pos: %d\n", state.pos);
	
		int elem_len = jsonparse_get_len(&state);
		//int totlen = elem_len;
		elem_len++;
		char* label_buf = malloc(sizeof(char)*elem_len);
		//char label_buf[elem_len];
		jsonparse_copy_value(&state, label_buf, elem_len);
		jsonparse_next(&state);
		elem_len = jsonparse_get_len(&state);
		//totlen = totlen+elem_len+6;
		elem_len++;
		char* value_buf = malloc(sizeof(char)*elem_len);
		//char value_buf[elem_len];
		jsonparse_copy_value(&state, value_buf, elem_len);
		
		token->label = label_buf;
		token->value = value_buf;
		oldLabelAddr = label_buf;
		oldValueAddr = value_buf;
		
		// printf("label_buf: %s\n", label_buf);
		// printf("value_buf: %s\n", value_buf);
		
		// printf("default stateAFTER pos: %d\n", state.pos);
		
		//printf("default! before json %s\n", state.json);
		state.json = state.json+state.pos;
		state.len = state.len-state.pos;
		//printf("default! after json %s\n", state.json);
		state.pos = 0;
	}
  }


void add_new_msg(char* msg){
	
	if(oldAddBufAddr != 0){
		//printf("FREE OLD ADD BUF ADDR");
		free(oldAddBufAddr);
	}
	
	/*
	printf("ADD! before total json %s\n", state.json);
	printf("ADD! before total len %d\n", state.len);
	printf("ADD! before total pos %d\n", state.pos);
	*/
	int stlen = state.len;
	int msglen = length(msg);
	int totlen = stlen+msglen;
	char *buf = malloc(sizeof(char)*(totlen+1));
	
	strcpy(buf, state.json);
	
	strcat(buf, msg);
	
	//free(state.json);
	
	state.json = buf;
	state.len = totlen;
	/*
	printf("ADD! after total json %s\n", state.json);
	printf("ADD! after total len %d\n", state.len);
	printf("ADD! after total pos %d\n", state.pos);
	printf("ADD! after total lenx %x\n", state.len);
	printf("ADD! after total posx %x\n", state.pos);
	printf("ADDRESS OF state.json in ADD: %p\n",state.json);
	printf("HEX OF state.json in ADD: %x\n",state.json);
	*/
	oldAddBufAddr = buf;
}