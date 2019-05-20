#include "jsonparse.h"
#include <string.h>
#include "senml-decode.h"
#include <string.h>

/* REQUIRES JSON APP - MAKE SURE TO INCLUDE IT IN YOUR PROJECT ASWELL */

struct jsonparse_state state;
struct pair lv;

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
	printf("INIT state.json: %s\n", state.json);
}

struct pair read_next_token(){
	printf("READ state.pos: %d\n", state.pos);
		switch(jsonparse_next(&state)){
	//Reads comma
	case ((int)44):
	{
		jsonparse_next(&state);
		int elem_len = jsonparse_get_len(&state);
		char* label_buf[elem_len];
		jsonparse_copy_value(&state, label_buf, elem_len);
		printf("READ44 label_buf: %s\n", label_buf);
		jsonparse_next(&state);
		elem_len = jsonparse_get_len(&state);
		char* value_buf[elem_len];
		jsonparse_copy_value(&state, value_buf, elem_len);
		printf("READ44 value_buf: %s\n", value_buf);
		//struct pair lv = {.label = label_buf, .value = value_buf};
		lv.label = label_buf;
		lv.value = value_buf;
	printf("44!! lv.labal: %s\n", lv.label);
		return lv;
		
	}
	//Reads }
	case ((int)125):
	{
		//If ]
		if(jsonparse_next(&state) == 93){
			struct pair lvnull = {.label = NULL, .value = NULL};
			
	printf("should be null: %s\n", lvnull.label);
			return lvnull;
		}
		//If not ] then it has to be comma 
		else{
			jsonparse_next(&state);
			jsonparse_next(&state);
			int elem_len = jsonparse_get_len(&state);
			char* label_buf[elem_len];
			jsonparse_copy_value(&state, label_buf, elem_len);
		printf("READ125 label_buf: %s\n", label_buf);
			jsonparse_next(&state);
			elem_len = jsonparse_get_len(&state);
			char* value_buf[elem_len];
			jsonparse_copy_value(&state, value_buf, elem_len);
		printf("READ125 value_buf: %s\n", value_buf);
			//struct pair lv = {.label = label_buf, .value = value_buf};
		lv.label = label_buf;
		lv.value = value_buf;
		
	printf("125 else!! lv.labal: %s\n", lv.label);
		return lv;
		}
	}
	//Reads a label+value
	default:
	{
		int elem_len = jsonparse_get_len(&state);
		elem_len++;
		printf("elemn_len LEABEL: %d\n", elem_len);
		char* label_buf[elem_len];
		jsonparse_copy_value(&state, label_buf, elem_len);
		printf("READdef label_buf: %s\n", label_buf);
		printf("READdef jsonparse_next: %d\n", jsonparse_next(&state));
		elem_len = jsonparse_get_len(&state);
		char* value_buf[elem_len];
		elem_len++;
		printf("elemn_len VALUE: %d\n", elem_len);
		jsonparse_copy_value(&state, value_buf, elem_len);
		printf("READdef value_buf: %s\n", value_buf);
		//struct pair lv = {.label = label_buf, .value = value_buf};
		lv.label = label_buf;
		lv.value = value_buf;
		
	printf("default!! lv.label: %s\n", lv.label);
	printf("default!! lv.value: %s\n", lv.value);
	printf("default!! lv.value: %d\n", lv);
	printf("default!! lv.value: %d\n", &lv.value);
		return lv;
	}
  }
}

void add_new_msg(char* msg){
	strcat(state.json, msg);
	state.len = state.len + length(msg) + 1;
}