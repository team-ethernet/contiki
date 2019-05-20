#include "jsonparse.h"
#include <string.h>
#include "senml-decode.h"

/* REQUIRES JSON APP - MAKE SURE TO INCLUDE IT IN YOUR PROJECT ASWELL */

struct jsonparse_state state;
//struct pair lv;

int length(char* string){
	int i;
	for(i = 0; string[i] != '\0'; ++i);
    return i;
}

struct pair pair_test(){
	lv.label = "test";
	lv.value = 56;
	return lv;
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
	int parseNextNumber = jsonparse_next(&state);
	
	//Reads comma
	if(parseNextNumber == 44){
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
	} else if(parseNextNumber == 125){ //Reads }
		//If ]
		if(jsonparse_next(&state) == 93){
			struct pair lvnull = {.label = NULL, .value = NULL};
			printf("should be null: %s\n", lvnull.label);
			return lvnull;
		}else{ 	//If not ] then it has to be comma 
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
	} else{ //Reads a label+value
		int elem_len = jsonparse_get_len(&state);
		elem_len++;
		printf("elemn_len LEABEL: %d\n", elem_len);
		char* label_buf[elem_len];
		char* label[elem_len];
		jsonparse_copy_value(&state, label_buf, elem_len);
		jsonparse_next(&state);
		printf("READdef label_buf: %s\n", label_buf);
		elem_len = jsonparse_get_len(&state);
		elem_len++;
		char* value_buf[elem_len];
		char* value[elem_len];
		printf("elemn_len VALUE: %d\n", elem_len);
		jsonparse_copy_value(&state, value_buf, elem_len);
		printf("READdef value_buf: %s\n", value_buf);
		//struct pair lv = {.label = label_buf, .value = value_buf};
		
		
		//printf("---------\n");
		printf("label buf: %s\n", label_buf);
		printf("value buf: %s\n", value_buf);
		
		 printf("label buf: %d\n", label_buf);
		 printf("value buf: %d\n", value_buf);
		
		printf("text buf: %s\n", "bn");
		printf("text buf: %s\n", "urn:mac:testID");
		
		printf("text buf: %d\n", "bn");
		printf("text buf: %d\n", "urn:mac:testID");
		
	
		
		
		printf("label: %s\n", label);
		printf("value: %s\n", value);
		
		printf("label: %d\n", label);
		printf("value: %d\n", value);
		
		printf("---------\n");
		
		strcpy(label, label_buf);
		strcpy(value, value_buf);
		
		printf("----AFTER COPY:::-----\n");
		
		printf("label: %s\n", label);
		printf("value: %s\n", value);
		
		printf("label: %d\n", label);
		printf("value: %d\n", value);
		printf("---------\n");
		
		lv.label = label;
		lv.value = value;
		
		printf("default!! lv.label: %s\n", lv.label);
		printf("default!! lv.value: %s\n", lv.value);
	
		return lv;
	}
  }


void add_new_msg(char* msg){
	strcat(state.json, msg);
	state.len = state.len + length(msg) + 1;
}