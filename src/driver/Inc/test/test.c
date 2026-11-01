#include "../input_handler.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

static core_input_handler_t test_input;
uint16_t min = 100;
uint16_t max = 4000;

void init_input_handler(){
	test_input.input_list = 2000;
	test_input.can_demote = false;
	test_input.last_valid_input = 0;
}


int main(){
	init_input_handler();
	bool onList = core_input_handler_insert(&test_input);
	printf("On List?: %d\n", onList);
	bool results = core_check_input_handler_irrational(max, min);
	printf("Result: %d\n", results);
	return 0;
}

