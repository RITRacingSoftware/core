#include "core_config.h"
#include "input_handler.h"
#include "timeout.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stm32g4xx_hal.h>

static core_input_handler_t *core_input_handler_list[CORE_INPUT_HANDLER_NUM];
static int n_core_input_handled = 0;


bool core_input_handler_insert(core_input_handler_t *input_handler){
    if(n_core_input_handled < CORE_INPUT_HANDLER_NUM){
        core_input_handler_list[n_core_input_handled] = input_handler;
        n_core_input_handled++;
        return true;
    }
    return false;
}

bool core_check_input_handler_irrational(){
    for (int i = 0; i < n_core_input_handled; i++){
        if(core_input_handler_list[i]->input <= core_input_handler_list[i]->max_value && core_input_handler_list[i]->input >= core_input_handler_list[i]->min_value){
            core_input_handler_list[i]->last_valid_input = core_input_handler_list[i]->input; 
            core_input_handler_list[i]->state = CORE_INPUT_HANDLER_STATE_ENABLED;
            return true;
        }
        core_input_handler_list[i]->state = CORE_INPUT_HANDLER_STATE_IRR;
        core_input_handler_reset(core_input_handler_list[i]);
        return false;
    }
}


void core_input_handler_reset(core_input_handler_t *input_handler){
    for (int i = 0; i < n_core_input_handled; i++){
        if(input_handler->state == CORE_INPUT_HANDLER_STATE_IRR){
           switch(input_handler->demote_state){
                case CORE_INPUT_HANDLER_NO_DEMOTION:
                    core_input_handler_callback(timeout);
                    // Use last valid input?
                case CORE_INPUT_HANDLER_DEMOTE_1:
                    input_handler->input = input_handler->new_input_source[0];
                    input_handler->state = CORE_INPUT_HANDLER_STATE_ENABLED;
                case CORE_INPUT_HANDLER_DEMOTE_2:
                    if (input_handler->new_input_source[1] == NULL) input_handler->demote_state = CORE_INPUT_HANDLER_NO_DEMOTION;
                    input_handler->input = input_handler->new_input_source[1];
                    input_handler->state = CORE_INPUT_HANDLER_STATE_ENABLED;
                case CORE_INPUT_HANDLER_DEMOTE_3:
                    if (input_handler->new_input_source[2] == NULL) input_handler->demote_state = CORE_INPUT_HANDLER_NO_DEMOTION;
                    input_handler->input = input_handler->new_input_source[2];
                    input_handler->state = CORE_INPUT_HANDLER_STATE_ENABLED;
            } 
        }  
    } 
}


void core_input_handler_callback(core_input_handler_t *input_handler, uint64_t fault_code){
    input_handler->fault = fault_code;
}


