#include "core_config.h"
#include "../Inc/input_handler.h"
#include "timeout.h"
#include "FaultManager.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stm32g4xx_hal.h>

static core_input_handler_t *core_input_handler_list[CORE_INPUT_HANDLER_NUM];
static int n_core_input_handled = 0;
static core_input_handler_t test_input;
uint16_t min = 100;
uint16_t max = 4000;



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
            core_input_handler_list[i]->last_valid_input = core_input_handler_list[i-1]->input; 
            core_input_handler_list[i]->state = CORE_INPUT_HANDLER_STATE_ENABLED;
            return true;
        }
        core_input_handler_list[i]->state = CORE_INPUT_HANDLER_STATE_IRR;
        core_input_handler_reset(core_input_handler_list[i]);
        return false;
    }
}


void core_input_handler_reset(core_input_handler_t *input_handler, core_timeout_t *timeout){
    for (int i = 0; i < n_core_input_handled; i++){
        if(input_handler->state == CORE_INPUT_HANDLER_STATE_DEMOTED && input_handler->demote_state == ){
            
        } 
        else if(input_handler->state == CORE_INPUT_HANDLER_STATE_IRR && input_handler->demote_state == CORE_INPUT_HANDLER_NO_DEMOTE){
            input_handler->state = CORE_INPUT_HANDLER_STATE_DEMOTED;
            core_timeout_reset(timeout);
        }        
    } 
}

void core_input_handler_callback(core_input_handler_t *input_handler){
    FaultManager_set(input_handler->fault_code);
}


