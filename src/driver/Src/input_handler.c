#include "core_config.h"
#include "input_handler.h"
#include "timeout.h"

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stm32g4xx_hal.h>

//Helper
bool core_input_handler_check_demotion(core_input_handler_t *input_handler){
    return input_handler->can_demote || !(input_handle->demotion_input_list);
}

void core_input_handler_setter(core_input_handler_t *input_handler){
    if(input_handler->input <= input_handler->max_value && input_handler->input >= input_handler->min_value){
        input_handler->last_valid_input = input_handler->input;
        input_handler->state = CORE_INPUT_HANDLER_STATE_ENABLED;
    }
}

void core_input_handler_getter(core_input_handler_t *input_handler){
    if(input_handler->state != CORE_INPUT_HANDLER_STATE_ENABLED && core_input_handler_check_demotion(input_handler) != CORE_INPUT_HANDLER_NO_DEMOTE){
        
    } 
    else{
        input_handler->callback();
    }

}



