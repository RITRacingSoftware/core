#ifndef CORE_INPUT_HANDLER_H
#define CORE_INPUT_HANDLER_H

#include <stdbool.h>
#include <stdint.h>


#define CORE_INPUT_HANDLER_NO_DEMOTE 0
#define CORE_INPUT_HANDLER_STATE_ENABLED 0x01
#define CORE_INPUT_HANDLER_STATE_IRR 0x02

typedef struct core_input_handler_s {
    void (*callback)(struct core_input_handler_s *);
    bool (*check)(struct core_input_handler_s *);
    uint16_t input;
    uint16_t last_valid_input;
    uint16_t max_value;
    uint16_t min_value;
    bool can_demote;
    uint64_t fault_code;
    uint8_t state;
} core_input_handler_t;



void core_input_handler_setter(core_input_handler_t *input_handler, uint16_t input);


void core_input_handler_getter(core_input_handler_t *input_handler, uint16_t new_input);

/**
* @brief Add inputs to list of inputs to check for irrationalty and demotion 
* @param input handler pointer to inputs
* @retval 1 if there is enough space
* @retval 0 otherwise
*/
bool core_input_handler_insert(core_input_handler_t *input_handler);

/**
* @brief Compares input to max and min possible values
* @retval 1 if input value is lower then max and lower then min
* @retval 0 otherwise
*/
bool core_check_input_handler_irrational();

/**
* @breif Resets or demotes when input is in an irrational state
*/ 
void core_input_handler_reset(core_input_handler_t *input_handler);

/**
*@brief Sets fault is 

void core_input_handler_callback(core_input_handler_t *input_handler, uint64_t fault_code);*/
#endif
