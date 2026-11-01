#ifndef CORE_INPUT_HANDLER_H
#define CORE_INPUT_HANDLER_H

#include <stdbool.h>
#include <stdint.h>

#define CORE_INPUT_HANDLER_STATE_ENABLED 0x01
#define CORE_INPUT_HANDLER_STATE_IRR 0x02
#define CORE_INPUT_HANDLER_STATE_DEMOTED 0x04

typedef struct core_input_handler_s {
    void (*callback)(struct core_input_handler_s *);
    bool (*check)(struct core_input_handler_s *);
    uint16_t input;
    uint16_t last_valid_input;
    uint8_t state;
    uint16_t max_value;
    uint16_t min_value;
    uint8_t demote_state;
    uint64_t fault_code;
} core_input_handler_t;

/**
* @brief Add inputs to list of inputs to check for irrationalty and demotion 
* @param input handler pointer to inputs
* @retval 1 if there is enough space
* @retval 0 otherwise
*/
bool core_input_handler_insert(core_input_handler_t *input_handler);

/**
* @brief 
* @param max value to compare to re
*/
bool core_check_input_handler_irrational();


/**
*
*
*/ 
void core_input_handler_reset(core_input_handler_t *input_handler);

/**
*
* 
*/

void core_input_handler_callback(core_input_handler_t *input_handler);
#endif
