/**
  * @file   timeout.c
  * @brief  Core timeout library
  *
  * This core library component can be used to set timeouts for both software
  * events and core library events. Timeouts are specified by a core_timeout_t
  * struct.
  *
  * ## Typical initialization
  *
  *  1. Initialize a core_timeout_t struct for each timeout
  *       - Set the `timeout` member to the desired timeout length in 
  *         milliseconds
  *       - Set the `callback` member to the timeout's callback function. When
  *         the function is called, a pointer to the struct defining the 
  *         timeout that called the callback function is passed to it.
  *       - Set the `module` and `ref` members. See below
  *  2. Add the timeout structs to the internal list with core_timeout_insert()
  *  3. Start all of the timeouts with core_timeout_start_all()
  *  4. Call `core_timeout_check_all` at regular intervals. If a timeout has 
  *     elapsed, the handler is called. Timeouts are checked in the order they 
  *     are added.
  *  5. Reset timeouts as needed with the core_timeout_reset() function.
  *
  * ## module and ref
  * The `module` and `ref` members of the core_timeout_t struct can be used to
  * define timeouts that are automatically reset by other core library
  * components. To reset these timeouts, the core_timeout_reset_by_module_ref()
  * function is used. The `module` member is set to a register struct defined
  * by the HAL (e.g. FDCAN1, SPI1, USART3, etc.). The meaning of `ref` depends
  * on the selected module. Currently, the modules that support this feature
  * are:
  *  1. `FDCAN1`, `FDCAN2`, and `FDCAN3`: The timeout is reset when a frame
  *     with an ID equal to the value of `ref` is received
  *
  * Timeouts with a `module` not in the above list will never be reset by the
  * core library. Thus, the user can define timeouts where `module` is NULL
  * or equal to a custom value and reset them using the
  * core_timeout_reset_by_module_ref() function.
  * 
  */

#include "timeout.h"
#include "core_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stm32g4xx_hal.h>

static core_timeout_t **core_timeout_list = NULL;
static int core_timeout_list_size = 0;
static int n_core_timeouts = 0;

/**
  * @brief  Add a timeout to the internal list of timeouts to be monitored
  * @param  timeout Pointer to the timeout
  */
void core_timeout_insert(core_timeout_t *timeout) {
    if (n_core_timeouts == core_timeout_list_size) {
        core_timeout_list_size += CORE_TIMEOUT_BLOCK_SIZE;
        core_timeout_list = realloc(core_timeout_list, CORE_TIMEOUT_BLOCK_SIZE*sizeof(core_timeout_t*));
    }
    core_timeout_list[n_core_timeouts] = timeout;
    n_core_timeouts++;
}

/**
  * @brief  Enable all timeouts currently in the internal timeout list
  */
void core_timeout_start_all() {
    uint32_t t = HAL_GetTick();
    for (int i=0; i < n_core_timeouts; i++) {
        core_timeout_list[i]->last_event = t;
        core_timeout_list[i]->state |= CORE_TIMEOUT_STATE_ENABLED;
    }
}

/**
  * @brief  Reset a timeout given the module and the refernce number
  * @param  module Module that is calling this function
  * @param  ref Reference number
  */
void core_timeout_reset_by_module_ref(void *module, uint32_t ref) {
    core_timeout_t *to;
    uint32_t t = HAL_GetTick();
    for (int i=0; i < n_core_timeouts; i++) {
        to = core_timeout_list[i];
        if ((to->module == module) && (to->ref == ref)) {
            to->last_event = t;
            to->state &= ~CORE_TIMEOUT_STATE_TIMED_OUT;
        }
    }
}

/**
  * @brief  Reset a timeout given a pointer to the timeout struct
  * @param  timeout Pointer to the timeout being reset
  */
void core_timeout_reset(core_timeout_t *timeout) {
    timeout->last_event = t;
    timeout->state &= ~CORE_TIMEOUT_STATE_TIMED_OUT;
}

/**
  * @brief  Check all timeouts currently stored in the internal timeout list.
  */
void core_timeout_check_all() {
    uint32_t t = HAL_GetTick();
    core_timeout_t *to;
    uint32_t diff;
    for (int i=0; i < n_core_timeouts; i++) {
        to = core_timeout_list[i];
        if (to->state & CORE_TIMEOUT_STATE_ENABLED) {
            diff = t - to->last_event;
            if (to->state & CORE_TIMEOUT_STATE_TIMED_OUT) {
                to->callback(to);
            } else if (diff >= to->timeout) {
                to->callback(to);
                to->state |= CORE_TIMEOUT_STATE_TIMED_OUT;
            }
        }
    }
}

