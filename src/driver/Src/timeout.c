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
  *       - Set the `module` and `ref` members. See below.
  *       - Set the 'check' member. See below.
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
  * ## check
  * The 'check' member can be implemented to have the timeout library
  * automatically check whether or not a timeout should be reset.
  * The 'check' member should point to a function returning a bool value,
  * *true* if the timeout should be reset, *false* if not. This is checked
  * inside the *core_timeout_check_all* function.
  *
  * ## latching
  * Set this member to *1* to not allow it to be reset by a normal
  * reset function. A timeout with this parameter set can only be
  * reset through the *core_timeout_hard_reset* function.
  *
  * ## single_shot
  * Set this member to *1* to only have callback trigger once, the first time
  * the timeout has hit its limit.
  */

#include "timeout.h"
#include "core_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stm32g4xx_hal.h>

static core_timeout_t *core_timeout_list[CORE_TIMEOUT_NUM];
static int n_core_timeouts = 0;

/**
  * @brief  Add a timeout to the internal list of timeouts to be monitored
  * @param  timeout Pointer to the timeout
  * @retval 1 if there is enough space
  * @retval 0 otherwise
  */
bool core_timeout_insert(core_timeout_t *timeout) {
    if (n_core_timeouts < CORE_TIMEOUT_NUM) {
        core_timeout_list[n_core_timeouts] = timeout;
        n_core_timeouts++;
        return true;
    }
    return false;
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
    for (int i=0; i < n_core_timeouts; i++) {
        to = core_timeout_list[i];
        if ((to->module == module) && (to->ref == ref)) {
            core_timeout_hard_reset(to);
        }
    }
}

/**
  * @brief  Reset a timeout given a pointer to the timeout struct
  * @param  timeout Pointer to the timeout being reset
  */
void core_timeout_reset(core_timeout_t *timeout) {
    if (!((timeout->state & CORE_TIMEOUT_STATE_TIMED_OUT) && timeout->latching)){
        core_timeout_hard_reset(timeout);
    }
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
        if ((to->state & CORE_TIMEOUT_STATE_ENABLED) && !(to->state & CORE_TIMEOUT_STATE_SUSPENDED)) {
            // If a checking function has been implemented, use it
            if ((to->check != NULL) && (to->check(to))) to->last_event = t;
            diff = t - to->last_event;

            if (to->state & CORE_TIMEOUT_STATE_TIMED_OUT) {
                if (!to->single_shot) to->callback(to);
            } else if (diff >= to->timeout) {
                to->callback(to);
                to->state |= CORE_TIMEOUT_STATE_TIMED_OUT;
            }
        }
    }
}

/**
 * @brief Suspend timeout
 * @param timeout Pointer to the timeout being suspended
 */
void core_timeout_suspend(core_timeout_t *timeout) {
    timeout->state |= CORE_TIMEOUT_STATE_SUSPENDED;
}

/**
 * @brief Resume suspended timeout
 * @param timeout Pointer to the timeout being resumed
 */
void core_timeout_resume(core_timeout_t *timeout) {
    timeout->state &= ~CORE_TIMEOUT_STATE_SUSPENDED;
    core_timeout_reset(timeout);
}

/**
 * @brief Force reset a timeout
 * @param timeout Pointer to timeout being reset
 */
void core_timeout_hard_reset(core_timeout_t *timeout) {
    uint32_t t = HAL_GetTick();
    timeout->last_event = t;
    timeout->state &= ~CORE_TIMEOUT_STATE_TIMED_OUT;
}
