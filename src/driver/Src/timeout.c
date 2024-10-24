#include "timeout.h"
#include "core_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stm32g4xx_hal.h>

static core_timeout_t **core_timeout_list = NULL;
static int core_timeout_list_size = 0;
static int n_core_timeouts = 0;

void core_timeout_insert(core_timeout_t *timeout) {
    if (n_core_timeouts == core_timeout_list_size) {
        core_timeout_list_size += CORE_TIMEOUT_BLOCK_SIZE;
        core_timeout_list = realloc(core_timeout_list, CORE_TIMEOUT_BLOCK_SIZE*sizeof(core_timeout_t*));
    }
    core_timeout_list[n_core_timeouts] = timeout;
    n_core_timeouts++;
}

void core_timeout_start_all() {
    uint32_t t = HAL_GetTick();
    for (int i=0; i < n_core_timeouts; i++) {
        core_timeout_list[i]->last_event = t;
        core_timeout_list[i]->state |= CORE_TIMEOUT_STATE_ENABLED;
    }
}

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

