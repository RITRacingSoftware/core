#include "timeout.h"
#include "core_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stm32g4xx_hal.h>

static core_timeout_t *core_timeout_list[CORE_TIMEOUT_NUM];
static int n_core_timeouts = 0;

bool core_timeout_insert(core_timeout_t *timeout) {
    if (n_core_timeouts < CORE_TIMEOUT_NUM) {
        core_timeout_list[n_core_timeouts] = timeout;
        n_core_timeouts++;
        return true;
    }
    return false;
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
    for (int i=0; i < n_core_timeouts; i++) {
        to = core_timeout_list[i];
        if ((to->module == module) && (to->ref == ref)) {
            core_timeout_hard_reset(to);
        }
    }
}

void core_timeout_reset(core_timeout_t *timeout) {
    if (!((timeout->state & CORE_TIMEOUT_STATE_TIMED_OUT) && timeout->latching)){
        core_timeout_hard_reset(timeout);
    }
}

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

void core_timeout_suspend(core_timeout_t *timeout) {
    timeout->state |= CORE_TIMEOUT_STATE_SUSPENDED;
}

void core_timeout_resume(core_timeout_t *timeout) {
    timeout->state &= ~CORE_TIMEOUT_STATE_SUSPENDED;
    core_timeout_reset(timeout);
}

void core_timeout_hard_reset(core_timeout_t *timeout) {
    uint32_t t = HAL_GetTick();
    timeout->last_event = t;
    timeout->state &= ~CORE_TIMEOUT_STATE_TIMED_OUT;
}
