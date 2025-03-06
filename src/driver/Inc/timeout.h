#ifndef CORE_TIMEOUT_H
#define CORE_TIMEOUT_H

#include <stdint.h>
#include <stdbool.h>

#define CORE_TIMEOUT_STATE_ENABLED 0x01
#define CORE_TIMEOUT_STATE_TIMED_OUT 0x02
#define CORE_TIMEOUT_STATE_SUSPENDED 0x04

typedef struct core_timeout_s {
    void *module;
    uint32_t ref;
    uint32_t last_event;
    uint32_t timeout;
    uint8_t state;
    void (*callback)(struct core_timeout_s *);
    bool (*check)(struct core_timeout_s *);
} core_timeout_t;

void core_timeout_insert(core_timeout_t *timeout);
void core_timeout_start_all();
void core_timeout_reset_by_module_ref(void *module, uint32_t ref);
void core_timeout_reset(core_timeout_t *timeout);
void core_timeout_check_all();
void core_timeout_suspend(core_timeout_t *timeout);
void core_timeout_resume(core_timeout_t *timeout);

#endif
