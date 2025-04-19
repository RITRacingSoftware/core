#include timeout.h

__weak     void (*callback)(struct core_timeout_s *) {}
__weak     bool (*check)(struct core_timeout_s *) {}
__weak bool core_timeout_insert(core_timeout_t *timeout) {}
__weak void core_timeout_start_all() {}
__weak void core_timeout_reset_by_module_ref(void *module, uint32_t ref) {}
__weak void core_timeout_reset(core_timeout_t *timeout) {}
__weak void core_timeout_check_all() {}
__weak void core_timeout_suspend(core_timeout_t *timeout) {}
__weak void core_timeout_resume(core_timeout_t *timeout) {}
__weak void core_timeout_hard_reset(core_timeout_t *timeout) {}
