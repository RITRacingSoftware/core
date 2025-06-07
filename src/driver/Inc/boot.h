#include <stdint.h>

#ifndef CORE_BOOT_H
#define CORE_BOOT_H

void core_boot_init();
void core_boot_reset_and_enter();

#if defined(CORE_BOOT_EXTERNAL) && (CORE_BOOT_EXTERNAL == 1)
extern void core_boot_external_read(uint8_t *ptr, uint32_t address, uint32_t length);
extern void core_boot_external_write(uint8_t *ptr, uint32_t address, uint32_t length);
extern void core_boot_external_enter();
extern void core_boot_external_exit();
#endif

#endif
