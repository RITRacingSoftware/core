#ifndef CORE_TIMESTAMP_H
#define CORE_TIMESTAMP_H
#include <stdint.h>

void core_timestamp_init();
void core_timestamp_read_separate(uint16_t *lsb, uint32_t *msb);
uint64_t core_timestamp_get_tick();

#endif
