#pragma once

#include <stdbool.h>
#include <stdint.h>

bool core_clock_ADC12_init();
bool core_clock_FDCAN_init();
uint8_t core_clock_generate_params(uint32_t src_freq, uint32_t target_freq, uint8_t *n, uint8_t *m, uint8_t *r);
bool core_clock_init(bool use_ext, uint32_t ext_freq, uint32_t sysclk_freq);
