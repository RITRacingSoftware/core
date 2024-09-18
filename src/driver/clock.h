#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CORE_CLOCK_USART1 1
#define CORE_CLOCK_USART2 2
#define CORE_CLOCK_USART3 3
#define CORE_CLOCK_UART4 4
#define CORE_CLOCK_UART5 5

#define CORE_CLOCK_I2C1 1
#define CORE_CLOCK_I2C2 2
#define CORE_CLOCK_I2C3 3
#define CORE_CLOCK_I2C4 4

bool core_clock_ADC12_init();
bool core_clock_FDCAN_init();
bool core_clock_USART_init(uint8_t usart_num);
bool core_clock_I2C_init(uint8_t i2c_num);
bool core_clock_RTC_init();
uint8_t core_clock_generate_params(uint32_t src_freq, uint32_t target_freq, uint8_t *n, uint8_t *m, uint8_t *r);
bool core_clock_init(bool use_ext, uint32_t ext_freq, uint32_t sysclk_freq);
