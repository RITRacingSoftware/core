#pragma once

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

void core_clock_ADC12_init();
void core_clock_ADC345_init();
void core_clock_FDCAN_init(FDCAN_GlobalTypeDef *can);
bool core_clock_USART_init(USART_TypeDef *usart);
bool core_clock_timer_init(TIM_TypeDef *timer);
bool core_clock_I2C_init(I2C_TypeDef *i2c);
bool core_clock_RTC_init();
void core_clock_port_init(GPIO_TypeDef *port);
uint8_t core_clock_generate_params(uint32_t src_freq, uint32_t target_freq, uint8_t *n, uint8_t *m, uint8_t *r);
bool core_clock_init();
