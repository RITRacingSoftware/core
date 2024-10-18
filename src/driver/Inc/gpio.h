#pragma once

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

void core_GPIO_init(GPIO_TypeDef *port, uint16_t pin, uint16_t dir, uint32_t pull);
void core_GPIO_pin_set(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state);

void core_heartbeat_init(GPIO_TypeDef *port, uint16_t pin);
void core_GPIO_toggle_heartbeat();
void core_GPIO_set_heartbeat(GPIO_PinState state);