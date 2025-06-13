#pragma once

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

void core_GPIO_init(GPIO_TypeDef *port, uint16_t pin, uint16_t dir, uint32_t pull);
void core_GPIO_digital_write(GPIO_TypeDef *port, uint16_t pin, bool state);
bool core_GPIO_digital_read(GPIO_TypeDef *port, uint16_t pin);

void core_heartbeat_init(GPIO_TypeDef *port, uint16_t pin);
void core_GPIO_toggle_heartbeat();
void core_GPIO_set_heartbeat(bool state);
