#pragma once

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

void GPIO_init(GPIO_TypeDef *port, uint32_t pin, uint32_t dir, uint32_t pull);

void heartbeat_init(GPIO_TypeDef *port, uint32_t pin);
void GPIO_toggle_heartbeat(void);
void GPIO_set_heartbeat(bool state);