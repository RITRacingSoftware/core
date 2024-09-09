#pragma once

#include<stdint.h>
#include "stdbool.h"

void GPIO_init(uint32_t port, uint32_t pin, uint32_t dir,
               uint32_t pull);

void heartbeat_init(uint32_t port, uint32_t pin);
void GPIO_toggle_heartbeat(void);
void GPIO_set_heartbeat(bool state);