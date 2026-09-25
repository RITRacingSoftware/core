#pragma once

#include <stdbool.h>
#include "stm32g4xx_hal.h"

bool core_PWM_input_init(TIM_TypeDef *tim, uint8_t chan, GPIO_TypeDef *port, uint16_t pin, uint8_t af);
uint32_t core_PWM_read_pulsewidth(TIM_TypeDef *tim, uint8_t chan);
uint32_t core_PWM_read_period(TIM_TypeDef *tim, uint8_t chan);
