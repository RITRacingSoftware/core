/**
  * @file   gpio.h
  * @brief  Core GPIO library
  *
  * This core library component is used to initialize and control GPIO pins.
  */

#pragma once

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/**
  * @brief  Initialize a GPIO pin
  * @param  port Port on which the pin is located (GPIOx)
  * @param  pin Pin number (GPIO_PIN_x)
  * @param  dir Direction or pin mode (GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP,
  *         GPIO_MODE_OUTPUT_OD, GPIO_MODE_AF_PP, GPIO_MODE_AF_OD, or 
  *         GPIO_MODE_ANALOG)
  * @param  pull Pullup/pulldown resistor configuration (GPIO_NOPULL, 
  *         GPIO_PULLUP, GPIO_PULLDOWN)
  */
void core_GPIO_init(GPIO_TypeDef *port, uint16_t pin, uint16_t dir, uint32_t pull);

/**
  * @brief  Set a digital output
  * @param  port GPIO port (GPIOx)
  * @param  pin GPIO pin (GPIO_PIN_x)
  * @param  state The desired setting of the output
  */
void core_GPIO_digital_write(GPIO_TypeDef *port, uint16_t pin, bool state);

/**
  * @brief  Read a digital input
  * @param  port GPIO port (GPIOx)
  * @param  pin GPIO pin (GPIO_PIN_x)
  * @return The state of the pin
  */
bool core_GPIO_digital_read(GPIO_TypeDef *port, uint16_t pin);

/**
  * @brief  Set a particular pin as the heartbeat LED output
  * @param  port GPIO port (GPIOx)
  * @param  pin GPIO pin (GPIO_PIN_x)
  */
void core_heartbeat_init(GPIO_TypeDef *port, uint16_t pin);

/**
  * @brief  Toggle the heartbeat LED output
  */
void core_GPIO_toggle_heartbeat();

/**
  * @brief  Set the state of the heartbeat LED
  * @param  state The desired setting of the output
  */
void core_GPIO_set_heartbeat(bool state);
