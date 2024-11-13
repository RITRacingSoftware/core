/**
  * @file   gpio.c
  * @brief  Core GPIO library
  *
  * This core library component is used to initialize and control GPIO pins.
  */
#include "gpio.h"
#include "core_config.h"

#include "clock.h"

#include <stm32g4xx_hal.h>

static GPIO_TypeDef *LED_port;
static uint32_t LED_pin;

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
void core_GPIO_init(GPIO_TypeDef *port, uint16_t pin, uint16_t dir, uint32_t pull)
{

    core_clock_port_init(port);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = dir;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

/**
  * @brief  Set a digital output
  * @param  port GPIO port (GPIOx)
  * @param  pin GPIO pin (GPIO_PIN_x)
  * @param  state The desired setting of the output
  */
void core_GPIO_digital_write(GPIO_TypeDef *port, uint16_t pin, bool state)
{
    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief  Read a digital input
  * @param  port GPIO port (GPIOx)
  * @param  pin GPIO pin (GPIO_PIN_x)
  * @return The state of the pin
  */
bool core_GPIO_digital_read(GPIO_TypeDef *port, uint16_t pin)
{
    return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET ? true : false;
}

/**
  * @brief  Set a particular pin as the heartbeat LED output
  * @param  port GPIO port (GPIOx)
  * @param  pin GPIO pin (GPIO_PIN_x)
  */
void core_heartbeat_init(GPIO_TypeDef *port, uint16_t pin)
{
    LED_port = port;
    LED_pin = pin;

    core_GPIO_init(LED_port, LED_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
}

/**
  * @brief  Toggle the heartbeat LED output
  */
void core_GPIO_toggle_heartbeat()
{
    HAL_GPIO_TogglePin(LED_port, LED_pin);
}

/**
  * @brief  Set the state of the heartbeat LED
  * @param  state The desired setting of the output
  */
void core_GPIO_set_heartbeat(bool state)
{
    core_GPIO_digital_write(LED_port, LED_pin, state);
}
