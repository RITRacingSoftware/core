#include "gpio.h"
#include "core_config.h"

#include "clock.h"

#include <stm32g4xx_hal.h>

GPIO_TypeDef *LED_port;
uint32_t LED_pin;

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

void core_GPIO_pin_set(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    HAL_GPIO_WritePin(port, pin, state);
}

void core_heartbeat_init(GPIO_TypeDef *port, uint16_t pin)
{
    LED_port = port;
    LED_pin = pin;

    core_GPIO_init(LED_port, LED_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
}

void core_GPIO_toggle_heartbeat()
{
	HAL_GPIO_TogglePin(LED_port, LED_pin);
}

void core_GPIO_set_heartbeat(GPIO_PinState state)
{
	HAL_GPIO_WritePin(LED_port, LED_pin, state);
}
