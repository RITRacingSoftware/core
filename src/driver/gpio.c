#include "gpio.h"
#include "clock.h"

#include "stdbool.h"

#include <stm32g4xx_hal.h>

uint32_t LED_port;
uint32_t LED_pin;

void GPIO_init(uint32_t port, uint32_t pin, uint32_t dir,
               uint32_t pull)
{

    clock_port_init(port);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = dir;
    GPIO_InitStruct.Pull = pull;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}


void heartbeat_init(uint32_t port, uint32_t pin)
{
    LED_port = port;
    LED_pin = pin;

    GPIO_init(LED_port, LED_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
}

void GPIO_toggle_heartbeat(void)
{
	HAL_GPIO_TogglePin(LED_port, LED_pin);
}

void GPIO_set_heartbeat(bool state)
{
	HAL_GPIO_WritePin(LED_port, LED_pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}