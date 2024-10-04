#ifndef USART_H
#define USART_H
#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_usart.h>

#define CORE_USART1_PORT GPIOC
#define CORE_USART1_PINS (GPIO_PIN_4 | GPIO_PIN_5)
#define CORE_USART2_PORT GPIOC
#define CORE_USART2_PINS (GPIO_PIN_4 | GPIO_PIN_5)
#define CORE_USART3_PORT GPIOC
#define CORE_USART3_PINS (GPIO_PIN_4 | GPIO_PIN_5)

bool core_USART_init(USART_TypeDef *usart);
uint8_t core_USART_read_buffer(USART_TypeDef *usart, uint8_t *dest);
bool core_USART_enable_rx(USART_TypeDef *usart);
bool core_USART_disable_rx(USART_TypeDef *usart);
bool core_USART_transmit(USART_TypeDef *usart, uint8_t *txbuf, uint8_t txbuflen);

#endif
