#ifndef USART_H
#define USART_H
#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_usart.h>

#define CORE_USART_RXBUFLEN 128
#define CORE_USART_RX_TIMEOUT 16

#define CORE_USART1_PORT GPIOC
#define CORE_USART1_PINS (GPIO_PIN_4 | GPIO_PIN_5)
#define CORE_USART2_PORT GPIOB
#define CORE_USART2_PINS (GPIO_PIN_3 | GPIO_PIN_4)
#define CORE_USART3_PORT GPIOB
#define CORE_USART3_PINS (GPIO_PIN_8 | GPIO_PIN_9)

#define CORE_USART1_UPDATE 0x01
#define CORE_USART2_UPDATE 0x02
#define CORE_USART3_UPDATE 0x04

bool core_USART_init(USART_TypeDef *usart, uint32_t baud);
bool core_USART_start_rx(USART_TypeDef *usart, volatile uint8_t *rxbuf, volatile uint32_t *rxbuflen);
void core_USART_update_disable(USART_TypeDef *usart);
void core_USART_update_enable(USART_TypeDef *usart);
bool core_USART_transmit(USART_TypeDef *usart, uint8_t *txbuf, uint8_t txbuflen);
void core_USART_RX_callback(USART_HandleTypeDef *husart);

#endif
