#ifndef USART_H
#define USART_H
#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_usart.h>
#include "core_config.h"

#define CORE_USART1_UPDATE 0x01
#define CORE_USART2_UPDATE 0x02
#define CORE_USART3_UPDATE 0x04

bool core_USART_init(USART_TypeDef *usart, uint32_t baud);
bool core_USART_start_rx(USART_TypeDef *usart, volatile uint8_t *rxbuf, volatile uint32_t *rxbuflen);
void core_USART_update_disable(USART_TypeDef *usart);
void core_USART_update_enable(USART_TypeDef *usart);
bool core_USART_transmit(USART_TypeDef *usart, uint8_t *txbuf, uint8_t txbuflen);
void core_USART_RX_callback(USART_HandleTypeDef *husart);

#ifdef CORE_USART_UPRINTF
int uprintf(USART_TypeDef *usart, const char *format, ...);
#endif

#endif
