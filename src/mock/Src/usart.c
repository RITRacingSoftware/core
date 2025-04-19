#include usart.h

__weak bool core_USART_init(USART_TypeDef *usart, uint32_t baud) {}
__weak bool core_USART_start_rx(USART_TypeDef *usart, volatile uint8_t *rxbuf, volatile uint32_t *rxbuflen) {}
__weak void core_USART_update_disable(USART_TypeDef *usart) {}
__weak void core_USART_update_enable(USART_TypeDef *usart) {}
__weak bool core_USART_transmit(USART_TypeDef *usart, uint8_t *txbuf, uint8_t txbuflen) {}
__weak void core_USART_RX_callback(USART_HandleTypeDef *husart) {}
__weak bool core_USART_register_callback(USART_TypeDef *usart, void (*callback)(uint8_t *, uint32_t)) {}
__weak int uprintf(USART_TypeDef *usart, const char *format, ...) {}
