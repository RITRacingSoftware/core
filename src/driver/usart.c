#include <stdint.h>
#include <stdbool.h>
#include "usart.h"
#include "clock.h"
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_usart.h>

uint8_t core_USART1_rxbuf[128];
uint8_t core_USART1_rxbuflen;
uint8_t core_USART2_rxbuf[128];
uint8_t core_USART2_rxbuflen;
uint8_t core_USART3_rxbuf[128];
uint8_t core_USART3_rxbuflen;

USART_HandleTypeDef usart1;
USART_HandleTypeDef usart2;
USART_HandleTypeDef usart3;

bool core_USART_init(USART_TypeDef *usart) {
    USART_HandleTypeDef *husart;
    GPIO_InitTypeDef usartGPIO = {0, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, 7};
    if (usart == USART1) {
        usartGPIO.Pin = CORE_USART1_PINS;
        core_clock_port_init(CORE_USART1_PORT);
        HAL_GPIO_Init(CORE_USART1_PORT, &usartGPIO);
        husart = &usart1;
    } else if (usart == USART2) {
        usartGPIO.Pin = CORE_USART2_PINS;
        core_clock_port_init(CORE_USART2_PORT);
        HAL_GPIO_Init(CORE_USART2_PORT, &usartGPIO);
        husart = &usart2;
    } else if (usart == USART3) {
        usartGPIO.Pin = CORE_USART3_PINS;
        core_clock_port_init(CORE_USART3_PORT);
        HAL_GPIO_Init(CORE_USART3_PORT, &usartGPIO);
        husart = &usart3;
    } else return false;
    
    if (!core_clock_USART_init(usart)) return false;

    husart->Instance = usart;
    husart->Init.BaudRate = 9600;
    husart->Init.WordLength = USART_WORDLENGTH_8B;
    husart->Init.StopBits = USART_STOPBITS_1;
    husart->Init.Parity = USART_PARITY_NONE;
    husart->Init.Mode = USART_MODE_TX_RX;
    husart->Init.ClockPrescaler = USART_PRESCALER_DIV4;
    husart->FifoMode = USART_FIFOMODE_ENABLE;
    husart->SlaveMode = USART_SLAVEMODE_DISABLE;

    return HAL_USART_Init(husart) == HAL_OK;
}

/*uint8_t core_USART_read_buffer(USART_TypeDef *usart, uint8_t *dest) {
    core_USART_disable_rx(usart);
    if (usart == USART1) {
        uint8_t len = core_USART1_rxbuflen;
        for (uint8_t i=0; i < core_USART1_rxbuflen; i++) dest[i] = core_USART1_rxbuf[i];
        core_USART1_rxbuflen = 0;
    } else if (usart == USART2) {
        uint8_t len = core_USART2_rxbuflen;
        for (uint8_t i=0; i < core_USART2_rxbuflen; i++) dest[i] = core_USART2_rxbuf[i];
        core_USART2_rxbuflen = 0;
    } else if (usart == USART3) {
        uint8_t len = core_USART3_rxbuflen;
        for (uint8_t i=0; i < core_USART3_rxbuflen; i++) dest[i] = core_USART3_rxbuf[i];
        core_USART3_rxbuflen = 0;
    } else return 0;
    core_USART_enable_rx(usart);
    return 0;
}

bool core_USART_enable_rx(USART_TypeDef *usart) {

}

bool core_USART_disable_rx(USART_TypeDef *usart) {

}*/

bool core_USART_transmit(USART_TypeDef *usart, uint8_t *txbuf, uint8_t txbuflen) {
    USART_HandleTypeDef *husart;
    if (usart == USART1) husart = &usart1;
    else if (usart == USART2) husart = &usart2;
    else if (usart == USART3) husart = &usart3;
    else return false;
    return HAL_USART_Transmit(husart, txbuf, txbuflen, 0xffffffff) == HAL_OK;
}

