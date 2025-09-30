#include "usart.h"
#include "timestamp.h"
#include "core_config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stm32g4xx_hal.h>


#include "clock.h"
#include "gpio.h"
#include "core_config.h"

static uint8_t core_USART1_rxbuf_int[CORE_USART_RXBUFLEN];
static uint32_t core_USART1_rxbuflen_int;
static void (*core_USART1_callback)(uint8_t *, uint32_t);
static uint8_t core_USART2_rxbuf_int[CORE_USART_RXBUFLEN];
static uint32_t core_USART2_rxbuflen_int;
static void (*core_USART2_callback)(uint8_t *, uint32_t);
static uint8_t core_USART3_rxbuf_int[CORE_USART_RXBUFLEN];
static uint32_t core_USART3_rxbuflen_int;
static void (*core_USART3_callback)(uint8_t *, uint32_t);

static volatile uint8_t core_USART_flags;

static USART_HandleTypeDef usart1;
static USART_HandleTypeDef usart2;
static USART_HandleTypeDef usart3;

#if defined(CORE_USART_UPRINTF) && (CORE_USART_UPRINTF != 0)
uint8_t core_USART_usartbuf[CORE_USART_TXBUFLEN];
#endif

bool core_USART_init(USART_TypeDef *usart, uint32_t baud) {
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
    husart->Init.BaudRate = baud;
    husart->Init.WordLength = USART_WORDLENGTH_8B;
    husart->Init.StopBits = USART_STOPBITS_1;
    husart->Init.Parity = USART_PARITY_NONE;
    husart->Init.Mode = USART_MODE_TX_RX;
    husart->Init.ClockPrescaler = USART_PRESCALER_DIV4;
    husart->FifoMode = USART_FIFOMODE_ENABLE;
    husart->SlaveMode = USART_SLAVEMODE_DISABLE;

    if (HAL_USART_Init(husart) != HAL_OK) return false;
    // Put the USART in asynchronous mode
    usart->CR1 &= ~USART_CR1_UE;
    usart->CR2 &= ~USART_CR2_CLKEN;
    usart->CR1 |= USART_CR1_UE;
    return true;
}

bool core_USART_register_callback(USART_TypeDef *usart, void (*callback)(uint8_t *, uint32_t)) {
    if (usart == USART1) {
        core_USART1_callback = callback;
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
        __HAL_USART_ENABLE_IT(&usart1, USART_IT_RXNE);
    } else if (usart == USART2) {
        core_USART2_callback = callback;
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        __HAL_USART_ENABLE_IT(&usart2, USART_IT_RXNE);
    } else if (usart == USART3) {
        core_USART3_callback = callback;
        HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
        __HAL_USART_ENABLE_IT(&usart3, USART_IT_RXNE);
    } else return false;
    // Enable the receiver timeout
    usart->RTOR = (usart->RTOR & 0xff000000) | CORE_USART_RX_TIMEOUT;
    usart->CR1 |= USART_CR1_RTOIE;
    usart->CR2 |= USART_CR2_RTOEN;
    return true;
}

void USART1_IRQHandler() {
    uint32_t flags = USART1->ISR;
    if (flags & USART_ISR_RXNE) {
        // Receive data
        core_USART1_rxbuf_int[core_USART1_rxbuflen_int++] = USART1->RDR;
    }
    if (flags & USART_ISR_RTOF) {
        // Timed out (end of transmission)
        if (core_USART1_callback) {
            core_USART1_callback(core_USART1_rxbuf_int, core_USART1_rxbuflen_int);
        }
        core_USART1_rxbuflen_int = 0;
        USART1->ICR = USART_ICR_RTOCF;
    }
}

void USART2_IRQHandler() {
    uint32_t flags = USART2->ISR;
    if (flags & USART_ISR_RXNE) {
        // Receive data
        core_USART2_rxbuf_int[core_USART2_rxbuflen_int++] = USART2->RDR;
    }
    if (flags & USART_ISR_RTOF) {
        // Timed out (end of transmission)
        if (core_USART2_callback) {
            core_USART2_callback(core_USART2_rxbuf_int, core_USART2_rxbuflen_int);
        }
        core_USART2_rxbuflen_int = 0;
        USART2->ICR = USART_ICR_RTOCF;
    }
}

void USART3_IRQHandler() {
    uint32_t flags = USART3->ISR;
    if (flags & USART_ISR_RXNE) {
        // Receive data
        core_USART3_rxbuf_int[core_USART3_rxbuflen_int++] = USART3->RDR;
    }
    if (flags & USART_ISR_RTOF) {
        // Timed out (end of transmission)
        if (core_USART3_callback) {
            core_USART3_callback(core_USART3_rxbuf_int, core_USART3_rxbuflen_int);
        }
        core_USART3_rxbuflen_int = 0;
        USART3->ICR = USART_ICR_RTOCF;
    }
}

bool core_USART_transmit(USART_TypeDef *usart, uint8_t *txbuf, uint8_t txbuflen) {
    USART_HandleTypeDef *husart;
    if (usart == USART1) husart = &usart1;
    else if (usart == USART2) husart = &usart2;
    else if (usart == USART3) husart = &usart3;
    else return false;
    return HAL_USART_Transmit(husart, txbuf, txbuflen, 0xffffffff) == HAL_OK;
}

#if (CORE_USART_UPRINTF != 0) || defined(DOXYGEN)
int uprintf(USART_TypeDef *usart, const char *format, ...) {
    va_list args;
    va_start(args, format);
    int n = vsprintf(core_USART_usartbuf, format, args);
    va_end(args);
    if (core_USART_transmit(usart, core_USART_usartbuf, n)) return n;
    else return -1;
}
#endif

uint32_t core_USART_receive(USART_TypeDef *usart, uint8_t *rxbuf, uint32_t rxbuflen, uint32_t timeout) {
    uint32_t nrx = 0;
    uint32_t t0 = core_timestamp_get_tick();
    uint32_t indata;
    while (1) {
        if (usart->ISR & USART_ISR_RXNE) {
            if (rxbuflen > 0) {
                rxbuf[nrx++] = usart->RDR;
                if (nrx >= rxbuflen) return nrx;
            } else indata = usart->RDR;
        }
        if (core_timestamp_get_tick() - t0 >= timeout) return nrx;
    }
}

