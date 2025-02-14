/**
  * @file   usart.c
  * @brief  Core UART library
  *
  * This core library component is used to initialize USARTs, transmit data 
  * over USART, and asynchronously receive data over USART.
  *
  * ## Initialization
  * To initialize a USART for transmitting, user code must call the function
  * core_USART_init() and specify the desired baud rate. To initialize a USART
  * for receiving, user code must call the function core_USART_start_rx().
  *
  * ## Enabling and disabling
  * When the USART module is configured for receiving, and it has not received
  * any bytes for a given time, the USART hardware triggers an interrupt, which
  * will copy the data in the internal RX buffer to the buffer passed to 
  * core_USART_start_rx(). If another function was processing the data in this
  * buffer when the interrupt is triggered, then the data will be corrupted.
  * Thus, the user code must call core_USART_update_disable() before processing
  * received data and core_USART_update_enable() when it is done
  *
  * @note   Any data received while updating is disabled will be lost.
  *
  */

#include "usart.h"
#include "core_config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_usart.h>


#include "clock.h"
#include "gpio.h"
#include "core_config.h"

static uint8_t core_USART1_rxbuf_int[CORE_USART_RXBUFLEN];
static uint32_t core_USART1_rxbuflen_int;
static volatile uint8_t *core_USART1_rxbuf;
static volatile uint32_t *core_USART1_rxbuflen;
static void (*core_USART1_callback)(uint8_t *, uint32_t);
static uint8_t core_USART2_rxbuf_int[CORE_USART_RXBUFLEN];
static uint32_t core_USART2_rxbuflen_int;
static volatile uint8_t *core_USART2_rxbuf;
static volatile uint32_t *core_USART2_rxbuflen;
static void (*core_USART2_callback)(uint8_t *, uint32_t);
static uint8_t core_USART3_rxbuf_int[CORE_USART_RXBUFLEN];
static uint32_t core_USART3_rxbuflen_int;
static volatile uint8_t *core_USART3_rxbuf;
static volatile uint32_t *core_USART3_rxbuflen;
static void (*core_USART3_callback)(uint8_t *, uint32_t);

static volatile uint8_t core_USART_flags;

static USART_HandleTypeDef usart1;
static USART_HandleTypeDef usart2;
static USART_HandleTypeDef usart3;

#ifdef CORE_USART_UPRINTF
uint8_t core_USART_usartbuf[CORE_USART_TXBUFLEN];
#endif

/**
  * @brief  Initialize a USART module in asynchronous mode with the given baud
  *         rate.
  * @param  usart The USART module to initialize
  * @param  baud Baud rate
  * @retval 0 if the given USART is not valid of if the initialization failed
  * @retval 1 otherwise
  */
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

/**
  * @brief  Start the receiver for the given USART module
  * @param  usart The USART module
  * @param  rxbuf Location where received data from the USART should be stored
  * @param  rxbuflen Location where the number of received bytes should be stored
  * @retval 0 if the given USART is not valid
  * @retval 1 otherwise
  */
bool core_USART_start_rx(USART_TypeDef *usart, volatile uint8_t *rxbuf, volatile uint32_t *rxbuflen) {
    if (usart == USART1) {
        core_USART1_rxbuf = rxbuf;
        core_USART1_rxbuflen = rxbuflen;
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
        __HAL_USART_ENABLE_IT(&usart1, USART_IT_RXNE);
    } else if (usart == USART2) {
        core_USART2_rxbuf = rxbuf;
        core_USART2_rxbuflen = rxbuflen;
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        __HAL_USART_ENABLE_IT(&usart2, USART_IT_RXNE);
    } else if (usart == USART3) {
        core_USART3_rxbuf = rxbuf;
        core_USART3_rxbuflen = rxbuflen;
        HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
        __HAL_USART_ENABLE_IT(&usart3, USART_IT_RXNE);
    } else return false;
    // Enable the receiver timeout
    usart->RTOR = (usart->RTOR & 0xff000000) | CORE_USART_RX_TIMEOUT;
    usart->CR1 |= USART_CR1_RTOIE;
    usart->CR2 |= USART_CR2_RTOEN;
    core_USART_update_enable(usart);
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
        if (core_USART1_rxbuf && core_USART1_rxbuflen && (core_USART_flags & CORE_USART1_UPDATE)) {
            for (uint32_t i=0; i < core_USART1_rxbuflen_int; i++) core_USART1_rxbuf[i] = core_USART1_rxbuf_int[i];
            *core_USART1_rxbuflen = core_USART1_rxbuflen_int;
        }
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
        if (core_USART2_rxbuf && core_USART2_rxbuflen && (core_USART_flags & CORE_USART2_UPDATE)) {
            for (uint32_t i=0; i < core_USART2_rxbuflen_int; i++) core_USART2_rxbuf[i] = core_USART2_rxbuf_int[i];
            *core_USART2_rxbuflen = core_USART2_rxbuflen_int;
        }
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
        if (core_USART3_rxbuf && core_USART3_rxbuflen && (core_USART_flags & CORE_USART3_UPDATE)) {
            for (uint32_t i=0; i < core_USART3_rxbuflen_int; i++) core_USART3_rxbuf[i] = core_USART3_rxbuf_int[i];
            *core_USART3_rxbuflen = core_USART3_rxbuflen_int;
        }
        if (core_USART3_callback) {
            core_USART3_callback(core_USART3_rxbuf_int, core_USART3_rxbuflen_int);
        }
        core_USART3_rxbuflen_int = 0;
        USART3->ICR = USART_ICR_RTOCF;
    }
}

/**
  * @brief  Disable updating the RX buffer for the given USART. Use this
  *         before reading from the buffer to which data is stored to 
  *         prevent corruption
  * @param  usart The USART module
  */
void core_USART_update_disable(USART_TypeDef *usart) {
    if (usart == USART1) core_USART_flags &= ~CORE_USART1_UPDATE;
    else if (usart == USART2) core_USART_flags &= ~CORE_USART2_UPDATE;
    else if (usart == USART3) core_USART_flags &= ~CORE_USART3_UPDATE;
}

/**
  * @brief  Enable updating the RX buffer for the given USART.
  * @note   Receiving is disabled by default. Make sure to call
  *         core_USART_update_enable after setting up the receiver with
  *         core_USART_start_rx.
  * @param  usart The USART module
  */
void core_USART_update_enable(USART_TypeDef *usart) {
    if (usart == USART1) core_USART_flags |= CORE_USART1_UPDATE;
    else if (usart == USART2) core_USART_flags |= CORE_USART2_UPDATE;
    else if (usart == USART3) core_USART_flags |= CORE_USART3_UPDATE;
}

/**
  * @brief  Transmit data from a USART
  * @note   This function is blocking and will not return until all data has
  *         been transmitted.
  * @param  usart The USART module
  * @param  txbuf Location where the data to be transmitted is read from
  * @param  txbuflen Number of bytes to transmit
  * @retval 1 if transmission was successful
  * @retval 0 otherwise
  */
bool core_USART_transmit(USART_TypeDef *usart, uint8_t *txbuf, uint8_t txbuflen) {
    USART_HandleTypeDef *husart;
    if (usart == USART1) husart = &usart1;
    else if (usart == USART2) husart = &usart2;
    else if (usart == USART3) husart = &usart3;
    else return false;
    return HAL_USART_Transmit(husart, txbuf, txbuflen, 0xffffffff) == HAL_OK;
}

#ifdef CORE_USART_UPRINTF
/**
  * @brief  Print a formatted string to a USART
  * @note   This function is blocking and will not return until all data has
  *         been transmitted.
  * @param  usart The USART module
  * @param  format Format string
  * @param  txbuflen Number of bytes to transmit
  * @return Returns -1 if the transmission failed. Otherwise, returns the
  *         number of transmitted bytes.
  */
int uprintf(USART_TypeDef *usart, const char *format, ...) {
    va_list args;
    va_start(args, format);
    int n = vsprintf(core_USART_usartbuf, format, args);
    va_end(args);
    if (core_USART_transmit(usart, core_USART_usartbuf, n)) return n;
    else return -1;
}
#endif

