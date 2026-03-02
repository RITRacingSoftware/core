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

static UART_HandleTypeDef usart1;
static UART_HandleTypeDef usart2;
static UART_HandleTypeDef usart3;
static UART_HandleTypeDef uart4;
static UART_HandleTypeDef uart5;

#if defined(CORE_USART_UPRINTF) && (CORE_USART_UPRINTF != 0)
uint8_t core_USART_usartbuf[CORE_USART_TXBUFLEN];
#endif

#ifndef CORE_USART1_IRQ_PRIO
#define CORE_USART1_IRQ_PRIO    0
#endif
#ifndef CORE_USART2_IRQ_PRIO
#define CORE_USART2_IRQ_PRIO    0
#endif
#ifndef CORE_USART3_IRQ_PRIO
#define CORE_USART3_IRQ_PRIO    0
#endif
//#ifndef CORE_UART4_IRQ_PRIO
//#define CORE_UART4_IRQ_PRIO     0
//#endif
#ifndef CORE_UART5_IRQ_PRIO
#define CORE_UART5_IRQ_PRIO     0
#endif

core_USART_module_t *core_USART_convert(USART_TypeDef *usart) {
    if (usart == USART1) return &usart1;
    if (usart == USART2) return &usart2;
    if (usart == USART3) return &usart3;
    if (usart == UART4) return &uart4;
    if (usart == UART5) return &uart5;
    return NULL;
}

bool core_USART_init(USART_TypeDef *usart, uint32_t baud) {
    core_USART_module_t *p_usart = core_USART_convert(usart);
    GPIO_InitTypeDef usartGPIO = {0, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, 0};
    p_usart->husart.Instance = usart;
    if (usart == USART1) {
        usartGPIO.Pin = CORE_USART1_TX_PIN; usartGPIO.Alternate = CORE_USART1_TX_AF;
        core_clock_port_init(CORE_USART1_TX_PORT);
        HAL_GPIO_Init(CORE_USART1_TX_PORT, &usartGPIO);
        usartGPIO.Pin = CORE_USART1_RX_PIN; usartGPIO.Alternate = CORE_USART1_RX_AF;
        core_clock_port_init(CORE_USART1_RX_PORT);
        HAL_GPIO_Init(CORE_USART1_RX_PORT, &usartGPIO);
    } else if (usart == USART2) {
        usartGPIO.Pin = CORE_USART2_TX_PIN; usartGPIO.Alternate = CORE_USART2_TX_AF;
        core_clock_port_init(CORE_USART2_TX_PORT);
        HAL_GPIO_Init(CORE_USART2_TX_PORT, &usartGPIO);
        usartGPIO.Pin = CORE_USART2_RX_PIN; usartGPIO.Alternate = CORE_USART2_RX_AF;
        core_clock_port_init(CORE_USART2_RX_PORT);
        HAL_GPIO_Init(CORE_USART2_RX_PORT, &usartGPIO);
    } else if (usart == USART3) {
        usartGPIO.Pin = CORE_USART3_TX_PIN; usartGPIO.Alternate = CORE_USART3_TX_AF;
        core_clock_port_init(CORE_USART3_TX_PORT);
        HAL_GPIO_Init(CORE_USART3_TX_PORT, &usartGPIO);
        usartGPIO.Pin = CORE_USART3_RX_PIN; usartGPIO.Alternate = CORE_USART3_RX_AF;
        core_clock_port_init(CORE_USART3_RX_PORT);
        HAL_GPIO_Init(CORE_USART3_RX_PORT, &usartGPIO);
    } else if (usart == UART4) {
        usartGPIO.Pin = CORE_UART4_TX_PIN; usartGPIO.Alternate = CORE_UART4_TX_AF;
        core_clock_port_init(CORE_UART4_TX_PORT);
        HAL_GPIO_Init(CORE_UART4_TX_PORT, &usartGPIO);
        usartGPIO.Pin = CORE_UART4_RX_PIN; usartGPIO.Alternate = CORE_UART4_RX_AF;
        core_clock_port_init(CORE_UART4_RX_PORT);
        HAL_GPIO_Init(CORE_UART4_RX_PORT, &usartGPIO);
    } else if (usart == UART5) {
        usartGPIO.Pin = CORE_UART5_TX_PIN; usartGPIO.Alternate = CORE_UART5_TX_AF;
        core_clock_port_init(CORE_UART5_TX_PORT);
        HAL_GPIO_Init(CORE_UART5_TX_PORT, &usartGPIO);
        usartGPIO.Pin = CORE_UART5_RX_PIN; usartGPIO.Alternate = CORE_UART5_RX_AF;
        core_clock_port_init(CORE_UART5_RX_PORT);
        HAL_GPIO_Init(CORE_UART5_RX_PORT, &usartGPIO);
    } else return false;
    
    if (!core_clock_USART_init(usart)) return false;

    p_usart->husart.Instance = usart;
    p_usart->husart.Init.BaudRate = baud;
    p_usart->husart.Init.WordLength = USART_WORDLENGTH_8B;
    p_usart->husart.Init.StopBits = USART_STOPBITS_1;
    p_usart->husart.Init.Parity = USART_PARITY_NONE;
    p_usart->husart.Init.Mode = USART_MODE_TX_RX;
    p_usart->husart.Init.ClockPrescaler = USART_PRESCALER_DIV4;
    p_usart->husart.FifoMode = USART_FIFOMODE_ENABLE;

    if (HAL_UART_Init(&(p_usart->husart)) != HAL_OK) return false;
    // Put the USART in asynchronous mode
    usart->CR1 &= ~USART_CR1_UE;
    usart->CR2 &= ~USART_CR2_CLKEN;
    usart->CR1 |= USART_CR1_UE;
    return true;
}

bool core_USART_register_callback(USART_TypeDef *usart, void (*callback)(uint8_t *, uint32_t)) {
    core_USART_module_t *p_usart = core_USART_convert(usart);
    if (usart == USART1) {
        HAL_NVIC_SetPriority(USART1_IRQn, CORE_USART1_IRQ_PRIO, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    } else if (usart == USART2) {
        HAL_NVIC_SetPriority(USART2_IRQn, CORE_USART2_IRQ_PRIO, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    } else if (usart == USART3) {
        HAL_NVIC_SetPriority(USART3_IRQn, CORE_USART3_IRQ_PRIO, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    } else if (usart == UART4) {
        HAL_NVIC_SetPriority(UART4_IRQn, CORE_UART4_IRQ_PRIO, 0);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
    } else if (usart == UART5) {
        HAL_NVIC_SetPriority(UART5_IRQn, CORE_UART5_IRQ_PRIO, 0);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
    } else return false;
    p_usart->callback = callback;
    // Enable the receiver timeout
    usart->RTOR = (usart->RTOR & 0xff000000) | CORE_USART_RX_TIMEOUT;
    usart->CR1 |= USART_CR1_RTOIE | USART_CR1_RXNEIE;
    usart->CR2 |= USART_CR2_RTOEN;
    return true;
}

static void USART_IRQHandler(USART_TypeDef *usart) {
    core_USART_module_t *p_usart = core_USART_convert(usart);
    uint32_t flags = usart->ISR;
    if (flags & USART_ISR_RXNE) {
        // Receive data
        if (p_usart->rxbuflen_int < CORE_USART_RXBUFLEN) {
            p_usart->rxbuf_int[p_usart->rxbuflen_int++] = usart->RDR;
        } else {
            uint8_t dummy = usart->RDR;
        }
    }
    if (flags & USART_ISR_RTOF) {
        // Timed out (end of transmission)
        if (p_usart->callback) {
            p_usart->callback(p_usart->rxbuf_int, p_usart->rxbuflen_int);
        }
        p_usart->rxbuflen_int = 0;
        usart->ICR = USART_ICR_RTOCF;
    }
    if (flags & USART_ISR_ORE) {
        usart->ICR = USART_ICR_ORECF;
    }

}

void USART1_IRQHandler() {USART_IRQHandler(USART1);}
void USART2_IRQHandler() {USART_IRQHandler(USART2);}
void USART3_IRQHandler() {USART_IRQHandler(USART3);}
void UART4_IRQHandler() {USART_IRQHandler(UART4);}
void UART5_IRQHandler() {USART_IRQHandler(UART5);}

bool core_USART_transmit(USART_TypeDef *usart, uint8_t *txbuf, uint8_t txbuflen) {
    UART_HandleTypeDef *husart = &(core_USART_convert(usart)->husart);
    return HAL_UART_Transmit(husart, txbuf, txbuflen, 0xffffffff) == HAL_OK;
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

