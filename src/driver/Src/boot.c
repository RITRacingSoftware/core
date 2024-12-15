#include <stdint.h>
#include "usart.h"
#include "clock.h"
#include "core_config.h"
#include "stm32g4xx_hal.h"

/**
  * @brief  Main code for the bootloader
  *
  * This code may not call any other functions unless they are located in the
  * .bootmain section
  */
static void __attribute__ ((section (".bootmain"))) __attribute__ ((__used__)) boot() {
    __disable_irq();
    uint16_t timeout = CORE_BOOT_USART_TIMEOUT;
    SysTick->VAL = SysTick->LOAD;
    while (timeout > 0) {
        if (USART2->ISR & USART_ISR_RXNE) break;
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) timeout--;
    }
    if (timeout == 0) {
        // No start signal detected, leave boot mode
        __enable_irq();
        return;
    }
    USART2->ICR = USART_ICR_RTOCF;
    uint16_t data = USART2->RDR;
    for (int i=0; i < 10000; i++)
    while (!(USART2->ISR & USART_ISR_RTOF)) {
        if (USART2->ISR & USART_ISR_RXNE) {
            while (1) {
                GPIOA->BSRR = (1<<5);
                for (int i=0; i < 2000000; i++);
                GPIOA->BSRR = (1<<21);
                for (int i=0; i < 5000000; i++);
            }
        }
    }
    GPIOA->BSRR = (1<<5);
    __enable_irq();
}


void core_boot_init() {
    core_USART_init(USART1, 500000);
    __HAL_RCC_USART2_CONFIG(RCC_USART2CLKSOURCE_PCLK1);
    __HAL_RCC_USART2_CLK_ENABLE();

    core_clock_port_init(CORE_BOOT_PORT);
    core_clock_port_init(GPIOB);
    GPIO_InitTypeDef usartGPIO = {CORE_BOOT_PIN, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, CORE_BOOT_AF};
    HAL_GPIO_Init(CORE_BOOT_PORT, &usartGPIO);

    USART2->CR1 = USART_CR1_FIFOEN | USART_CR1_RE;
    USART2->CR2 = USART_CR2_RTOEN;
    USART2->CR3 = 0;
    USART2->BRR = 1000 * CORE_CLOCK_SYSCLK_FREQ / CORE_BOOT_USART_BAUD;
    USART2->RTOR = (USART2->RTOR & 0xff000000) | (CORE_BOOT_USART_BAUD * CORE_BOOT_USART_TIMEOUT / 1000);
    USART2->PRESC = 0;
    USART2->CR1 |= USART_CR1_UE;
    // Configure the systick timer for a 1ms period
    SysTick->LOAD = CORE_CLOCK_SYSCLK_FREQ;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x00000005;
    uprintf(USART1, "divider: %08x\n", SysTick->LOAD);
    boot();
}
