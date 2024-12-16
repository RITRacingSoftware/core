#include <stdint.h>
#include "usart.h"
#include "clock.h"
#include "core_config.h"
#include "stm32g4xx_hal.h"

#define BOOTMAIN __attribute__ ((section (".bootmain"))) __attribute__ ((__used__))
#define BOOTSTART __attribute__ ((section (".bootstart"))) __attribute__ ((__used__))

static uint32_t page_erased[8];
static uint32_t base_address;
static uint32_t address;
static uint8_t rxbuf[100];
static uint8_t rxbuflen;
static uint8_t *databuf;
static uint8_t checksum;


static void BOOTMAIN boot_blink() {
    while (1) {
        GPIOA->BSRR = (1<<5);
        for (int i=0; i < 2000000; i++);
        GPIOA->BSRR = (1<<21);
        for (int i=0; i < 5000000; i++);
    }
}

static void BOOTMAIN boot_printhex(uint8_t data) {
    checksum += data;
    uint8_t nibble = data>>4;
    if (nibble >= 10) nibble += 'A' - 10;
    else nibble += '0';
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = nibble;
    nibble = data & 0x0f;
    if (nibble >= 10) nibble += 'A' - 10;
    else nibble += '0';
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = nibble;
}

static void BOOTMAIN boot_reset() {
    //SCB->AIRCR  = (NVIC_AIRCR_VECTKEY | (SCB->AIRCR & (0x700)) | (1<<NVIC_SYSRESETREQ)); /* Keep priority group unchanged */
    SCB->AIRCR  = (SCB->AIRCR & (0x700)) | 0x05fa0004;
    __DSB();
}

static uint8_t BOOTMAIN boot_await_data() {
    uint8_t state = 0;
    uint8_t nibble = 0;
    char recv;
    rxbuflen = 0;
    // Wait for data to arrive
    while (1) {
        while (!(USART2->ISR & USART_ISR_RXNE));
        recv = USART2->RDR;
        if (recv & 0x80) {
            // Control data received
            if (recv == 0xc0) boot_reset();
        } else if (state == 0) {
            if (recv == ':') state = 1;
        } else if (state == 1) {
            // Receive HEX data
            if (recv == '\n') {
                // End of a HEX record. Return the length for data records,
                // and 0 for control records
                state = 0;
                if (rxbuf[3] == 0) {
                    // Note that the address here is the doubleword address, not
                    // the byte address
                    address = base_address + (rxbuf[1] << 11) + (rxbuf[2] << 3);
                    return rxbuf[0];
                }
                else if (rxbuf[3] == 2) {
                    base_address = (rxbuf[4] << 12) | (rxbuf[5] << 4);
                } else if (rxbuf[3] == 4) {
                    base_address = (rxbuf[4] << 24) | (rxbuf[5] << 16);
                }
                return 0;
            }
            else if (('0' <= recv) && ('9' >= recv)) {
                rxbuf[rxbuflen] = (rxbuf[rxbuflen]<<4) | (recv - '0');
            }
            else if (('a' <= recv) && ('f' >= recv)) {
                rxbuf[rxbuflen] = (rxbuf[rxbuflen]<<4) | (recv - 'a' + 10);
            }
            else if (('A' <= recv) && ('F' >= recv)) {
                rxbuf[rxbuflen] = (rxbuf[rxbuflen]<<4) | (recv - 'A' + 10);
            } else continue;
            if (nibble) rxbuflen++;
            nibble = 1 - nibble;
        }
    }
}

static void BOOTMAIN boot_echo_data(uint8_t length) {
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = ':';
    checksum = 0;
    boot_printhex(length);
    boot_printhex((address >> 11) & 0xff);
    boot_printhex((address >> 3) & 0xff);
    boot_printhex(0);
    for (uint8_t i=0; i < length; i++) boot_printhex(databuf[i]);
    boot_printhex((-checksum) & 0xff);
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = '\n';
}

static uint8_t BOOTMAIN boot_program_and_verify(uint8_t length) {
    // If the length is not doubleword-aligned, return 0 (error).
    if (length & 0x07) return 0;
    // If the address is not doubleword-aligned, return 0 (error).
    if (address & 0x07) return 0;
    // In case the target section overlaps with bootloader code, return 0.
    // An empty echo frame will be transmitted, indicating an error.
    if (address + length > 0x7c000) return 0;
    //return length;
    // Unlock the flash
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    uint8_t page;
    if (FLASH->OPTR & FLASH_OPTR_DBANK) page = address >> 11;
    else page = (address >> 12) & 0x7f;
    while (FLASH->SR & FLASH_SR_BSY);
    if (!((page_erased[page>>5]>>(page & 0x1f)) & 1)) {
        // Page has not been erased, so erase it
        // Clear error flags
        FLASH->SR = FLASH->SR & 0xffff;
        // Erase page
        FLASH->CR = ((page & 0x7f) << FLASH_CR_PNB_Pos) | FLASH_CR_PER | (page >= 0x80 ? FLASH_CR_BKER : 0);
        FLASH->CR |= FLASH_CR_STRT;
        page_erased[page>>5] |= 1<<(page & 0x1f);
        while (FLASH->SR & FLASH_SR_BSY);
        // Error while erasing
        if (FLASH->SR & 0xfffe) return 0;
    }
    FLASH->CR = FLASH_CR_PG;
    uint64_t temp;
    for (uint8_t i=0; i < length; i += 8) {
        // Clear error flags
        FLASH->SR = FLASH->SR & 0xffff;
        temp = ((uint64_t*)(databuf+i))[0];
        // Program the doubleword
        *(uint32_t*)(FLASH_BASE + address+i) = (uint32_t)temp;
        __ISB();
        *(uint32_t*)(FLASH_BASE + address+i+4) = (uint32_t)(temp >> 32);
        while (FLASH->SR & FLASH_SR_BSY);
        // Return 0 if an error occurred
        if (FLASH->SR & 0xfffe) return 0;
    }
    FLASH->CR = FLASH_CR_LOCK;
    for (uint8_t i=0; i < length; i++) {
        databuf[i] = *(uint8_t*)(FLASH_BASE + address+i);
    }
    return length;
}

/**
  * @brief  Main code for the bootloader
  *
  * This code may not call any other functions unless they are located in the
  * .bootmain section
  */
static void BOOTMAIN boot() {
    __disable_irq();
    // Point databuf to the first data byte in a HEX record
    databuf = rxbuf+4;
    // Wait for the USART start signal
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
    uint8_t address_nrecv = 0;
    while (!(USART2->ISR & USART_ISR_RTOF)) {
        if (USART2->ISR & USART_ISR_RXNE) {
            if (USART2->RDR == (0x80 | CORE_BOOT_ADDRESS)) address_nrecv++;
            else address_nrecv = 0;
            if (address_nrecv == CORE_BOOT_NRECV) break;
        }
    }
    if (address_nrecv < CORE_BOOT_NRECV) {
        // Address not detected, leave boot mode
        __enable_irq();
        return;
    }
    for (uint8_t i=0; i < 8; i++) page_erased[i] = 0;
    // Main programming loop
    uint8_t ndata = 0;
    base_address = 0;
    address = 0;
    while (1) {
        ndata = boot_await_data();
        if (rxbuf[3] == 6) {
            GPIOA->BSRR = (1<<5);
            for (int i=0; i < 2000000; i++);
            GPIOA->BSRR = (1<<21);
        }
        if (ndata) {
            ndata = boot_program_and_verify(ndata);
            boot_echo_data(ndata);
        }

    }

    __enable_irq();
}

static void BOOTSTART boot_start() {
    boot();
}


void core_boot_init() {
    core_USART_init(USART1, 500000);
    __HAL_RCC_USART2_CONFIG(RCC_USART2CLKSOURCE_PCLK1);
    __HAL_RCC_USART2_CLK_ENABLE();

    core_clock_port_init(CORE_BOOT_PORT);
    core_clock_port_init(GPIOB);
    GPIO_InitTypeDef usartGPIO = {CORE_BOOT_PIN, GPIO_MODE_AF_PP, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW, CORE_BOOT_AF};
    HAL_GPIO_Init(CORE_BOOT_PORT, &usartGPIO);
    usartGPIO.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &usartGPIO);

    USART2->CR1 = USART_CR1_FIFOEN | USART_CR1_RE | USART_CR1_TE;
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
    boot_start();
}
