#include <stdint.h>
#include <stdbool.h>
#include "usart.h"
#include "clock.h"
#include "core_config.h"
#include "stm32g4xx_hal.h"

#define BOOTMAIN __attribute__ ((section (".bootmain"))) __attribute__ ((__used__))
#define BOOTSTART __attribute__ ((section (".bootstart"))) __attribute__ ((__used__))
#define BOOTSTATE __attribute__ ((section (".bootstate")))

#define ALTBANK_BASE 0x08040000

uint32_t boot_state BOOTSTATE;
// Stores the opcodes used for soft bank switching. Soft bank switching must be
// performed from RAM, since the code in the other bank may continue from a
// different address.
uint32_t boot_toggle[8];
uint32_t boot_reset_state;

static uint32_t page_erased[4];
static uint32_t base_address;
static uint32_t address;
static uint8_t rxbuf[100];
static uint8_t rxbuflen;
static uint8_t *databuf;
static uint8_t checksum;

void boot_reset() {
    // Clear reset flags from previous reset
    RCC->CSR |= 0x00800000;
    SCB->AIRCR  = (SCB->AIRCR & (0x700)) | 0x05fa0004;
    __DSB();
}

#define BOOT_STATE_KEY 0xABCDEF00
#define BOOT_STATE_NORMAL 0x00
#define BOOT_STATE_VERIFY 0x01
#define BOOT_STATE_VERIFY_SOFT_SWITCH 0x02
#define BOOT_STATE_SOFT_SWITCHED 0x04
#define BOOT_STATE_VERIFIED 0x08
#define BOOT_STATE_ERROR 0x80

/**
  * @brief  Soft bank swap
  *
  * This function is defined in startup_stm32g473xx.s
  */
extern void boot_soft_toggle();

uint32_t check_nonbooting() {
    return ((FLASH->OPTR >> 20) ^ (SYSCFG->MEMRMP >> 8)) & 1;
}

/**
  * @brief  Process the current boot state and swap banks if needed
  *
  * This function is called from startup_stm32g473xx.s and is called before the
  * HAL is initialized and before the RAM is initialized. The boot state is
  * preserved in a special section (.noinit) at the beginning of RAM that is
  * not initialized, so its contents are preserved between resets.
  */
void boot_state_machine() {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    uint32_t rst = RCC->CSR;
    SCB->VTOR = 0x08000000;
    boot_reset_state = rst;
    RCC->CSR = rst | 0x00800000;
    // If all reset flags are cleared, then a software reset has
    // occurred but has already been handled
    if (!(rst & 0xfe000000)) rst |= RCC_CSR_SFTRSTF;
    if (rst & RCC_CSR_BORRSTF) {
        // Power-on reset detected
        boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
    }
    if (rst & (RCC_CSR_SFTRSTF | RCC_CSR_OBLRSTF)) {
        // Internal reset or option byte reload
        // bankmode = 0 if booting bank mapped at address 0
        //            1 if non-booting bank is mapped at address 0
        if (check_nonbooting()) {
            // Non-booting bank
            if ((boot_state & 0xffffff00) != BOOT_STATE_KEY) {
                // Boot state register probably does not contain the boot
                // state (address mismatch?), reset to indicate soft 
                // switch failure
                boot_reset();
            }
            if ((boot_state & 0xff) == BOOT_STATE_VERIFY_SOFT_SWITCH) {
                // Expected state, continue to code for verification
                boot_state = BOOT_STATE_KEY | BOOT_STATE_SOFT_SWITCHED;
                return;
            } else {
                // Bad state, raise error and reset
                boot_state |= BOOT_STATE_ERROR;
                boot_reset();
            }
        } else {
            // Booting bank
            if ((boot_state & 0xffffff00) != BOOT_STATE_KEY) {
                // Boot state has not been initialized or has changed location.
                // Continue to main code and the bootloader will produce an
                // error message and change the boot state to NORMAL.
                return;
            }
            switch (boot_state & 0xff) {
                case BOOT_STATE_NORMAL:
                    return;
                case BOOT_STATE_VERIFY:
                    boot_state = BOOT_STATE_KEY | BOOT_STATE_VERIFY_SOFT_SWITCH;
                    boot_soft_toggle();
                default:
                    // If the boot state is not in one of the expected states,
                    // set the error flag and continue to main code. The
                    // bootloader will transmit an error message and reset the
                    // boot state to NORMAL
                    boot_state |= BOOT_STATE_ERROR | 0x40;
                    return;
            }
        }
    } else {
        // External reset
        boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
    }
}

static void boot_blink() {
    while (1) {
        GPIOA->BSRR = (1<<5);
        for (int i=0; i < 2000000; i++);
        GPIOA->BSRR = (1<<21);
        for (int i=0; i < 5000000; i++);
    }
}

static void boot_printhex(uint8_t data) {
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

static void boot_bankswap() {
    //SCB->AIRCR  = (NVIC_AIRCR_VECTKEY | (SCB->AIRCR & (0x700)) | (1<<NVIC_SYSRESETREQ)); /* Keep priority group unchanged */
    //SCB->AIRCR  = (SCB->AIRCR & (0x700)) | 0x05fa0004;
    //__DSB();

    // Unlock the flash
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    // Unlock the option bytes
    FLASH->OPTKEYR = 0x08192A3B;
    FLASH->OPTKEYR = 0x4C5D6E7F;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->OPTR ^= FLASH_OPTR_BFB2;
    FLASH->CR = FLASH_CR_OPTSTRT;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR = FLASH_CR_OBL_LAUNCH;
}

static uint8_t boot_await_data() {
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
            if (recv == 0xc0) {
                boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
                boot_reset();
            }
            if (recv == 0xc1) {
                // Failsafe bank swap
                boot_state = BOOT_STATE_KEY | BOOT_STATE_VERIFY;
                boot_reset();
            }
            if (recv == 0xc2) {
                // If the program is verified, swap the banks, set state to
                // NORMAL, and reset
                if ((check_nonbooting()) && (boot_state == (BOOT_STATE_KEY | BOOT_STATE_VERIFIED))) {
                    boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
                    boot_bankswap();
                }
            }
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

static void boot_echo_data(uint8_t length) {
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = ':';
    checksum = 0;
    boot_printhex(length);
    boot_printhex((address >> 11) & 0xff);
    boot_printhex((address >> 3) & 0xff);
    boot_printhex(rxbuf[3]);
    for (uint8_t i=0; i < length; i++) boot_printhex(databuf[i]);
    boot_printhex((-checksum) & 0xff);
    while (!(USART2->ISR & USART_ISR_TXE));
    USART2->TDR = '\n';
}

static uint8_t boot_program_and_verify(uint8_t length) {
    // If the length is not doubleword-aligned, return 0 (error).
    if (length & 0x07) return 0;
    // If the address is not doubleword-aligned, return 0 (error).
    if (address & 0x07) return 0;
    // In case the target section overlaps with bootloader code, return 0.
    // An empty echo frame will be transmitted, indicating an error.
    if (address + length > 0x40000) return 0;
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
        FLASH->CR = ((page & 0x7f) << FLASH_CR_PNB_Pos) | FLASH_CR_PER | ((SYSCFG->MEMRMP & (1<<8)) ? 0 : FLASH_CR_BKER);
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
        *(uint32_t*)(ALTBANK_BASE + address+i) = (uint32_t)temp;
        __ISB();
        *(uint32_t*)(ALTBANK_BASE + address+i+4) = (uint32_t)(temp >> 32);
        while (FLASH->SR & FLASH_SR_BSY);
        // Return 0 if an error occurred
        if (FLASH->SR & 0xfffe) return 0;
    }
    FLASH->CR = FLASH_CR_LOCK;
    for (uint8_t i=0; i < length; i++) {
        databuf[i] = *(uint8_t*)(ALTBANK_BASE + address+i);
    }
    return length;
}

/**
  * @brief  Main code for the bootloader
  *
  */
static void boot() {
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
        } else if (rxbuf[3] == 7) {
            // Verification command
            if (boot_state == (BOOT_STATE_KEY | BOOT_STATE_SOFT_SWITCHED)) {
                boot_state = BOOT_STATE_KEY | BOOT_STATE_VERIFIED;
            } else {
                boot_state = BOOT_STATE_KEY | BOOT_STATE_ERROR;
            }
            boot_echo_data(0);
        }
        if (ndata) {
            ndata = boot_program_and_verify(ndata);
            boot_echo_data(ndata);
        }

    }

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
    //uprintf(USART1, "divider: %08x\n", SysTick->LOAD);
    uint32_t temp = (boot_state & 0xffff) | ((SYSCFG->MEMRMP << 8) & 0x10000) | (FLASH->OPTR & (1<<20)) | (boot_reset_state & 0xff000000);
    core_USART_transmit(USART1, &temp, 4);
    if (!check_nonbooting()) {
        boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
    }
    boot();
}
