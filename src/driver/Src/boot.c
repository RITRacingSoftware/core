#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "usart.h"
#include "gpio.h"
#include "clock.h"
#include "can.h"
#include "error_handler.h"
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
static uint8_t databuf[100];

static FDCAN_HandleTypeDef *hfdcan;

void boot_reset() {
    // Clear reset flags from previous reset
    RCC->CSR |= 0x00800000;
    SCB->AIRCR  = (SCB->AIRCR & (0x700)) | 0x05fa0004;
    __DSB();
}

#define BOOT_STATE_KEY          0xABCDEF00
#define BOOT_STATE_NORMAL       0x00
#define BOOT_STATE_VERIFY       0x01
#define BOOT_STATE_VERIFY_SOFT_SWITCH 0x02
#define BOOT_STATE_SOFT_SWITCHED 0x04
#define BOOT_STATE_VERIFIED     0x08
#define BOOT_STATE_ENTER        0x10
#define BOOT_STATE_ERROR        0x80
#define BOOT_STATE_NB_ERROR     0x40

#define BOOT_STATUS_OK          0x00
#define BOOT_STATUS_INVALID_ADDRESS 0x01
#define BOOT_STATUS_ERASE_ERROR 0x02
#define BOOT_STATUS_PROG_ERROR  0x03
#define BOOT_STATUS_STATE_ERROR 0x04
#define BOOT_STATUS_NB_ERROR    0x05
#define BOOT_STATUS_NONBOOTING_PROGRAM 0x06

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
  * preserved in a special section (.bootstate) at the end of the RAM that is
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
            // If the ENTER bit is set, it must be preserved
            if ((boot_state & 0xff & (~BOOT_STATE_ENTER)) == BOOT_STATE_VERIFY_SOFT_SWITCH) {
                // Expected state and no errors, continue to code for verification
                boot_state = BOOT_STATE_KEY | BOOT_STATE_SOFT_SWITCHED | (boot_state & BOOT_STATE_ENTER);
                return;
            } else {
                // Bad state, raise error and reset
                boot_state |= BOOT_STATE_NB_ERROR;
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
            if (boot_state & (BOOT_STATE_NB_ERROR | BOOT_STATE_ERROR)) {
                // An error was reported, continue to main code and the
                // bootloader will produce an error message and change the boot
                // state to NORMAL.
                return;
            }
            switch (boot_state & 0x0f) {
                case BOOT_STATE_NORMAL:
                case BOOT_STATE_ENTER:
                    return;
                case BOOT_STATE_VERIFY:
                    // Soft bank switching can only occur when there are no errors.
                    boot_state = (boot_state & 0xfffffff0) | BOOT_STATE_VERIFY_SOFT_SWITCH;
                    boot_soft_toggle();
                default:
                    // If the boot state is not in one of the expected states,
                    // set the error flag and continue to main code. The
                    // bootloader will transmit an error message and reset the
                    // boot state to NORMAL
                    boot_state |= BOOT_STATE_ERROR;
                    return;
            }
        }
    } else {
        // External reset
        boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
    }
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
    __DSB();
}

void boot_transmit_can(uint8_t length, bool is_data) {
    uint32_t id;
    id = (CORE_BOOT_FDCAN_MASTER_ID << 18) | (1<<17) | address;
    if (is_data) id |= (1<<16);
    if (length > 64) length = 64;
    if ((length >= 32) && (length & 8)) {
        length += 8;
        id |= (1<<15);
    }
    core_CAN_send_fd_message(CORE_BOOT_FDCAN, id, length, databuf);
}

void boot_transmit_status(uint8_t code) {
    address = 0;
    databuf[0] = code;
    // bit 0: remap status (which bank the code runs on)
    // bit 1: BFB2 (which bank the chip boots from)
    databuf[1] = ((FLASH->OPTR >> 19) & 2) | ((SYSCFG->MEMRMP >> 8) & 1);
    databuf[2] = (FLASH->SR) & 0xff;
    databuf[3] = (FLASH->SR >> 8) & 0xff;
    memcpy(databuf+4, &boot_state, 4);
    boot_transmit_can(8, 0);
}

static uint8_t boot_await_data() {
    while (!(CORE_BOOT_FDCAN->RXF0S & FDCAN_RXF0S_F0FL));
    // Data received over CAN
    FDCAN_RxHeaderTypeDef head;
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &head, databuf);
    uint32_t id = head.Identifier;
    uint32_t length = core_CAN_dlc_lookup[head.DataLength];
    if ((id >> 18) != CORE_BOOT_FDCAN_ID) return 0;
    if (id & (1<<17)) {
        // Data frame
        if (id & (1<<15)) length -= 8;
        address = (id & 0x7fff) << 3;
        return length;
    } else {
        // Control frame
        if (databuf[0] == 0x00) {
            boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
            boot_reset();
        } else if (databuf[0] == 0x01) {
            // Failsafe bank swap
            boot_state = BOOT_STATE_KEY | BOOT_STATE_VERIFY;
            boot_reset();
        } else if (databuf[0] == 0x02) {
            // Verification command
            if (boot_state == (BOOT_STATE_KEY | BOOT_STATE_SOFT_SWITCHED)) {
                boot_state = BOOT_STATE_KEY | BOOT_STATE_VERIFIED;
            } else {
                boot_state = BOOT_STATE_KEY | BOOT_STATE_NB_ERROR;
            }
            boot_transmit_status(0);
            return 0;
        } else if (databuf[0] == 0x03) {
            // If the program is verified, swap the banks, set state to
            // NORMAL, and reset
            if ((check_nonbooting()) && (boot_state == (BOOT_STATE_KEY | BOOT_STATE_VERIFIED))) {
                boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
                boot_bankswap();
            }
        }
    }
    return 0;
}

static int8_t boot_program_and_verify(uint8_t length) {
    // If the length is not doubleword-aligned, return 0 (error).
    if (length & 0x07) return -BOOT_STATUS_INVALID_ADDRESS;
    // If the address is not doubleword-aligned, return 0 (error).
    if (address & 0x07) return -BOOT_STATUS_INVALID_ADDRESS;
    // In case the target section overlaps with bootloader code, return 0.
    // An empty echo frame will be transmitted, indicating an error.
    if (address + length > 0x40000) return -BOOT_STATUS_INVALID_ADDRESS;
    if (check_nonbooting()) return -BOOT_STATUS_NONBOOTING_PROGRAM;
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
        if (FLASH->SR & 0xfffe) return -BOOT_STATUS_ERASE_ERROR;
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
        if (FLASH->SR & 0xfffe) return -BOOT_STATUS_PROG_ERROR;
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
    core_CAN_module_t *p_can = core_CAN_convert(CORE_BOOT_FDCAN);
    hfdcan = &(p_can->hfdcan);
    //core_CAN_add_filter(CORE_BOOT_FDCAN, 1, (CORE_BOOT_FDCAN_ID << 18), ((CORE_BOOT_FDCAN_ID+1) << 18)-1);
    boot_transmit_status(0);

    for (uint8_t i=0; i < 8; i++) page_erased[i] = 0;
    // Main programming loop
    int8_t ndata = 0;
    base_address = 0;
    address = 0;
    uint8_t bankmode = check_nonbooting();
    while (1) {
        ndata = boot_await_data();
        if (ndata) {
            if (bankmode) boot_transmit_status(BOOT_STATUS_NB_ERROR);
            else {
                ndata = boot_program_and_verify(ndata);
                if (ndata >= 0) {
                    boot_transmit_can(ndata, 1);
                    core_GPIO_toggle_heartbeat();
                }
                else boot_transmit_status(-ndata);
            }
        } else {
            boot_transmit_status(13);
        }
    }
}

/**
  * @brief  Reset the chip and enter the bootloader
  */
void core_boot_reset_and_enter() {
    if (check_nonbooting()) {
        if (boot_state == (BOOT_STATE_KEY | BOOT_STATE_SOFT_SWITCHED)) {
            boot_state = BOOT_STATE_KEY | BOOT_STATE_ENTER | BOOT_STATE_VERIFY;
        } else if ((boot_state & 0xffffff00) == BOOT_STATE_KEY) boot_state |= BOOT_STATE_NB_ERROR;
        // If the boot state key is not valid, then the boot state machine did
        // not run and the boot_state variable points to the wrong location. 
        // Return to the booting bank, which will see that the state is 
        // VERIFY_SOFT_SWITCH and raise an error.
    } else {
        if (boot_state == (BOOT_STATE_KEY | BOOT_STATE_NORMAL)) {
            boot_state = BOOT_STATE_KEY | BOOT_STATE_ENTER;
        } else boot_state |= BOOT_STATE_ERROR;
    }
    boot_reset();
}

/**
  * @brief  Check the boot state and enter the bootloader if necessary
  * @note   This function should be called after the FDCAN module is
  *         initialized.
  */
void core_boot_init() {
    // Make sure the device listens for its boot address
    core_CAN_add_filter(CORE_BOOT_FDCAN, 1, (CORE_BOOT_FDCAN_ID << 18), ((CORE_BOOT_FDCAN_ID+1) << 18)-1);
    //boot_transmit_status(21);
    //for (int i=0; i < 200000; i++);
    if (check_nonbooting()) {
        if ((boot_state & 0xffffff00) != BOOT_STATE_KEY) {
            // Boot state key is invalid. This will only happen if the boot 
            // state machine does not run and if the boot_state variable points
            // to the wrong location. Return to the booting bank, which will 
            // see that the state is VERIFY_SOFT_SWITCH and raise an error.
            boot_reset();
        } else if ((boot_state & 0x0000000f) != BOOT_STATE_SOFT_SWITCHED) {
            // Invalid state, likely because the boot state machine was not
            // run. Set the NB_ERROR flag and reset.
            boot_state |= BOOT_STATE_NB_ERROR;
            boot_reset();
        }
        // Soft switching can only occur if there are no errors. If the boot
        // state machine in the non-booting bank sees an error, a reset will be
        // triggered.
    } else {
        if (((boot_state & 0xffffff00) != BOOT_STATE_KEY) || (boot_state & BOOT_STATE_ERROR)) {
            // An error was reported or the boot state key is not valid, 
            // transmit an error message on CAN.
            boot_transmit_status(BOOT_STATUS_STATE_ERROR);
            boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
        }
    }

    if ((boot_state & 0xffffff00) == BOOT_STATE_KEY) {
        if (boot_state & BOOT_STATE_ENTER) {
            boot_state &= ~BOOT_STATE_ENTER;
            __disable_irq();
            boot();
        }
    }
    __enable_irq();

    if (!check_nonbooting()) {
        boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
    }
}
