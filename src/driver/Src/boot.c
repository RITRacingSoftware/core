#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "usart.h"
#include "boot.h"
#include "gpio.h"
#include "clock.h"
#include "can.h"
#include "error_handler.h"
#include "core_config.h"
#include "stm32g4xx_hal.h"

#define BOOTSTATE __attribute__ ((section (".bootstate")))
#define BOOTPROGNAME __attribute__ ((section (".progname"))) __attribute__ ((__used__))

#define ALTBANK_BASE 0x08040000

const char BOOTPROGNAME progname[32] = PROGRAM_NAME_STRING;

uint32_t BOOTSTATE boot_state;
// Stores the opcodes used for soft bank switching. Soft bank switching must be
// performed from RAM, since the code in the other bank may continue from a
// different address.
uint32_t boot_toggle[8];
uint32_t boot_reset_state;

static uint32_t page_erased[4];
static uint32_t address;
static uint32_t id;
static uint8_t databuf[100];
static uint8_t extmode;

static FDCAN_HandleTypeDef *hfdcan;


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
#define BOOT_STATUS_ALREADY_BOOTED  0x06
#define BOOT_STATUS_NO_BSM      0x07
#define BOOT_STATUS_SOFTSWAP_SUCCESS    0x08
#define BOOT_STATUS_MAINBANK    0x09

#define BOOT_OPCODE_RESET       0x00
#define BOOT_OPCODE_SOFTSWAP    0x01
#define BOOT_OPCODE_VERIFY      0x02
#define BOOT_OPCODE_HARDSWAP    0x03
#define BOOT_OPCODE_EXTERNAL    0x04
#define BOOT_OPCODE_INTERNAL    0x05

/**
  * @brief  Soft bank swap
  *
  * This function is defined in startup_stm32g473xx.s
  */
extern void boot_soft_toggle();

/**
  * @brief  Reset the chip
  */
void boot_reset() {
    // Clear reset flags from previous reset
    RCC->CSR |= 0x00800000;
    SCB->AIRCR  = (SCB->AIRCR & (0x700)) | 0x05fa0004;
    __DSB();
}

/**
  * @brief  Check if the program currently running is running from the
  *         non-booting bank
  * @retval 0 The program currently running is in the booting bank
  * @retval 1 The program currently running is in the non-booting bank
  */
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

/**
  * @brief  Change the BFB2 bit in the option bytes
  */
static void boot_bankswap() {
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

/**
  * @brief  Transmit an FDCAN message in the bootloader FDCAN format
  * @param  length Length of the data to be transmitted
  * @param  is_data 1 for data, 0 for status messages
  */
static void boot_transmit_can(uint8_t length, bool is_data) {
    uint32_t tx_id;
    tx_id = (CORE_BOOT_FDCAN_MASTER_ID << 18) | (1<<17) | (address & 0x7fff);
    if (is_data) tx_id |= (1<<16);
    if (length > 64) length = 64;
    if ((length >= 32) && (length & 8)) {
        length += 8;
        tx_id |= (1<<15);
    }
    core_CAN_send_fd_message(CORE_BOOT_FDCAN, tx_id, length, databuf);
    while ((CORE_BOOT_FDCAN->PSR & 0x18) == 0x18);
}

/**
  * @brief  Transmit a bootloader status message over FDCAN
  * @param  code Bootloader status code
  */
static void boot_transmit_status(uint8_t code) {
    address = 0;
    databuf[0] = code;
    // bit 0: remap status (which bank the code runs on)
    // bit 1: BFB2 (which bank the chip boots from)
    // bit 7: external mode
    databuf[1] = (extmode << 7) | ((FLASH->OPTR >> 19) & 2) | ((SYSCFG->MEMRMP >> 8) & 1);
    databuf[2] = (FLASH->SR) & 0xff;
    databuf[3] = (FLASH->SR >> 8) & 0xff;
    memcpy(databuf+4, &boot_state, 4);
    boot_transmit_can(8, 0);
}

/**
  * @brief  Wait for data to come in over FDCAN
  * @return Length of the data received, or 0 if the message is a control message
  */
static uint8_t boot_await_data() {
    while (1) {
        while (!(CORE_BOOT_FDCAN->RXF0S & FDCAN_RXF0S_F0FL));
        // Data received over CAN
        FDCAN_RxHeaderTypeDef head;
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &head, databuf);
        id = head.Identifier;
        uint32_t length = core_CAN_dlc_lookup[head.DataLength];
        // Reset is the only opcode that can be executed using the broadcast address
        if ((!(id & (1<<17))) && (databuf[0] == BOOT_OPCODE_RESET) && (((id >> 18) == CORE_BOOT_FDCAN_ID) || ((id >> 18) == CORE_BOOT_FDCAN_BROADCAST_ID))) {
                boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
                boot_reset();
        }
        if ((head.IdType != FDCAN_EXTENDED_ID) || ((id >> 18) != CORE_BOOT_FDCAN_ID)) continue;
        //__BKPT(1);

        if (id & (1<<17)) {
            // Data frame
            if (id & (1<<15)) length -= 8;
            address = (address & 0xffff8000) | (id & 0x7fff);
            return length;
        } else {
            // Control frame
            switch (databuf[0]) {
                case BOOT_OPCODE_RESET:
                    boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
                    boot_reset();
                    break;
                case BOOT_OPCODE_SOFTSWAP:
                    // Failsafe bank swap
                    boot_state = BOOT_STATE_KEY | BOOT_STATE_VERIFY;
                    boot_reset();
                    break;
                case BOOT_OPCODE_VERIFY:
                    // Verification command
                    if (boot_state == (BOOT_STATE_KEY | BOOT_STATE_SOFT_SWITCHED)) {
                        boot_state = BOOT_STATE_KEY | BOOT_STATE_VERIFIED;
                    } else {
                        boot_state = BOOT_STATE_KEY | BOOT_STATE_NB_ERROR;
                    }
                    boot_transmit_status(0);
                    return 0;
                case BOOT_OPCODE_HARDSWAP:
                    // If the program is verified, swap the banks, set state to
                    // NORMAL, and reset
                    if ((check_nonbooting()) && (boot_state == (BOOT_STATE_KEY | BOOT_STATE_VERIFIED))) {
                        boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
                        boot_transmit_status(0);
                        boot_bankswap();
                    } else {
                        boot_transmit_status(BOOT_STATUS_STATE_ERROR);
                        boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
                    }
                    break;
#if defined(CORE_BOOT_EXTERNAL) && (CORE_BOOT_EXTERNAL == 1)
                case BOOT_OPCODE_EXTERNAL:
                    extmode = 1;
                    core_boot_external_enter();
                    boot_transmit_status(0);
                    break;
                case BOOT_OPCODE_INTERNAL:
                    extmode = 0;
                    core_boot_external_exit();
                    boot_transmit_status(0);
                    break;
#endif
                case 0x55:
                    boot_transmit_status(BOOT_STATUS_ALREADY_BOOTED);
                    break;
                default:
                    break;
            }
        }
        return 0;
    }
}

static int8_t boot_program_and_verify(uint8_t length) {
    // If the length is not doubleword-aligned, return 0 (error).
    if (length & 0x07) return -BOOT_STATUS_INVALID_ADDRESS;
    // Unlock the flash
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    uint8_t page;
    if (FLASH->OPTR & FLASH_OPTR_DBANK) page = address >> 8;
    else page = (address >> 9) & 0x7f;
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
        *(uint32_t*)(ALTBANK_BASE + (address<<3)+i) = (uint32_t)temp;
        __ISB();
        *(uint32_t*)(ALTBANK_BASE + (address<<3)+i+4) = (uint32_t)(temp >> 32);
        while (FLASH->SR & FLASH_SR_BSY);
        // Return 0 if an error occurred
        if (FLASH->SR & 0xfffe) return -BOOT_STATUS_PROG_ERROR;
    }
    FLASH->CR = FLASH_CR_LOCK;
    for (uint8_t i=0; i < length; i++) {
        databuf[i] = *(uint8_t*)(ALTBANK_BASE + (address<<3)+i);
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
    address = 0;
    uint8_t bankmode = check_nonbooting();
    while (1) {
        ndata = boot_await_data();
        // Read data
        if (id & (1<<16)) {
            ndata = databuf[0];
#if defined(CORE_BOOT_EXTERNAL) && (CORE_BOOT_EXTERNAL == 1)
            if (extmode) {
                address = (address & 0x7fff) | (databuf[1] << 15) | (databuf[2] << 23);
                core_boot_external_read(databuf, address, databuf[0]);
            } else 
#endif
            {
                uint8_t bank = databuf[1];
                for (uint8_t i=0; i < ndata; i++) {
                    databuf[i] = *(uint8_t*)((bank ? ALTBANK_BASE : 0x08000000) + (address<<3)+i);
                }
            }
            if (ndata) boot_transmit_can(ndata, 1);
        } 
        // Write data only if the ID is equal to the boot ID (rather than the
        // broadcast ID)
        else if (ndata && (((id >> 18) & 0x7ff) == CORE_BOOT_FDCAN_ID)) {
#if defined(CORE_BOOT_EXTERNAL) && (CORE_BOOT_EXTERNAL == 1)
            if (extmode) {
                core_boot_external_write(databuf, address, ndata);
                memset(databuf, 0, 64);
                core_boot_external_read(databuf, address, ndata);
                boot_transmit_can(ndata, 1);
            }
            else 
#endif
            if (bankmode) boot_transmit_status(BOOT_STATUS_NB_ERROR);
            else {
                ndata = boot_program_and_verify(ndata);
                if (ndata >= 0) {
                    boot_transmit_can(ndata, 1);
                    core_GPIO_toggle_heartbeat();
                }
                else boot_transmit_status(-ndata);
            }
        }
    }
}

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

void core_boot_init() {
    // Make sure the device listens for its boot address
    core_CAN_add_filter(CORE_BOOT_FDCAN, 1, (CORE_BOOT_FDCAN_ID << 18), ((CORE_BOOT_FDCAN_ID+1) << 18)-1);
    core_CAN_add_filter(CORE_BOOT_FDCAN, 1, (0x7ff << 18), 0x1fffffff);
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
            // run. Set the NB_ERROR flag and reset. This error case should
            // not occur, since soft switching requires the BSM to run
            boot_state |= BOOT_STATE_NB_ERROR;
            boot_reset();
        }
        // Soft switching can only occur if there are no errors. If the boot
        // state machine in the non-booting bank sees an error, a reset will be
        // triggered.
        boot_transmit_status(BOOT_STATUS_SOFTSWAP_SUCCESS);
    } else {
        boot_transmit_status(BOOT_STATUS_MAINBANK);
        if (((boot_state & 0xffffff00) != BOOT_STATE_KEY) || (boot_state & BOOT_STATE_ERROR)) {
            // An error was reported or the boot state key is not valid, 
            // transmit an error message on CAN.
            boot_transmit_status(BOOT_STATUS_STATE_ERROR);
            boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
        } else if ((boot_state & 0xff) == BOOT_STATE_VERIFY) {
            boot_transmit_status(BOOT_STATUS_NO_BSM);
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
