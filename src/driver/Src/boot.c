/**
  * @file   boot.c
  * @brief  Core bootloader
  * 
  * This core library component implements a bootloader that allows boards to
  * be programmed over CAN.
  *
  * ## Theory of operation
  * The STM32G473 has 512k of FLASH, which is split into two banks of 256k
  * each. The chip can be configured to boot either from the first or the
  * second bank by configuring the non-volatile `BFB2` bit in the option byte
  * registers. When the `BFB2` bit is set in the FLASH option byte register,
  * the boots from the second bank, otherwise, it boots from the first bank.
  * However, there is also an option to temporarily swap the banks and run
  * the code in the second bank even when booting from the first bank. This
  * allows the code in the non-booting bank to be verified before finalizing
  * the swap. If the verification fails, then the chip will fall back to the
  * working code in the booting bank.
  *
  * Programming a board takes place according to the following process:
  *  1. The programmer sends a command to the target board to enter the bootloader
  *  2. The programmer sends program data to the target board. The target board
  *     writes the program data to the non-booting bank
  *  3. After each block of data is written, the target board reads the block
  *     back so the programmer can verify it
  *  4. Once all of the data has been written, the programmer commands the
  *     target board to switch to the non-booting bank
  *  5. The target board board resets and performs a soft bank swap
  *  6. The programmer commands the target board to enter the bootloader in
  *     the non-booting bank
  *  7. The programmer sends a command to the bootloader in the non-booting
  *     bank to verify that FDCAN communication is working
  *  8. The programmer commands the target board to binalize the bank swap
  *  9. The target board updates the option byte, resets, and runs the new code
  *
  * The bootloader keeps track of its state across resets and between banks
  * using the `boot_state` variable, which is stored at the highest RAM
  * address (above the stack). This variable is not initialized when the chip
  * is reset, so its value is always preserved unless the chip is power cycled.
  * The highest 24 bits of `boot_state` are known as the boot key and must
  * always be set to `0xABCDEF`. If the boot key is incorrect, an error is
  * raised. This might occur if the `boot_state` variable is not correctly
  * configured.
  *
  * ## FDCAN format
  * The bootloader communicates with the programmer board using FDCAN with
  * extended IDs. The extra bits in the ID are used to communicate they type of
  * message and the address to be programmed (if required), so all 64 bytes in
  * the body of the message can be used for data.
  *
  * Each board has a unique board ID and master ID, so the master will respond
  * to several IDs, one for each device that can be programmed. The 29-bit 
  * extended board IDs have the following format:
  * <table class="doxtable RegisterTable">
  * <tr class="RegisterBitNumber">
  *   <td>31</td><td>30</td><td>29</td><td>28</td><td>27</td><td>26</td><td>25</td><td>24</td>
  *   <td>23</td><td>22</td><td>21</td><td>20</td><td>19</td><td>18</td><td>17</td><td>16</td>
  * </tr>
  * <tr class="RegisterFields">
  *   <td colspan=3></td>
  *   <td colspan=11>`ID[10:0]`</td>
  *   <td><span class=overlined>CTRL</span></td>
  *   <td>`RD/`<span class=overlined>WR</span></td>
  * </tr>
  * <tr class="RegisterBitNumber">
  *   <td>15</td><td>14</td><td>13</td><td>12</td><td>11</td><td>10</td><td>9</td><td>8</td>
  *   <td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td>
  * </tr>
  * <tr class="RegisterFields">
  *   <td>`PAD`</td>
  *   <td colspan=15>`ADDR[14:0]`</td>
  * </tr>
  * </table>
  *
  *  - `ID[10:0]`: slave ID, used for arbitration
  *  - <span class="overlined">CTRL</span>: 0 for a control frame, 1 for a data frame
  *  - `RD/`<span class="overlined">WR</span>: 1 to read from the slave, 0 to write to the slave
  *  - `PAD`: Set if the last doubleword in the frame is a padding doubleword
  *  - `ADDR[14:0]`: Doubleword address
  *
  * The master IDs have the following format:
  * <table class="doxtable RegisterTable">
  * <tr class="RegisterBitNumber">
  *   <td>31</td><td>30</td><td>29</td><td>28</td><td>27</td><td>26</td><td>25</td><td>24</td>
  *   <td>23</td><td>22</td><td>21</td><td>20</td><td>19</td><td>18</td><td>17</td><td>16</td>
  * </tr>
  * <tr class="RegisterFields">
  *   <td colspan=3></td>
  *   <td colspan=11>`ID[10:0]`</td>
  *   <td>1</td>
  *   <td>`DATA/`<span class=overlined>STAT</span></td>
  * </tr>
  * <tr class="RegisterBitNumber">
  *   <td>15</td><td>14</td><td>13</td><td>12</td><td>11</td><td>10</td><td>9</td><td>8</td>
  *   <td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td>
  * </tr>
  * <tr class="RegisterFields">
  *   <td>`PAD`</td>
  *   <td colspan=15>`ADDR[14:0]`</td>
  * </tr>
  * </table>
  *  - `ID[10:0]`: master ID, used for arbitration
  *  - `DATA/`<span class="overlined">STAT</span>: 1 if the frame contains data
  *    (echo or read), 0 if the frame contains a status message
  *  - `PAD`: Set if the last doubleword in the frame is a padding doubleword
  *  - `ADDR[15:0]`: Doubleword address
  *
  * Status messages transmitted from a board to the programmer are 64 bits long
  * and have the following format:
  * <table class="doxtable RegisterTable">
  * <tr class="RegisterBitNumber">
  *   <td>63</td><td>62</td><td>61</td><td>60</td><td>59</td><td>58</td><td>57</td><td>56</td>
  *   <td>55</td><td>54</td><td>53</td><td>52</td><td>51</td><td>50</td><td>49</td><td>48</td>
  * </tr>
  * <tr class="RegisterFields">
  *  <td colspan=8>`STATUS[7:0]`</td>
  *  <td colspan=6></td>
  *  <td>`BFB2`</td>
  *  <td>`MEMRMP`</td>
  * </tr>
  * <tr class="RegisterBitNumber">
  *   <td>47</td><td>46</td><td>45</td><td>44</td><td>43</td><td>42</td><td>41</td><td>40</td>
  *   <td>39</td><td>38</td><td>37</td><td>36</td><td>35</td><td>34</td><td>33</td><td>32</td>
  * </tr>
  * <tr class="RegisterFields">
  *   <td>`OPTVERR`</td>
  *   <td>`RDERR`</td>
  *   <td colspan=4></td>
  *   <td>`FASTERR`</td>
  *   <td>`MISSERR`</td>
  *   <td>`PGSERR`</td>
  *   <td>`SIZERR`</td>
  *   <td>`PGAERR`</td>
  *   <td>`WRPERR`</td>
  *   <td>`PROGERR`</td>
  *   <td></td>
  *   <td>`OPERR`</td>
  *   <td>`EOP`</td>
  * </tr>
  * <tr class="RegisterBitNumber">
  *   <td>31</td><td>30</td><td>29</td><td>28</td><td>27</td><td>26</td><td>25</td><td>24</td>
  *   <td>23</td><td>22</td><td>21</td><td>20</td><td>19</td><td>18</td><td>17</td><td>16</td>
  * </tr>
  * <tr class="RegisterFields">
  *   <td colspan=16>`BOOT_STATE_KEY[23:8]`</td>
  * </tr>
  * <tr class="RegisterBitNumber">
  *   <td>15</td><td>14</td><td>13</td><td>12</td><td>11</td><td>10</td><td>9</td><td>8</td>
  *   <td>7</td><td>6</td><td>5</td><td>4</td><td>3</td><td>2</td><td>1</td><td>0</td>
  * </tr>
  * <tr class="RegisterFields">
  *   <td colspan=8>`BOOT_STATE_KEY[7:0]`</td>
  *   <td>`ERROR`</td>
  *   <td>`NB_ERROR`</td>
  *   <td></td>
  *   <td>`ENTER`</td>
  *   <td>`VERIFIED`</td>
  *   <td>`SOFT_SWITCHED`</td>
  *   <td>`VERIFY_SOFT_SWITCH`</td>
  *   <td>`VERIFY`</td>
  * </tr>
  * </table>
  *  - `STATUS[7:0]`: Status code
  *    <table>
  *    <tr><td>0</td><td>No error</td></tr>
  *    <tr><td>1</td><td>Address out of range</td></tr>
  *    <tr><td>2</td><td>Error while erasing</td></tr>
  *    <tr><td>3</td><td>Error while programming</td></tr>
  *    <tr><td>4</td><td>Boot state error</td></tr>
  *    <tr><td>5</td><td>Write from non-booting bank</td></tr>
  *    </table>
  *  - `BFB2`: `BFB2` bit from the option byte register. Indicates which bank
  *    the chip will boot from
  *  - `MEMRMP`: `MEMRMP` bit from the memory remap register. Indicates which
  *    bank is currently running
  *  - Next two bytes contain the lowest two bytes of `FLASH_SR`
  *  - Next three bytes contain the boot key, which should be 0xABCDEF
  *  - Bits 7 through 0 contain the boot state.
  *  - `ERROR`: Indicates a state error occurred in the booting bank
  *  - `NB_ERROR`: Indicates a state error occurred in the non-booting bank
  *  - `ENTER`: Indicates that the program should enter the bootloader after
  *    the next reset
  *  - `VERIFIED`: Indicates that the program in the non-booting bank has been
  *    verified. If this bit is set, the banks can be swapped
  *  - `SOFT_SWITCHED`: Indicates that the soft switching succeeded
  *  - `VERIFY_SOFT_SWITCH`: Indicates that the signal to soft switch has been
  *    processed by the boot state machine in the booting bank
  *  - `VERIFY`: Indicates that the chip should soft switch after the next reset
  *
  *
  * ## Components
  * The bootloader consists of four main components: the startup script, the
  * boot state machine, the core_boot_init() function, and an entry point.
  *
  * ### Startup script
  * For the bootloader to work, the default startup script must be replaced by
  * the startup script startup_stm32g473xx.s. The startup script defines the
  * interrupt handlers for the program, including the reset handler. By
  * default, the reset handler initializes the stack and jumps to the
  * application code. The modified startup script required for the bootloader
  * also runs the boot state machine before performing any additional
  * initialization. 
  *
  * ### Boot state machine
  * The boot state machine runs before any other code and updates the boot
  * state or soft swaps as necessary. The BSM first checks the nature of the
  * reset. If the reset was caused externally (by pulling the nRST pin low),
  * then the state is reset to NORMAL and the BSM continues to the application
  * code. 
  *
  * The behavior of the BSM depends on whether it is running in the booting or
  * non-booting banks. In the booting bank, the BSM will change the state to 
  * VERIFY_SOFT_SWAP and soft-swap if the state is VERIFY and continue to 
  * application code otherwise.
  *
  * In the non-booting bank, the BSM will change the state to SOFT_SWITCHED
  * if the state is VERIFY_SOFT_SWAP. Otherwise, the BSM will set the NB_ERROR
  * bit and reset. This causes the chip to return to the booting bank.
  *
  * \image html boot_state_machine.svg
  *
  * ### core_boot_init()
  * The core_boot_init() function is called by application code after the
  * clocks, GPIO, and FDCAN modules needed for the bootloader have been
  * initialized. This function initializes the RX filter for the board's
  * bootloader ID. The function also checks the boot state and enters the
  * bootloader if necessary. See the state diagram above for details.
  *
  * ### Entry point
  * The software enters the bootloader by calling core_boot_reset_and_enter().
  * The FDCAN RX interrupt hander in can.c will automatically call this
  * function if it receives a packet addressed to its bootloader FDCAN ID
  * containing the command to enter the bootloader
  */
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

#define BOOT_OPCODE_RESET       0x00
#define BOOT_OPCODE_SOFTSWAP    0x01
#define BOOT_OPCODE_VERIFY      0x02
#define BOOT_OPCODE_HARDSWAP    0x03

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
    tx_id = (CORE_BOOT_FDCAN_MASTER_ID << 18) | (1<<17) | address;
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
    databuf[1] = ((FLASH->OPTR >> 19) & 2) | ((SYSCFG->MEMRMP >> 8) & 1);
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
    if ((id >> 18) != CORE_BOOT_FDCAN_ID) return 0;
    if (id & (1<<17)) {
        // Data frame
        if (id & (1<<15)) length -= 8;
        address = (id & 0x7fff) << 3;
        return length;
    } else {
        // Control frame
        if (databuf[0] == BOOT_OPCODE_RESET) {
            boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
            boot_reset();
        } else if (databuf[0] == BOOT_OPCODE_SOFTSWAP) {
            // Failsafe bank swap
            boot_state = BOOT_STATE_KEY | BOOT_STATE_VERIFY;
            boot_reset();
        } else if (databuf[0] == BOOT_OPCODE_VERIFY) {
            // Verification command
            if (boot_state == (BOOT_STATE_KEY | BOOT_STATE_SOFT_SWITCHED)) {
                boot_state = BOOT_STATE_KEY | BOOT_STATE_VERIFIED;
            } else {
                boot_state = BOOT_STATE_KEY | BOOT_STATE_NB_ERROR;
            }
            boot_transmit_status(0);
            return 0;
        } else if (databuf[0] == BOOT_OPCODE_HARDSWAP) {
            // If the program is verified, swap the banks, set state to
            // NORMAL, and reset
            if ((check_nonbooting()) && (boot_state == (BOOT_STATE_KEY | BOOT_STATE_VERIFIED))) {
                boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
                boot_transmit_status(0);
                boot_bankswap();
            } else {
                boot_transmit_status(4);
                boot_state = BOOT_STATE_KEY | BOOT_STATE_NORMAL;
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
    address = 0;
    uint8_t bankmode = check_nonbooting();
    while (1) {
        ndata = boot_await_data();
        // Read data
        if (id & (1<<16)) {
            ndata = databuf[0];
            uint8_t bank = databuf[1];
            for (uint8_t i=0; i < ndata; i++) {
                databuf[i] = *(uint8_t*)((bank ? ALTBANK_BASE : 0) + address+i);
            }
            boot_transmit_can(ndata, 1);
        } 
        // Write data only if the ID is equal to the boot ID (rather than the
        // broadcast ID)
        else if (ndata && (((id >> 18) & 0x7ff) == CORE_BOOT_FDCAN_ID)) {
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
  * @brief  Initialize the FDCAN filters, check the boot state, and enter
  *         the bootloader if necessary. If the state is not valid, an error
  *         message will be transmitted
  * @note   This function should be called after the FDCAN module is
  *         initialized.
  */
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
