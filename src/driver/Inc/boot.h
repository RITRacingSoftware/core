/**
  * @file   boot.h
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
  * ### Communication from the programmer to the target boards
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
  * To decrease the number of bits required to transmit the address, the 
  * address is left-shifted by 3 bits. This means that the address will be 
  * aligned on an 8-byte boundary. This is required for writing, since all
  * writes to the FLASH memory must be doubleword-aligned.
  *
  * Messages sent with <span class="overlined">CTRL</span> set to 0 can have
  * the following contents:
  * | Data                      | Operation |
  * |---------------------------| --------- |
  * | `55 55 55 55 55 55 55 55` | Enter bootloader |
  * | `00`                      | Reset chip |
  * | `01`                      | Soft swap (jump to non-booting bank) |
  * | `02`                      | Verify |
  * | `03`                      | Hard swap (only possible after verification with `02`) |
  * | `04`                      | Switch to external programming mode (only possible in booting bank) |
  * | `05`                      | Leave external programming mode (only possible in booting bank) |
  * Note that all commands except "enter bootloader" require the chip to
  * have been booted. 
  *
  * Messages sent with <span class="overlined">CTRL</span> set to 1 can either 
  * be read or write requests. For a read request, the first data byte is the 
  * length and the second byte indicates the bank. Note that while the address
  * must be 8-byte aligned, the length of the read can have any value. If the 
  * bank byte is non-zero, then the target board will read from the bank that 
  * is not currently running. 
  *
  * For a write request, all of the data bytes contain data to be written. The
  * number of data bytes must be a multiple of 8. Since FDCAN only supports 
  * certain packet lengths, if the packet contains fewer doublewords than would
  * fit in the packet, the `PAD` byte is set. For example, to transmit 56 bytes
  * of data (7 doublewords), one would need a packet length of 64 (DLC set to
  * 15) and set the `PAD` bit to indicate that the last doubleword should be
  * ignored.
  *
  *
  * ### Communication from the target boards to the programmer
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
  *   <td>0</td><td>1</td><td>2</td><td>3</td><td>4</td><td>5</td><td>6</td><td>7</td>
  *   <td>8</td><td>9</td><td>10</td><td>11</td><td>12</td><td>13</td><td>14</td><td>15</td>
  * </tr>
  * <tr class="RegisterFields">
  *  <td colspan=8>`STATUS[7:0]`</td>
  *  <td>`MEMRMP`</td>
  *  <td>`BFB2`</td>
  *  <td colspan=6></td>
  * </tr>
  * <tr class="RegisterBitNumber">
  *   <td>16</td><td>17</td><td>18</td><td>19</td><td>20</td><td>21</td><td>22</td><td>23</td>
  *   <td>24</td><td>25</td><td>26</td><td>27</td><td>28</td><td>29</td><td>30</td><td>31</td>
  * </tr>
  * <tr class="RegisterFields">
  *   <td>`EOP`</td>
  *   <td>`OPERR`</td>
  *   <td></td>
  *   <td>`PROGERR`</td>
  *   <td>`WRPERR`</td>
  *   <td>`PGAERR`</td>
  *   <td>`SIZERR`</td>
  *   <td>`PGSERR`</td>
  *   <td>`MISSERR`</td>
  *   <td>`FASTERR`</td>
  *   <td colspan=4></td>
  *   <td>`RDERR`</td>
  *   <td>`OPTVERR`</td>
  * </tr>
  * <tr class="RegisterBitNumber">
  *   <td>32</td><td>33</td><td>34</td><td>35</td><td>36</td><td>37</td><td>38</td><td>39</td>
  *   <td>40</td><td>41</td><td>42</td><td>43</td><td>44</td><td>45</td><td>46</td><td>47</td>
  * </tr>
  * <tr class="RegisterFields">
  *   <td>`VERIFY`</td>
  *   <td>`VERIFY_SOFT_SWITCH`</td>
  *   <td>`SOFT_SWITCHED`</td>
  *   <td>`VERIFIED`</td>
  *   <td>`ENTER`</td>
  *   <td></td>
  *   <td>`NB_ERROR`</td>
  *   <td>`ERROR`</td>
  *   <td colspan=8>`BOOT_STATE_KEY[7:0]`</td>
  * </tr>
  * <tr class="RegisterBitNumber">
  *   <td>48</td><td>49</td><td>50</td><td>51</td><td>52</td><td>53</td><td>54</td><td>55</td>
  *   <td>56</td><td>57</td><td>58</td><td>59</td><td>60</td><td>61</td><td>62</td><td>63</td>
  * </tr>
  * <tr class="RegisterFields">
  *   <td colspan=16>`BOOT_STATE_KEY[23:8]`</td>
  * </tr>
  * </table>
  *  - `STATUS[7:0]`: Status code
  *    <table>
  *    <tr><td>0</td><td>Status</td><td>No error</td></tr>
  *    <tr><td>1</td><td>Error</td><td>Address out of range</td></tr>
  *    <tr><td>2</td><td>Error</td><td>Error while erasing</td></tr>
  *    <tr><td>3</td><td>Error</td><td>Error while programming</td></tr>
  *    <tr><td>4</td><td>Error</td><td>Boot state error</td></tr>
  *    <tr><td>5</td><td>Error</td><td>Write from non-booting bank</td></tr>
  *    <tr><td>6</td><td>Status</td><td>Board is already booted (sent when bootloader received opcode 0x55)</td></tr>
  *    <tr><td>7</td><td>Status</td><td>BSM did not run</td></tr>
  *    <tr><td>8</td><td>Status</td><td>Soft swap successful (sent when 
  *         core_boot_init() runs in the non-booting bank)</td></tr>
  *    <tr><td>9</td><td>Status</td><td>Sent when core_boot_init() runs in the
  *         booting bank</td></tr>
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

#ifndef CORE_BOOT_H
#define CORE_BOOT_H

#include <stdint.h>
#include "core_config.h"

/**
  * @brief  Initialize the FDCAN filters, check the boot state, and enter
  *         the bootloader if necessary. If the state is not valid, an error
  *         message will be transmitted
  * @note   This function should be called after the FDCAN module is
  *         initialized.
  */
void core_boot_init();

/**
  * @brief  Reset the chip and enter the bootloader
  */
void core_boot_reset_and_enter();

#if defined(CORE_BOOT_EXTERNAL) && (CORE_BOOT_EXTERNAL == 1)
extern void core_boot_external_read(uint8_t *ptr, uint32_t address, uint32_t length);
extern void core_boot_external_write(uint8_t *ptr, uint32_t address, uint32_t length);
extern void core_boot_external_enter();
extern void core_boot_external_exit();
#endif

#endif
