/**
  * @file   clock.h
  * @brief  Core clock library
  *
  * This core library component is used to initialize peripheral clocks.
  * Generally, it is only necessary to call core_clock_init() from user code. 
  * Other clock initialization functions are called by the respective core 
  * module init function.
  */

#pragma once

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/**
  * @brief  Set the clocks for ADC1 and ADC2 to SYSCLK and enable them
  */
void core_clock_ADC12_init();

/**
  * @brief  Set the clocks for ADC3, ADC4, and ADC5 to SYSCLK and enable them
  */
void core_clock_ADC345_init();

/**
  * @brief  Set FDCAN clock to PCLK1 and enable it.
  *
  * Initialize GPIO port clocks corresponding to CAN bus selected
  * @param  can FDCAN module to initialize
  */
void core_clock_FDCAN_init(FDCAN_GlobalTypeDef *can);

/**
  * @brief  Set a USART clock to PCLK1 and enable it
  * @param  usart USART module to initialize
  * @retval 1 if usart_num is a valid USART module
  * @retval 0 otherwise
  */
bool core_clock_USART_init(USART_TypeDef *usart);

/**
  * @brief  Enable a timer clock
  * @param  timer Timer to initialize (TIM2 through TIM7)
  * @retval 1 if timer is a supported timer module
  * @retval 0 otherwise
  */
bool core_clock_timer_init(TIM_TypeDef *timer);

/**
  * @brief  Set an I2C clock to PCLK1 and enable it
  * @param  i2c  I2C module to initialize
  * @retval 1 if i2c_num is a valid I2C module
  * @retval 0 otherwise
  */
bool core_clock_I2C_init(I2C_TypeDef *i2c);

/**
  * @brief  Enable the external 32.768kHz oscillator and set the RTC clock to it.
  * @retval 1
  */
bool core_clock_RTC_init();

/**
  * @brief Initializes port clock for selected port
  * @param port Port to initialize clock for
  */
void core_clock_port_init(GPIO_TypeDef *port);

/**
  * @brief  Generate the settings for the N, M, and R dividers from the clock
  *         source frequency and the desired output frequency
  * @param  src_freq  Frequency of the clock source of the PLL, in kilohertz
  * @param  target_freq  Desired frequency of the R output of the PLL, in kilohertz
  * @param  n  Location where the setting of the N divider is stored
  * @param  m  Location where the setting of the M divider is stored
  * @param  r  Location where the setting of the R divider is stored
  * @note   The algorithm used here will try to maximize the frequency
  *         of the VCO output. Thus, in certain cases, the algorithm will
  *         not find an exact setting, even if one exists.
  * @retval 0 if the desired setting is outside of the frequency range of the VCO
  * @retval 1 otherwise
  */
uint8_t core_clock_generate_params(uint32_t src_freq, uint32_t target_freq, uint8_t *n, uint8_t *m, uint8_t *r);

/**
  * @brief  Initialize the STM32G4's core clocks
  *
  * The SYSCLK frequency will be set to CORE_CLOCK_SYSCLK_FREQ, which is given
  * in kilohertz and defined in `core_config.h`. This function is generally
  * called immediately after HAL_Init() in the user code.
  *
  * @retval 1 on success
  * @retval 0 otherwise
  */
bool core_clock_init();
