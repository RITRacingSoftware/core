/**
  * @file   clock.c
  * @brief  Core clock library
  *
  * This core library component is used to initialize peripheral clocks.
  * Generally, it is only necessary to call core_clock_init() from user code. 
  * Other clock initialization functions are called by the respective core 
  * module init function.
  */

#include "clock.h"
#include "core_config.h"

#include <stdbool.h>

#include "stm32g4xx_hal.h"


/**
  * @brief  Set the clocks for ADC1 and ADC2 to SYSCLK and enable them
  */
void core_clock_ADC12_init() {
    __HAL_RCC_ADC12_CONFIG(RCC_ADC12CLKSOURCE_SYSCLK);
    __HAL_RCC_ADC12_CLK_ENABLE();
}

/**
  * @brief  Set the clocks for ADC3, ADC4, and ADC5 to SYSCLK and enable them
  */
void core_clock_ADC345_init() {
    __HAL_RCC_ADC345_CONFIG(RCC_ADC345CLKSOURCE_SYSCLK);
    __HAL_RCC_ADC12_CLK_ENABLE();
}

/**
  * @brief  Set FDCAN clock to PCLK1 and enable it.
  *
  * Initialize GPIO port clocks corresponding to CAN bus selected
  * @param  can FDCAN module to initialize
  */
void core_clock_FDCAN_init(FDCAN_GlobalTypeDef *can)
{
    // Initialize peripheral clocks
    __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PCLK1);
    if (can == FDCAN1) {
        core_clock_port_init(CORE_FDCAN1_RX_PORT);
        core_clock_port_init(CORE_FDCAN1_TX_PORT);
    } else if (can == FDCAN2) {
        core_clock_port_init(CORE_FDCAN2_RX_PORT);
        core_clock_port_init(CORE_FDCAN2_TX_PORT);
    } else if (can == FDCAN3) {
        core_clock_port_init(CORE_FDCAN3_RX_PORT);
        core_clock_port_init(CORE_FDCAN3_TX_PORT);
    }
    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
}

/**
  * @brief  Set a USART clock to PCLK1 and enable it
  * @param  usart USART module to initialize
  * @retval 1 if usart_num is a valid USART module
  * @retval 0 otherwise
  */
bool core_clock_USART_init(USART_TypeDef *usart) {
    if (usart == USART1) {
        __HAL_RCC_USART1_CONFIG(RCC_USART1CLKSOURCE_PCLK2);
        __HAL_RCC_USART1_CLK_ENABLE();
    }
    else if (usart == USART2) {
        __HAL_RCC_USART2_CONFIG(RCC_USART2CLKSOURCE_PCLK1);
        __HAL_RCC_USART2_CLK_ENABLE();
    }
    else if (usart == USART3) {
        __HAL_RCC_USART3_CONFIG(RCC_USART3CLKSOURCE_PCLK1);
        __HAL_RCC_USART3_CLK_ENABLE();
    }
    else if (usart == UART4) {
        __HAL_RCC_UART4_CONFIG(RCC_UART4CLKSOURCE_PCLK1);
        __HAL_RCC_UART4_CLK_ENABLE();
    }
    else if (usart == UART5) {
        __HAL_RCC_UART5_CONFIG(RCC_UART5CLKSOURCE_PCLK1);
        __HAL_RCC_UART5_CLK_ENABLE();
    }
    else return false;
    return true;
}

/**
  * @brief  Set an I2C clock to PCLK1 and enable it
  * @param  i2c  I2C module to initialize
  * @retval 1 if i2c_num is a valid I2C module
  * @retval 0 otherwise
  */
bool core_clock_I2C_init(I2C_TypeDef *i2c) {
    if (i2c == I2C1) {
        __HAL_RCC_I2C1_CONFIG(RCC_I2C1CLKSOURCE_PCLK1);
        __HAL_RCC_I2C1_CLK_ENABLE();
    }
    else if (i2c == I2C2) {
        __HAL_RCC_I2C2_CONFIG(RCC_I2C2CLKSOURCE_PCLK1);
        __HAL_RCC_I2C2_CLK_ENABLE();
    }
    else if (i2c == I2C3) {
        __HAL_RCC_I2C3_CONFIG(RCC_I2C3CLKSOURCE_PCLK1);
        __HAL_RCC_I2C3_CLK_ENABLE();
    }
    else if (i2c == I2C4) {
        __HAL_RCC_I2C4_CONFIG(RCC_I2C4CLKSOURCE_PCLK1);
        __HAL_RCC_I2C4_CLK_ENABLE();
    }
    else return false;
    return true;
}

/**
  * @brief  Enable the external 32.768kHz oscillator and set the RTC clock to it.
  * @retval 1
  */
bool core_clock_RTC_init() {
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_RTCAPB_CLK_ENABLE();
    __HAL_RCC_LSE_CONFIG(RCC_LSE_ON);
    __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSE);
    __HAL_RCC_RTC_ENABLE();
    return true;
}

/**
  * @brief Initializes port clock for selected port
  * @param port Port to initialize clock for
  */
void core_clock_port_init(GPIO_TypeDef *port) {
    if (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
}

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
uint8_t core_clock_generate_params(uint32_t src_freq, uint32_t target_freq, uint8_t *n, uint8_t *m, uint8_t *r) {
    // Determine the largest possible R divider setting for which the required
    // VCO output frequency would still be in range. If this is not possible,
    // return 0.
    if (target_freq > 170000) return 0;
    target_freq = target_freq * 2;
    *r = 344000 / target_freq;
    if (*r > 4) *r = 4;
    // Update target_freq with the frequency that would be found before the R
    // divider, that is, the desired frequency on the output of the VCO.
    target_freq *= *r;
    *r = *r * 2;
    if (target_freq < 96000) return 0;
    // Perform a binary search on the Stern-Brocot tree to find the fraction
    // closest to the ratio target_freq/src_freq. Initially, target_freq/src_freq
    // is guaranteed to be between lnum/lden and hnum/hden. Also, all of the fractions
    // generated by the algorithm are guaranteed to be in lowest form
    uint8_t lnum = target_freq / src_freq, lden = 1;
    uint8_t hnum = lnum + 1, hden = 1;
    uint8_t cnum, cden;
    uint32_t s, t;
    for (uint8_t i=0; i < 30; i++) {
        cnum = lnum + hnum;
        cden = lden + hden;
        // The fraction cnum/cden is guaranteed to be between lnum/lden and 
        // hnum/hden.
        s = cnum * src_freq;
        t = cden * target_freq;
        // If cnum exceeds the maximum possible value for the N divider,
        // or cden exceeds the maximum possible value for the M divider, stop
        // iterating and pick the pair (that is, either hnum and hden, or 
        // lnum and lden) that would produce a frequency closer to the desired
        // frequency.
        if ((cnum > 127) || (cden > 16)) {
            if (2*target_freq*lden*hden >= target_freq*(hnum*lden + hden*lnum)) {
                *n = hnum;
                *m = hden;
            } else {
                *n = lnum;
                *m = lden;
            }
            return 1;
        }
        if (s == t) {
            // If cnum/cden == target_freq/src_freq, then use N=cnum and M=cden
            *n = cnum;
            *m = cden;
            return 1;
        } else if (s > t) {
            // If cnum/cden > target_freq/src_freq, then repeat, this time using
            // cnum/cden as the upper bound
            hnum = cnum;
            hden = cden;
        } else if (s < t) {
            // If cnum/cden < target_freq/src_freq, then repeat, this time using
            // cnum/cden as the lower bound
            lnum = cnum;
            lden = cden;
        }
    }
    return 0;
}

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
bool core_clock_init() {
    HAL_Init();

    // Configure the main internal regulator output voltage
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Compute the values of the N, M, and R dividers
    uint8_t ndiv, mdiv, rdiv;
    uint32_t ext_freq;
#ifdef CORE_CLOCK_USE_HSE
    ext_freq = CORE_CLOCK_HSE_FREQ;
#else
    ext_freq = CORE_CLOCK_HSI_FREQ;
#endif
    uint32_t sysclk_freq = CORE_CLOCK_SYSCLK_FREQ;
    if (!core_clock_generate_params(ext_freq, sysclk_freq, &ndiv, &mdiv, &rdiv)) {
        return false;
    }

    // Initialize the RCC Oscillators
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
#ifdef CORE_CLOCK_USE_HSE
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#else
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    //RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
#endif
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLM = mdiv;
    RCC_OscInitStruct.PLL.PLLN = ndiv;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = rdiv;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        return false;
    }

    // Initialize the CPU, AHB and APB buses clocks
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
#ifdef CORE_CLOCK_USE_HSE
    RCC_ClkInitStruct.SYSCLKSource = (ext_freq == sysclk_freq ? RCC_SYSCLKSOURCE_HSE : RCC_SYSCLKSOURCE_PLLCLK);
#else
    RCC_ClkInitStruct.SYSCLKSource = (ext_freq == sysclk_freq ? RCC_SYSCLKSOURCE_HSI : RCC_SYSCLKSOURCE_PLLCLK);
#endif
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        return false;
    }

    return true;
}
