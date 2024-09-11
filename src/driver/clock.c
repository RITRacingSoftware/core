#include <stdbool.h>
#include <stdint.h>

#include "clock.h"

#include "stm32g4xx_hal.h"

bool core_clock_ADC12_init() {
    // Initialize peripheral clocks
    __HAL_RCC_ADC12_CONFIG(RCC_ADC12CLKSOURCE_SYSCLK);
    return true;
}

bool core_clock_FDCAN_init() {
    // Code copied from HAL source
    __HAL_RCC_FDCAN_CONFIG(RCC_FDCANCLKSOURCE_PCLK1);
    __HAL_RCC_FDCAN_CLK_ENABLE();
    return true;
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
  * @retval 0 if the desired setting is outside of the frequency range of the
  *         VCO, 1 otherwise
  */
uint8_t core_clock_generate_params(uint32_t src_freq, uint32_t target_freq, uint8_t *n, uint8_t *m, uint8_t *r) {
    if (target_freq > 170000) return 0;
    target_freq = target_freq * 2;
    *r = 344000 / target_freq;
    if (*r > 4) *r = 4;
    target_freq *= *r;
    *r = *r * 2;
    if (target_freq < 96000) return 0;
    uint8_t lnum = target_freq / src_freq, lden = 1;
    uint8_t hnum = lnum + 1, hden = 1;
    uint8_t cnum, cden;
    uint32_t s, t;
    for (uint8_t i=0; i < 30; i++) {
        cnum = lnum + hnum;
        cden = lden + hden;
        s = cnum * src_freq;
        t = cden * target_freq;
        if ((cnum > 127) || (cden > 16)) {
            // No exact match found, pick closer frequency
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
            // Exact match
            *n = cnum;
            *m = cden;
            return 1;
        } else if (s > t) {
            hnum = cnum;
            hden = cden;
        } else if (s < t) {
            lnum = cnum;
            lden = cden;
        }
    }
    return 0;
}

/**
  * @brief  Initialize the STM32G4's core clocks
  * @param  use_ext  Boolean that indicates if an external oscillator should
  *         be used.
  * @param  ext_freq  Frequency of the external oscillator in kilohertz
  * @param  sysclk_freq  Desired frequency of the SYSCLK in kilohertz
  * @retval 1 on success, 0 otherwise
  */
bool core_clock_init(bool use_ext, uint32_t ext_freq, uint32_t sysclk_freq) {
    HAL_Init();

    // Configure the main internal regulator output voltage
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

    // Compute the values of the N, M, and R dividers
    uint8_t ndiv, mdiv, rdiv;
    ext_freq = (use_ext ? ext_freq : 16000);
    if (!core_clock_generate_params(ext_freq, sysclk_freq, &ndiv, &mdiv, &rdiv)) {
        return false;
    }
    

    // Initialize the RCC Oscillators
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = (use_ext ? RCC_HSE_ON : RCC_HSE_OFF);
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = (use_ext ? RCC_PLLSOURCE_HSE : RCC_PLLSOURCE_HSI);
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
    RCC_ClkInitStruct.SYSCLKSource = (ext_freq == sysclk_freq ? (use_ext ? RCC_SYSCLKSOURCE_HSE : RCC_SYSCLKSOURCE_HSI) : RCC_SYSCLKSOURCE_PLLCLK);
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
        return false;
    }


    // Initialize peripheral clocks
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    return true;
}
