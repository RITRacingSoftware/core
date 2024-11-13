/**
  * @file   adc.c
  * @brief  Core ADC library
  *
  * This core library component is used to initialize ADCs and read from analog
  * inputs. 
  *
  * ## Initialization
  * Before initializing any pins, the user code must initialize one or more ADC
  * modules. 
  * 
  * To initialize a pin, the user code calls core_ADC_setup_pin(). This
  * function takes the port and pin number as well as a third argument
  * specifying whether the analog signal should be routed through an internal
  * op amp follower circuit. This will improve the accuracy of the measurement
  * when the analog input is fed from a high-impedance source
  *
  * If core_ADC_setup_pin() returns 0, then the desired pin cannot be connected
  * to any of the ADCs that are currently initialized. It may be necessary to
  * initialize an additional ADC module or change the pin the analog input is
  * connected to.
  *
  * ## Reading
  * To read from an analog input, the user code calls core_ADC_read_channel().
  * The result is stored in a pointer passed as an argument, and the return
  * value specifies whether the conversion was successful or not.
  */

#include "adc.h"
#include "core_config.h"

#include <stdio.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_adc.h>
#include <string.h>

#include "gpio.h"
#include "clock.h"
#include "error_handler.h"

static ADC_HandleTypeDef adc1 = {0};
static ADC_HandleTypeDef adc2 = {0};
static ADC_HandleTypeDef adc3 = {0};
static ADC_HandleTypeDef adc4 = {0};
static ADC_HandleTypeDef adc5 = {0};


static core_ADC_def_t adc_defs[] = {
{GPIOA, GPIO_PIN_0, CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_ADC2, {1, 1, 0, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOA, GPIO_PIN_1, CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_ADC2 | CORE_ADC_ALLOWED_OPAMP1 | CORE_ADC_ALLOWED_OPAMP3, {2, 2, 0, 0, 0}, 32, NULL, NULL, 0, 0},
{GPIOA, GPIO_PIN_2, CORE_ADC_ALLOWED_ADC1, {3, 0, 0, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOA, GPIO_PIN_3, CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_OPAMP1, {4, 0, 0, 0, 0}, 1, NULL, NULL, 0, 0},
{GPIOA, GPIO_PIN_4, CORE_ADC_ALLOWED_ADC2, {0, 17, 0, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOA, GPIO_PIN_5, CORE_ADC_ALLOWED_ADC2, {0, 13, 0, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOA, GPIO_PIN_6, CORE_ADC_ALLOWED_ADC2, {0, 3, 0, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOA, GPIO_PIN_7, CORE_ADC_ALLOWED_ADC2 | CORE_ADC_ALLOWED_OPAMP1 | CORE_ADC_ALLOWED_OPAMP2, {0, 4, 0, 0, 0}, 2, NULL, NULL, 0, 0},
{GPIOA, GPIO_PIN_8, CORE_ADC_ALLOWED_ADC5, {0, 0, 0, 0, 1}, 0, NULL, NULL, 0, 0},
{GPIOA, GPIO_PIN_9, CORE_ADC_ALLOWED_ADC5, {0, 0, 0, 0, 2}, 0, NULL, NULL, 0, 0},
{GPIOB, GPIO_PIN_0, CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_ADC3 | CORE_ADC_ALLOWED_OPAMP2 | CORE_ADC_ALLOWED_OPAMP3, {15, 0, 12, 0, 0}, 8, NULL, NULL, 0, 0},
{GPIOB, GPIO_PIN_1, CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_ADC3, {12, 0, 1, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOB, GPIO_PIN_2, CORE_ADC_ALLOWED_ADC2, {0, 12, 0, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOB, GPIO_PIN_11, CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_ADC2 | CORE_ADC_ALLOWED_OPAMP4, {14, 14, 0, 0, 0}, 128, NULL, NULL, 0, 0},
{GPIOB, GPIO_PIN_12, CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_OPAMP6, {11, 0, 0, 3, 0}, 0, NULL, NULL, 0, 0},
{GPIOB, GPIO_PIN_13, CORE_ADC_ALLOWED_ADC3 | CORE_ADC_ALLOWED_OPAMP3 | CORE_ADC_ALLOWED_OPAMP4 | CORE_ADC_ALLOWED_OPAMP6, {0, 0, 5, 0, 0}, 2064, NULL, NULL, 0, 0},
{GPIOB, GPIO_PIN_14, CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_OPAMP2 | CORE_ADC_ALLOWED_OPAMP5, {5, 0, 0, 4, 0}, 4, NULL, NULL, 0, 0},
{GPIOB, GPIO_PIN_15, CORE_ADC_ALLOWED_ADC2 | CORE_ADC_ALLOWED_ADC4, {0, 15, 0, 5, 0}, 0, NULL, NULL, 0, 0},
{GPIOC, GPIO_PIN_0, CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_ADC2, {6, 6, 0, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOC, GPIO_PIN_1, CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_ADC2, {7, 7, 0, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOC, GPIO_PIN_2, CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_ADC2, {8, 8, 0, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOC, GPIO_PIN_3, CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_ADC2 | CORE_ADC_ALLOWED_OPAMP5, {9, 9, 0, 0, 0}, 512, NULL, NULL, 0, 0},
{GPIOC, GPIO_PIN_4, CORE_ADC_ALLOWED_ADC2, {0, 5, 0, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOC, GPIO_PIN_5, CORE_ADC_ALLOWED_ADC2, {0, 11, 0, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOD, GPIO_PIN_10, CORE_ADC_ALLOWED_ADC3 | CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_ADC5, {0, 0, 7, 7, 7}, 0, NULL, NULL, 0, 0},
{GPIOD, GPIO_PIN_11, CORE_ADC_ALLOWED_ADC3 | CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_ADC5 | CORE_ADC_ALLOWED_OPAMP4, {0, 0, 8, 8, 8}, 64, NULL, NULL, 0, 0},
{GPIOD, GPIO_PIN_12, CORE_ADC_ALLOWED_ADC3 | CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_ADC5 | CORE_ADC_ALLOWED_OPAMP5, {0, 0, 9, 9, 9}, 256, NULL, NULL, 0, 0},
{GPIOD, GPIO_PIN_13, CORE_ADC_ALLOWED_ADC3 | CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_ADC5, {0, 0, 10, 10, 10}, 0, NULL, NULL, 0, 0},
{GPIOD, GPIO_PIN_14, CORE_ADC_ALLOWED_ADC3 | CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_ADC5 | CORE_ADC_ALLOWED_OPAMP2, {0, 0, 11, 11, 11}, 12, NULL, NULL, 0, 0},
{GPIOD, GPIO_PIN_8, CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_ADC5, {0, 0, 0, 12, 12}, 0, NULL, NULL, 0, 0},
{GPIOD, GPIO_PIN_9, CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_ADC5 | CORE_ADC_ALLOWED_OPAMP6, {0, 0, 0, 13, 13}, 1024, NULL, NULL, 0, 0},
{GPIOE, GPIO_PIN_7, CORE_ADC_ALLOWED_ADC3, {0, 0, 4, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOE, GPIO_PIN_8, CORE_ADC_ALLOWED_ADC3 | CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_ADC5, {0, 0, 6, 6, 6}, 0, NULL, NULL, 0, 0},
{GPIOE, GPIO_PIN_9, CORE_ADC_ALLOWED_ADC3, {0, 0, 2, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOE, GPIO_PIN_10, CORE_ADC_ALLOWED_ADC3 | CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_ADC5, {0, 0, 14, 14, 14}, 0, NULL, NULL, 0, 0},
{GPIOE, GPIO_PIN_11, CORE_ADC_ALLOWED_ADC3 | CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_ADC5, {0, 0, 15, 15, 15}, 0, NULL, NULL, 0, 0},
{GPIOE, GPIO_PIN_12, CORE_ADC_ALLOWED_ADC3 | CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_ADC5, {0, 0, 16, 16, 16}, 0, NULL, NULL, 0, 0},
{GPIOE, GPIO_PIN_13, CORE_ADC_ALLOWED_ADC3, {0, 0, 3, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOE, GPIO_PIN_14, CORE_ADC_ALLOWED_ADC4, {0, 0, 0, 1, 0}, 0, NULL, NULL, 0, 0},
{GPIOE, GPIO_PIN_15, CORE_ADC_ALLOWED_ADC4, {0, 0, 0, 2, 0}, 0, NULL, NULL, 0, 0},
{GPIOF, GPIO_PIN_0, CORE_ADC_ALLOWED_ADC1, {10, 0, 0, 0, 0}, 0, NULL, NULL, 0, 0},
{GPIOF, GPIO_PIN_1, CORE_ADC_ALLOWED_ADC2, {0, 10, 0, 0, 0}, 0, NULL, NULL, 0, 0},
};

static uint16_t core_ADC_initialized = 0;
static const uint32_t core_ADC_channel_lookup[19] = {
    ADC_CHANNEL_0, ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3,
    ADC_CHANNEL_4, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7,
    ADC_CHANNEL_8, ADC_CHANNEL_9, ADC_CHANNEL_10, ADC_CHANNEL_11,
    ADC_CHANNEL_12, ADC_CHANNEL_13, ADC_CHANNEL_14, ADC_CHANNEL_15,
    ADC_CHANNEL_16, ADC_CHANNEL_17, ADC_CHANNEL_18};


/**
  * @brief  Initialize an ADC module, including its clock. Also performs
  *         calibration. GPIO ports are not initialized.
  * @param  adc The ADC module to initialize
  * @retval 0 if adc is not a valid ADC module or if the ADC fails to
  *         initialize
  * @retval 1 otherwise.
  */
bool core_ADC_init(ADC_TypeDef *adc) {
    // Enable clock for opamps
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    ADC_HandleTypeDef *hadc;
    if (adc == ADC1) {
        hadc = &adc1;
        core_clock_ADC12_init();
        core_ADC_initialized |= CORE_ADC_ALLOWED_ADC1 | CORE_ADC_ALLOWED_OPAMP1;
    } else if (adc == ADC2) {
        hadc = &adc2;
        core_clock_ADC12_init();
        core_ADC_initialized |= CORE_ADC_ALLOWED_ADC2 | CORE_ADC_ALLOWED_OPAMP2 | CORE_ADC_ALLOWED_OPAMP3;
    } else if (adc == ADC3) {
        hadc = &adc3;
        core_clock_ADC345_init();
        core_ADC_initialized |= CORE_ADC_ALLOWED_ADC3 | CORE_ADC_ALLOWED_OPAMP3;
    } else if (adc == ADC4) {
        hadc = &adc4;
        core_clock_ADC345_init();
        core_ADC_initialized |= CORE_ADC_ALLOWED_ADC4 | CORE_ADC_ALLOWED_OPAMP6;
    } else if (adc == ADC5) {
        hadc = &adc5;
        core_clock_ADC345_init();
        core_ADC_initialized |= CORE_ADC_ALLOWED_ADC5 | CORE_ADC_ALLOWED_OPAMP4 | CORE_ADC_ALLOWED_OPAMP5;
    } else return false;

    hadc->Instance = adc;
    hadc->Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc->Init.Resolution = ADC_RESOLUTION_12B;
    hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc->Init.GainCompensation = 0;
    hadc->Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc->Init.LowPowerAutoWait = DISABLE;
    hadc->Init.ContinuousConvMode = DISABLE;
    hadc->Init.NbrOfConversion = 1;
    hadc->Init.DiscontinuousConvMode = DISABLE;
    hadc->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc->Init.DMAContinuousRequests = DISABLE;
    hadc->Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc->Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(hadc) != HAL_OK) return false;

    if (HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED) != HAL_OK) return false;
    return true;
}

/**
  * @brief  Set up a pin as an analog input
  * @param  port GPIO port (GPIOx)
  * @param  pin GPIO pin (GPIO_PIN_x)
  * @param  opamp 1 if the input should be routed through an opamp, 
  *         0 otherwise
  * @retval 1 if a configuration was found for the given pin
  * @retval 0 otherwise
  */
bool core_ADC_setup_pin(GPIO_TypeDef *port, uint32_t pin, uint8_t opamp) {
    core_ADC_def_t *adc_def_ptr = NULL;
    for (uint8_t i=0; i < 42; i++) {
        if ((adc_defs[i].port == port) && (adc_defs[i].pin == pin)) {
            adc_def_ptr = adc_defs + i;
            break;
        }
    }
    if (!adc_def_ptr) return false;
    uint16_t mask = adc_def_ptr->allowed_connections & core_ADC_initialized;
    
    if (opamp) {
        // Look for opamps that can be connected to
        if (mask & CORE_ADC_ALLOWED_OPAMP1) {
            adc_def_ptr->adc = ADC1;
            adc_def_ptr->adc_chan_sel = 13;
            adc_def_ptr->opamp = OPAMP1;
            adc_def_ptr->opamp_chan_sel = (adc_def_ptr->opamp_chan & 3);
        } else if (mask & CORE_ADC_ALLOWED_OPAMP2) {
            adc_def_ptr->adc = ADC2;
            adc_def_ptr->adc_chan_sel = 16;
            adc_def_ptr->opamp = OPAMP2;
            adc_def_ptr->opamp_chan_sel = ((adc_def_ptr->opamp_chan>>2) & 3);
        } else if (mask & CORE_ADC_ALLOWED_OPAMP3) {
            if (core_ADC_initialized & CORE_ADC_ALLOWED_ADC2) {
                adc_def_ptr->adc = ADC2;
                adc_def_ptr->adc_chan_sel = 18;
            } else {
                adc_def_ptr->adc = ADC3;
                adc_def_ptr->adc_chan_sel = 13;
            }
            adc_def_ptr->opamp = OPAMP3;
            adc_def_ptr->opamp_chan_sel = ((adc_def_ptr->opamp_chan>>4) & 3);
        } else if (mask & CORE_ADC_ALLOWED_OPAMP4) {
            adc_def_ptr->adc = ADC5;
            adc_def_ptr->adc_chan_sel = 5;
            adc_def_ptr->opamp = OPAMP4;
            adc_def_ptr->opamp_chan_sel = ((adc_def_ptr->opamp_chan>>6) & 3);
        } else if (mask & CORE_ADC_ALLOWED_OPAMP5) {
            adc_def_ptr->adc = ADC5;
            adc_def_ptr->adc_chan_sel = 3;
            adc_def_ptr->opamp = OPAMP5;
            adc_def_ptr->opamp_chan_sel = ((adc_def_ptr->opamp_chan>>8) & 3);
        } else if (mask & CORE_ADC_ALLOWED_OPAMP6) {
            adc_def_ptr->adc = ADC4;
            adc_def_ptr->adc_chan_sel = 17;
            adc_def_ptr->opamp = OPAMP6;
            adc_def_ptr->opamp_chan_sel = ((adc_def_ptr->opamp_chan>>10) & 3);
        } else return false;
    } else {
        if (mask & CORE_ADC_ALLOWED_ADC1) {
            adc_def_ptr->adc = ADC1;
            adc_def_ptr->adc_chan_sel = adc_def_ptr->chan[0];
        } else if (mask & CORE_ADC_ALLOWED_ADC2) {
            adc_def_ptr->adc = ADC2;
            adc_def_ptr->adc_chan_sel = adc_def_ptr->chan[1];
        } else if (mask & CORE_ADC_ALLOWED_ADC3) {
            adc_def_ptr->adc = ADC3;
            adc_def_ptr->adc_chan_sel = adc_def_ptr->chan[2];
        } else if (mask & CORE_ADC_ALLOWED_ADC4) {
            adc_def_ptr->adc = ADC4;
            adc_def_ptr->adc_chan_sel = adc_def_ptr->chan[3];
        } else if (mask & CORE_ADC_ALLOWED_ADC5) {
            adc_def_ptr->adc = ADC5;
            adc_def_ptr->adc_chan_sel = adc_def_ptr->chan[4];
        }
        else return false;
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.Pin = pin;
        GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStructure.Pull = GPIO_NOPULL;
        GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(port, &GPIO_InitStructure);
    }
    return true;
}


/**
  * @brief  Read the value of an analog input as a value between 0 and 4095
  * @param  port GPIO port of the pin to be read (GPIOx)
  * @param  pin Pin number of the pin to be read (GPIO_PIN_x)
  * @param  result Location to which the result should be stored
  * @retval 0 if the given pin is not an analog input or if the corresponding
  *         ADC module is not initialized or if an error occurs while reading,
  * @retval 1 otherwise
  */
bool core_ADC_read_channel(GPIO_TypeDef *port, uint32_t pin, uint16_t *result) {
    core_ADC_def_t *adc_def_ptr = NULL;
    for (uint8_t i=0; i < 42; i++) {
        if ((adc_defs[i].port == port) && (adc_defs[i].pin == pin)) {
            adc_def_ptr = adc_defs + i;
            break;
        }
    }
    ADC_HandleTypeDef *hadc;
    uint32_t chan;
    if (!adc_def_ptr) return false;
    if (adc_def_ptr->adc == ADC1) {
        hadc = &adc1;
    } else if (adc_def_ptr->adc == ADC2) {
        hadc = &adc2;
    } else if (adc_def_ptr->adc == ADC3) {
        hadc = &adc3;
    } else if (adc_def_ptr->adc == ADC4) {
        hadc = &adc4;
    } else if (adc_def_ptr->adc == ADC5) {
        hadc = &adc5;
    } else return false;

    if (adc_def_ptr->opamp) {
        adc_def_ptr->opamp->CSR = OPAMP_CSR_VMSEL_0 | OPAMP_CSR_VMSEL_1 | 1 | OPAMP_CSR_OPAMPINTEN | ((adc_def_ptr->opamp_chan_sel<<2) & 0xc);
    }

    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel = core_ADC_channel_lookup[adc_def_ptr->adc_chan_sel];
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) return false;
    // Perform reading
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
    *result = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return true;
}

