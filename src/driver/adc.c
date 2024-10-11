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

static const core_ADC_def_t adc1_defs[19] = {
    {ADC_CHANNEL_0, NULL, 0},
    {ADC_CHANNEL_1, GPIOA, GPIO_PIN_0},
    {ADC_CHANNEL_2, GPIOA, GPIO_PIN_1},
    {ADC_CHANNEL_3, GPIOA, GPIO_PIN_2},
    {ADC_CHANNEL_4, GPIOA, GPIO_PIN_3},
    {ADC_CHANNEL_5, GPIOB, GPIO_PIN_14},
    {ADC_CHANNEL_6, GPIOC, GPIO_PIN_0},
    {ADC_CHANNEL_7, GPIOC, GPIO_PIN_1},
    {ADC_CHANNEL_8, GPIOC, GPIO_PIN_2},
    {ADC_CHANNEL_9, GPIOC, GPIO_PIN_3},
    {ADC_CHANNEL_10, GPIOF, GPIO_PIN_0},
    {ADC_CHANNEL_11, GPIOB, GPIO_PIN_12},
    {ADC_CHANNEL_12, GPIOB, GPIO_PIN_1},
    {ADC_CHANNEL_13, NULL, 0},
    {ADC_CHANNEL_14, GPIOB, GPIO_PIN_11},
    {ADC_CHANNEL_15, GPIOB, GPIO_PIN_0},
    {ADC_CHANNEL_16, NULL, 0},
    {ADC_CHANNEL_17, NULL, 0},
    {ADC_CHANNEL_18, NULL, 0}
};
static const core_ADC_def_t adc2_defs[19] = {
    {ADC_CHANNEL_0, NULL, 0},
    {ADC_CHANNEL_1, GPIOA, GPIO_PIN_0},
    {ADC_CHANNEL_2, GPIOA, GPIO_PIN_1},
    {ADC_CHANNEL_3, GPIOA, GPIO_PIN_6},
    {ADC_CHANNEL_4, GPIOA, GPIO_PIN_7},
    {ADC_CHANNEL_5, GPIOC, GPIO_PIN_4},
    {ADC_CHANNEL_6, GPIOC, GPIO_PIN_0},
    {ADC_CHANNEL_7, GPIOC, GPIO_PIN_1},
    {ADC_CHANNEL_8, GPIOC, GPIO_PIN_2},
    {ADC_CHANNEL_9, GPIOC, GPIO_PIN_3},
    {ADC_CHANNEL_10, GPIOF, GPIO_PIN_1},
    {ADC_CHANNEL_11, GPIOC, GPIO_PIN_5},
    {ADC_CHANNEL_12, GPIOB, GPIO_PIN_2},
    {ADC_CHANNEL_13, GPIOA, GPIO_PIN_5},
    {ADC_CHANNEL_14, GPIOB, GPIO_PIN_11},
    {ADC_CHANNEL_15, GPIOB, GPIO_PIN_15},
    {ADC_CHANNEL_16, NULL, 0},
    {ADC_CHANNEL_17, GPIOA, GPIO_PIN_4},
    {ADC_CHANNEL_18, NULL, 0}
};
static const core_ADC_def_t adc3_defs[19] = {
    {ADC_CHANNEL_0, NULL, 0},
    {ADC_CHANNEL_1, GPIOB, GPIO_PIN_1},
    {ADC_CHANNEL_2, GPIOE, GPIO_PIN_9},
    {ADC_CHANNEL_3, GPIOE, GPIO_PIN_13},
    {ADC_CHANNEL_4, GPIOE, GPIO_PIN_7},
    {ADC_CHANNEL_5, GPIOB, GPIO_PIN_13},
    {ADC_CHANNEL_6, GPIOE, GPIO_PIN_8},
    {ADC_CHANNEL_7, GPIOD, GPIO_PIN_10},
    {ADC_CHANNEL_8, GPIOD, GPIO_PIN_11},
    {ADC_CHANNEL_9, GPIOD, GPIO_PIN_12},
    {ADC_CHANNEL_10, GPIOD, GPIO_PIN_13},
    {ADC_CHANNEL_11, GPIOD, GPIO_PIN_14},
    {ADC_CHANNEL_12, GPIOB, GPIO_PIN_0},
    {ADC_CHANNEL_13, NULL, 0},
    {ADC_CHANNEL_14, GPIOE, GPIO_PIN_10},
    {ADC_CHANNEL_15, GPIOE, GPIO_PIN_11},
    {ADC_CHANNEL_16, GPIOE, GPIO_PIN_12},
    {ADC_CHANNEL_17, NULL, 0},
    {ADC_CHANNEL_18, NULL, 0}
};
static const core_ADC_def_t adc4_defs[19] = {
    {ADC_CHANNEL_0, NULL, 0},
    {ADC_CHANNEL_1, GPIOE, GPIO_PIN_14},
    {ADC_CHANNEL_2, GPIOE, GPIO_PIN_15},
    {ADC_CHANNEL_3, GPIOB, GPIO_PIN_12},
    {ADC_CHANNEL_4, GPIOB, GPIO_PIN_14},
    {ADC_CHANNEL_5, GPIOB, GPIO_PIN_15},
    {ADC_CHANNEL_6, GPIOE, GPIO_PIN_8},
    {ADC_CHANNEL_7, GPIOD, GPIO_PIN_10},
    {ADC_CHANNEL_8, GPIOD, GPIO_PIN_11},
    {ADC_CHANNEL_9, GPIOD, GPIO_PIN_12},
    {ADC_CHANNEL_10, GPIOD, GPIO_PIN_13},
    {ADC_CHANNEL_11, GPIOD, GPIO_PIN_14},
    {ADC_CHANNEL_12, GPIOD, GPIO_PIN_8},
    {ADC_CHANNEL_13, GPIOD, GPIO_PIN_9},
    {ADC_CHANNEL_14, GPIOE, GPIO_PIN_10},
    {ADC_CHANNEL_15, GPIOE, GPIO_PIN_11},
    {ADC_CHANNEL_16, GPIOE, GPIO_PIN_12},
    {ADC_CHANNEL_17, NULL, 0},
    {ADC_CHANNEL_18, NULL, 0}
};
static const core_ADC_def_t adc5_defs[19] = {
    {ADC_CHANNEL_0, NULL, 0},
    {ADC_CHANNEL_1, GPIOA, GPIO_PIN_8},
    {ADC_CHANNEL_2, GPIOA, GPIO_PIN_9},
    {ADC_CHANNEL_3, NULL, 0},
    {ADC_CHANNEL_4, NULL, 0},
    {ADC_CHANNEL_5, NULL, 0},
    {ADC_CHANNEL_6, GPIOE, GPIO_PIN_8},
    {ADC_CHANNEL_7, GPIOD, GPIO_PIN_10},
    {ADC_CHANNEL_8, GPIOD, GPIO_PIN_11},
    {ADC_CHANNEL_9, GPIOD, GPIO_PIN_12},
    {ADC_CHANNEL_10, GPIOD, GPIO_PIN_13},
    {ADC_CHANNEL_11, GPIOD, GPIO_PIN_14},
    {ADC_CHANNEL_12, GPIOD, GPIO_PIN_8},
    {ADC_CHANNEL_13, GPIOD, GPIO_PIN_9},
    {ADC_CHANNEL_14, GPIOE, GPIO_PIN_10},
    {ADC_CHANNEL_15, GPIOE, GPIO_PIN_11},
    {ADC_CHANNEL_16, GPIOE, GPIO_PIN_12},
    {ADC_CHANNEL_17, NULL, 0},
    {ADC_CHANNEL_18, NULL, 0}
};

bool core_ADC_init(ADC_TypeDef *adc) {
    ADC_HandleTypeDef *hadc;
    if (adc == ADC1) {
        hadc = &adc1;
        core_clock_ADC12_init();
    } else if (adc == ADC2) {
        hadc = &adc2;
        core_clock_ADC12_init();
    } else if (adc == ADC3) {
        hadc = &adc3;
        core_clock_ADC345_init();
    } else if (adc == ADC4) {
        hadc = &adc4;
        core_clock_ADC345_init();
    } else if (adc == ADC5) {
        hadc = &adc5;
        core_clock_ADC345_init();
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

void core_ADC_setup_pin(GPIO_TypeDef *port, uint32_t pin) {
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin = pin;
    GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(port, &GPIO_InitStructure);
}

bool core_ADC_convert_pin(GPIO_TypeDef *port, uint32_t pin, ADC_TypeDef *adc, ADC_HandleTypeDef **hadc_r, uint32_t *chan_r) {
    if (adc == NULL) return false;
    const core_ADC_def_t *defs = NULL;
    ADC_HandleTypeDef *hadc;
    if (adc == ADC1) {
        defs = adc1_defs;
        hadc = &adc1;
    } else if (adc == ADC2) {
        defs = adc2_defs;
        hadc = &adc2;
    } else if (adc == ADC3) {
        defs = adc3_defs;
        hadc = &adc3;
    } else if (adc == ADC4) {
        defs = adc4_defs;
        hadc = &adc4;
    } else if (adc == ADC5) {
        defs = adc5_defs;
        hadc = &adc5;
    } else return false;
    if (hadc->Instance == NULL) return false;
    for (uint8_t i=0; i < 19; i++) {
        if ((defs[i].port == port) && (defs[i].pin == pin)) {
            *hadc_r = hadc;
            *chan_r = defs[i].chan;
            return true;
        }
    }
    return false;
}

bool core_ADC_read_channel(GPIO_TypeDef *port, uint32_t pin, uint16_t *result, ADC_TypeDef *preference) {
    // Initialize channel
    ADC_HandleTypeDef *hadc;
    uint32_t chan;
    if (!(core_ADC_convert_pin(port, pin, preference, &hadc, &chan) || 
                core_ADC_convert_pin(port, pin, ADC1, &hadc, &chan) || 
                core_ADC_convert_pin(port, pin, ADC2, &hadc, &chan) || 
                core_ADC_convert_pin(port, pin, ADC3, &hadc, &chan) || 
                core_ADC_convert_pin(port, pin, ADC4, &hadc, &chan) || 
                core_ADC_convert_pin(port, pin, ADC5, &hadc, &chan))) return false;


    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel = chan;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
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
