#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>

typedef struct core_ADC_def_s {
    uint32_t chan;
    GPIO_TypeDef *port;
    uint32_t pin;
} core_ADC_def_t;

#ifndef CORE_ADC_H
#define CORE_ADC_H

bool core_ADC_init(ADC_TypeDef *adc);
void core_ADC_setup_pin(GPIO_TypeDef *port, uint32_t pin);
bool core_ADC_convert_pin(GPIO_TypeDef *port, uint32_t pin, ADC_TypeDef *adc, ADC_HandleTypeDef **hadc_r, uint32_t *chan_r);
bool core_ADC_read_channel(GPIO_TypeDef *port, uint32_t pin, uint16_t *result, ADC_TypeDef *preference);

#endif
