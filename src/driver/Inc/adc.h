#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>

#ifndef CORE_ADC_H
#define CORE_ADC_H

#define CORE_ADC_ALLOWED_ADC1       0x0001
#define CORE_ADC_ALLOWED_ADC2       0x0002
#define CORE_ADC_ALLOWED_ADC3       0x0004
#define CORE_ADC_ALLOWED_ADC4       0x0008
#define CORE_ADC_ALLOWED_ADC5       0x0010
#define CORE_ADC_ALLOWED_OPAMP1     0x0100
#define CORE_ADC_ALLOWED_OPAMP2     0x0200
#define CORE_ADC_ALLOWED_OPAMP3     0x0400
#define CORE_ADC_ALLOWED_OPAMP4     0x0800
#define CORE_ADC_ALLOWED_OPAMP5     0x1000
#define CORE_ADC_ALLOWED_OPAMP6     0x2000
#define CORE_ADC_ALLOWED_ADC_MASK   0x001f
#define CORE_ADC_ALLOWED_OPAMP_MASK 0x3f00

typedef struct core_ADC_def_s {
    // Constant fields
    GPIO_TypeDef *port;
    uint32_t pin;
    uint16_t allowed_connections;
    uint8_t chan[5];
    uint16_t opamp_chan;
    // Fields edited by the program
    ADC_TypeDef *adc;
    OPAMP_TypeDef *opamp;
    uint8_t adc_chan_sel;
    uint8_t opamp_chan_sel;
} core_ADC_def_t;


bool core_ADC_init(ADC_TypeDef *adc);
bool core_ADC_setup_pin(GPIO_TypeDef *port, uint32_t pin, uint8_t opamp);
bool core_ADC_read_channel(GPIO_TypeDef *port, uint32_t pin, uint16_t *result);
uint16_t core_ADC_read_vrefint();

#endif
