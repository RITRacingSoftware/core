#include adc.h

__weak bool core_ADC_init(ADC_TypeDef *adc) {}
__weak bool core_ADC_setup_pin(GPIO_TypeDef *port, uint32_t pin, uint8_t opamp) {}
__weak bool core_ADC_read_channel(GPIO_TypeDef *port, uint32_t pin, uint16_t *result) {}
