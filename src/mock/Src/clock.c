#include clock.h

__weak void core_clock_ADC12_init() {}
__weak void core_clock_ADC345_init() {}
__weak void core_clock_FDCAN_init(FDCAN_GlobalTypeDef *can) {}
__weak bool core_clock_USART_init(USART_TypeDef *usart) {}
__weak bool core_clock_I2C_init(I2C_TypeDef *i2c) {}
__weak bool core_clock_RTC_init() {}
__weak void core_clock_port_init(GPIO_TypeDef *port) {}
__weak uint8_t core_clock_generate_params(uint32_t src_freq, uint32_t target_freq, uint8_t *n, uint8_t *m, uint8_t *r) {}
__weak bool core_clock_init() {}
