#include gpio.h

__weak void core_GPIO_init(GPIO_TypeDef *port, uint16_t pin, uint16_t dir, uint32_t pull) {}
__weak void core_GPIO_digital_write(GPIO_TypeDef *port, uint16_t pin, bool state) {}
__weak bool core_GPIO_digital_read(GPIO_TypeDef *port, uint16_t pin) {}
__weak void core_heartbeat_init(GPIO_TypeDef *port, uint16_t pin) {}
__weak void core_GPIO_toggle_heartbeat() {}
__weak void core_GPIO_set_heartbeat(bool state {}
