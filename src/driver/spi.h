#ifndef CORE_SPI_H
#define CORE_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>

bool core_SPI_init(SPI_TypeDef *spi);
bool core_SPI_fifowrite(SPI_TypeDef *spi, uint8_t value);

#endif
