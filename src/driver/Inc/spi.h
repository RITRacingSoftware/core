/**
  * @file   spi.h
  * @brief  Core SPI library
  *
  * This core library component is used to interact with SPI devices.
  */

#ifndef CORE_SPI_H
#define CORE_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>


/**
  * @brief  Initialize an SPI module and set up a CS pin for it
  * @param  spi The SPI module to be initialized
  * @param  cs_port Port the CS pin is located on (GPIOx)
  * @param  cs_pin CS pin (GPIO_PIN_x)
  */
bool core_SPI_init(SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin);

/**
  * @brief  Transmit data from a buffer on the given SPI bus and store the
  *         incoming data in a separate buffer
  * @note   The number of bytes transmitted or received on the SPI bus is the
  *         greater of txbuflen and rxbuflen. Only the number of bytes given
  *         by each parameter will be read/stored respectively.
  * @param  spi The SPI module
  * @param  txbuf Buffer from which data to be transmitted is read
  * @param  txbuflen Number of bytes to read from the TX buffer
  * @param  rxbuf Buffer to which received data is to be stored
  * @param  rxbuflen Number of bytes to write to the RX buffer
  */
bool core_SPI_read_write(SPI_TypeDef *spi, uint8_t *txbuf, uint32_t txbuflen, uint8_t *rxbuf, uint32_t rxbuflen);

/**
  * @brief  Set the CS pin for an SPI bus low
  * @param  spi The SPI module
  */
bool core_SPI_start(SPI_TypeDef *spi);

/**
  * @brief  Set the CS pin for an SPI bus high
  * @param  spi The SPI module
  */
bool core_SPI_stop(SPI_TypeDef *spi);

#endif
