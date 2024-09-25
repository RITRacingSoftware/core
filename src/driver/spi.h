#ifndef CORE_SPI_H
#define CORE_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>

// Note that if you change the SPI ports, you must also change
// the GPIO port init calls in spi.c
#define CORE_SPI1_SCK_PORT  GPIOA
#define CORE_SPI1_SCK_PIN   GPIO_PIN_5
#define CORE_SPI1_MISO_PORT GPIOA
#define CORE_SPI1_MISO_PIN  GPIO_PIN_6
#define CORE_SPI1_MOSI_PORT GPIOA
#define CORE_SPI1_MOSI_PIN  GPIO_PIN_7

#define CORE_SPI2_SCK_PORT  GPIOB
#define CORE_SPI2_SCK_PIN   GPIO_PIN_13
#define CORE_SPI2_MISO_PORT GPIOB
#define CORE_SPI2_MISO_PIN  GPIO_PIN_14
#define CORE_SPI2_MOSI_PORT GPIOB
#define CORE_SPI2_MOSI_PIN  GPIO_PIN_15

#define CORE_SPI3_SCK_PORT  GPIOC
#define CORE_SPI3_SCK_PIN   GPIO_PIN_10
#define CORE_SPI3_MISO_PORT GPIOC
#define CORE_SPI3_MISO_PIN  GPIO_PIN_11
#define CORE_SPI3_MOSI_PORT GPIOC
#define CORE_SPI3_MOSI_PIN  GPIO_PIN_12

#define CORE_SPI4_SCK_PORT  GPIOE
#define CORE_SPI4_SCK_PIN   GPIO_PIN_12
#define CORE_SPI4_MISO_PORT GPIOE
#define CORE_SPI4_MISO_PIN  GPIO_PIN_13
#define CORE_SPI4_MOSI_PORT GPIOE
#define CORE_SPI4_MOSI_PIN  GPIO_PIN_14


bool core_SPI_init(SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
bool core_SPI_read_write(SPI_TypeDef *spi, uint8_t *txbuf, uint32_t txbuflen, uint8_t *rxbuf, uint32_t rxbuflen);

#endif
