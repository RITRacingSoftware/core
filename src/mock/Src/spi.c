#include spi.h

__weak bool core_SPI_init(SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {}
__weak bool core_SPI_read_write(SPI_TypeDef *spi, uint8_t *txbuf, uint32_t txbuflen, uint8_t *rxbuf, uint32_t rxbuflen) {}
__weak bool core_SPI_start(SPI_TypeDef *spi) {}
__weak bool core_SPI_stop(SPI_TypeDef *spi) {}
