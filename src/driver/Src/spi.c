#include "spi.h"
#include "core_config.h"

#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_spi.h>

#include "clock.h"

uint16_t core_SPI1_CS_pin;
GPIO_TypeDef *core_SPI1_CS_port;
uint16_t core_SPI2_CS_pin;
GPIO_TypeDef *core_SPI2_CS_port;
uint16_t core_SPI3_CS_pin;
GPIO_TypeDef *core_SPI3_CS_port;
uint16_t core_SPI4_CS_pin;
GPIO_TypeDef *core_SPI4_CS_port;

bool core_SPI_init(SPI_TypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    uint8_t div;
    uint8_t data_size;
    uint8_t master;
    if (spi == SPI1) {
        core_SPI1_CS_port = cs_port;
        core_SPI1_CS_pin = cs_pin;
        __HAL_RCC_GPIOA_CLK_ENABLE();
        core_clock_port_init(CORE_SPI1_SCK_PORT);
        GPIO_InitTypeDef spiGPIOinit2 = {CORE_SPI1_SCK_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CORE_SPI1_SCK_AF};
        HAL_GPIO_Init(CORE_SPI1_SCK_PORT, &spiGPIOinit2);
        core_clock_port_init(CORE_SPI1_MISO_PORT);
        GPIO_InitTypeDef spiGPIOinit3 = {CORE_SPI1_MISO_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CORE_SPI1_MISO_AF};
        HAL_GPIO_Init(CORE_SPI1_MISO_PORT, &spiGPIOinit3);
        core_clock_port_init(CORE_SPI1_MOSI_PORT);
        GPIO_InitTypeDef spiGPIOinit4 = {CORE_SPI1_MOSI_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CORE_SPI1_MOSI_AF};
        HAL_GPIO_Init(CORE_SPI1_MOSI_PORT, &spiGPIOinit4);
        __HAL_RCC_SPI1_CLK_ENABLE();
        div = CORE_SPI1_DIVIDER;
        data_size = CORE_SPI1_DATA_SIZE - 1;
        master = CORE_SPI1_MASTER;
    }
    else if (spi == SPI2) {
        core_SPI2_CS_port = cs_port;
        core_SPI2_CS_pin = cs_pin;
        core_clock_port_init(CORE_SPI2_SCK_PORT);
        GPIO_InitTypeDef spiGPIOinit2 = {CORE_SPI2_SCK_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CORE_SPI2_SCK_AF};
        HAL_GPIO_Init(CORE_SPI2_SCK_PORT, &spiGPIOinit2);
        core_clock_port_init(CORE_SPI2_MISO_PORT);
        GPIO_InitTypeDef spiGPIOinit3 = {CORE_SPI2_MISO_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CORE_SPI2_MISO_AF};
        HAL_GPIO_Init(CORE_SPI2_MISO_PORT, &spiGPIOinit3);
        core_clock_port_init(CORE_SPI2_MOSI_PORT);
        GPIO_InitTypeDef spiGPIOinit4 = {CORE_SPI2_MOSI_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CORE_SPI2_MOSI_AF};
        HAL_GPIO_Init(CORE_SPI2_MOSI_PORT, &spiGPIOinit4);
        __HAL_RCC_SPI2_CLK_ENABLE();
        div = CORE_SPI2_DIVIDER;
        data_size = CORE_SPI2_DATA_SIZE - 1;
        master = CORE_SPI2_MASTER;
    }
    else if (spi == SPI3) {
        core_SPI3_CS_port = cs_port;
        core_SPI3_CS_pin = cs_pin;
        core_clock_port_init(CORE_SPI3_SCK_PORT);
        GPIO_InitTypeDef spiGPIOinit2 = {CORE_SPI3_SCK_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CORE_SPI3_SCK_AF};
        HAL_GPIO_Init(CORE_SPI3_SCK_PORT, &spiGPIOinit2);
        core_clock_port_init(CORE_SPI3_MISO_PORT);
        GPIO_InitTypeDef spiGPIOinit3 = {CORE_SPI3_MISO_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CORE_SPI3_MISO_AF};
        HAL_GPIO_Init(CORE_SPI3_MISO_PORT, &spiGPIOinit3);
        core_clock_port_init(CORE_SPI3_MOSI_PORT);
        GPIO_InitTypeDef spiGPIOinit4 = {CORE_SPI3_MOSI_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CORE_SPI3_MOSI_AF};
        HAL_GPIO_Init(CORE_SPI3_MOSI_PORT, &spiGPIOinit4);
        __HAL_RCC_SPI3_CLK_ENABLE();
        div = CORE_SPI3_DIVIDER;
        data_size = CORE_SPI3_DATA_SIZE - 1;
        master = CORE_SPI3_MASTER;
    }
    else if (spi == SPI4) {
        core_SPI4_CS_port = cs_port;
        core_SPI4_CS_pin = cs_pin;
        core_clock_port_init(CORE_SPI4_SCK_PORT);
        GPIO_InitTypeDef spiGPIOinit2 = {CORE_SPI4_SCK_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CORE_SPI4_SCK_AF};
        HAL_GPIO_Init(CORE_SPI4_SCK_PORT, &spiGPIOinit2);
        core_clock_port_init(CORE_SPI4_MISO_PORT);
        GPIO_InitTypeDef spiGPIOinit3 = {CORE_SPI4_MISO_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CORE_SPI4_MISO_AF};
        HAL_GPIO_Init(CORE_SPI4_MISO_PORT, &spiGPIOinit3);
        core_clock_port_init(CORE_SPI4_MOSI_PORT);
        GPIO_InitTypeDef spiGPIOinit4 = {CORE_SPI4_MOSI_PIN, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, CORE_SPI4_MOSI_AF};
        HAL_GPIO_Init(CORE_SPI4_MOSI_PORT, &spiGPIOinit4);
        __HAL_RCC_SPI4_CLK_ENABLE();
        div = CORE_SPI4_DIVIDER;
        data_size = CORE_SPI4_DATA_SIZE - 1;
        master = CORE_SPI4_MASTER;
    }
    else return false;
    if ((div > 7) || (data_size > 15)) return false;
    
    if (master) {
        GPIO_InitTypeDef cs_init = {cs_pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, 0};
        HAL_GPIO_Init(cs_port, &cs_init);
        HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    } else {
        GPIO_InitTypeDef cs_init = {cs_pin, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, 5};
        HAL_GPIO_Init(cs_port, &cs_init);
    }

    // SPI master, /256 prescaler, CPOL=0, CPHA=0
    spi->I2SCFGR = 0;
    spi->CR1 = (div << SPI_CR1_BR_Pos) | (master ? (SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM) : 0);
    spi->CR2 = SPI_CR2_SSOE | (data_size << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
    spi->CR1 |= SPI_CR1_SPE;

    /*SPI_InitTypeDef spiInit;
    spiInit.Direction = SPI_DIRECTION_2LINES;
    spiInit.Mode = SPI_MODE_MASTER;
    spiInit.DataSize = SPI_DATASIZE_8BIT;
    spiInit.CPOL = SPI_POLARITY_HIGH;
    spiInit.CPHA = SPI_PHASE_2EDGE;
    spiInit.NSS = SPI_NSS_SOFT;
    spiInit.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
    spiInit.FirstBit = SPI_FIRSTBIT_MSB;
    spiInit.CRCPolynomial = 0;
    HAL_SPI_I2S_DeInit(spi);

    HAL_SPI_Init(spi, &spiInit);
    //HAL_SPI_RxFIFOThresholdConfig(SPI1, SPI_RXFIFO_THRESHOLD_HF); //Don't know if should be HF or QF
    //HAL_SPI_Cmd(SPI1, ENABLE);
    spi->CR1 |= SPI_CR1_SPE;*/
    return true;
}

bool core_SPI_read_write(SPI_TypeDef *spi, uint8_t *txbuf, uint32_t txbuflen, uint8_t *rxbuf, uint32_t rxbuflen) {
    uint32_t total = (txbuflen > rxbuflen ? txbuflen : rxbuflen);
    uint32_t n_tx = 0;
    uint32_t n_rx = 0;
    uint8_t temp;
    while (spi->SR & SPI_SR_RXNE) temp = *(__IO uint8_t *)&(spi->DR);
    while ((n_tx < total) || (n_rx < total)) {
        if ((spi->SR & SPI_SR_TXE) && (n_tx < total)) {
            *(__IO uint8_t *)&(spi->DR) = (n_tx < txbuflen ? txbuf[n_tx] : 0x00);
            n_tx++;
        }
        if (spi->SR & SPI_SR_RXNE) {
            if (n_rx < rxbuflen) rxbuf[n_rx] = *(__IO uint8_t *)&(spi->DR);
            n_rx++;
        }
    }
    while (spi->SR & SPI_SR_BSY);
    return true;
}

bool core_SPI_start(SPI_TypeDef *spi) {
    uint16_t cs_pin;
    GPIO_TypeDef *cs_port;
    if (spi == SPI1) {
        cs_pin = core_SPI1_CS_pin;
        cs_port = core_SPI1_CS_port;
    } else if (spi == SPI2) {
        cs_pin = core_SPI2_CS_pin;
        cs_port = core_SPI2_CS_port;
    } else if (spi == SPI3) {
        cs_pin = core_SPI3_CS_pin;
        cs_port = core_SPI3_CS_port;
    } else if (spi == SPI4) {
        cs_pin = core_SPI4_CS_pin;
        cs_port = core_SPI4_CS_port;
    } else return false;
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    return true;
}

bool core_SPI_stop(SPI_TypeDef *spi) {
    uint16_t cs_pin;
    GPIO_TypeDef *cs_port;
    if (spi == SPI1) {
        cs_pin = core_SPI1_CS_pin;
        cs_port = core_SPI1_CS_port;
    } else if (spi == SPI2) {
        cs_pin = core_SPI2_CS_pin;
        cs_port = core_SPI2_CS_port;
    } else if (spi == SPI3) {
        cs_pin = core_SPI3_CS_pin;
        cs_port = core_SPI3_CS_port;
    } else if (spi == SPI4) {
        cs_pin = core_SPI4_CS_pin;
        cs_port = core_SPI4_CS_port;
    } else return false;
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    return true;
}

