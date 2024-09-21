#include <stdint.h>
#include <stdbool.h>
#include <stm32g4xx_hal.h>
#include <stm32g4xx_hal_spi.h>

bool core_SPI_init(SPI_TypeDef *spi) {
    if (spi == SPI1) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitTypeDef spiGPIOinit1 = {GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0};
        HAL_GPIO_Init(GPIOA, &spiGPIOinit1);
        GPIO_InitTypeDef spiGPIOinit2 = {GPIO_PIN_5, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 5};
        HAL_GPIO_Init(GPIOA, &spiGPIOinit2);
        GPIO_InitTypeDef spiGPIOinit3 = {GPIO_PIN_6, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 5};
        HAL_GPIO_Init(GPIOA, &spiGPIOinit3);
        GPIO_InitTypeDef spiGPIOinit4 = {GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 5};
        HAL_GPIO_Init(GPIOA, &spiGPIOinit4);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        __HAL_RCC_SPI1_CLK_ENABLE();
    }
    else if (spi == SPI2) {
        /*GPIO_InitTypeDef spiGPIOinit1 = {12, GPIO_Mode_OUT, GPIO_Speed_Level_3, GPIO_OType_PP, GPIO_PuPd_NOPULL};
        GPIO_Init(GPIOB, &spiGPIOinit1);
        GPIO_InitTypeDef spiGPIOinit2 = {13, GPIO_Mode_AF, GPIO_Speed_Level_3, GPIO_OType_PP, GPIO_PuPd_NOPULL};
        GPIO_Init(GPIOB, &spiGPIOinit2);
        GPIO_InitTypeDef spiGPIOinit3 = {14, GPIO_Mode_AF, GPIO_Speed_Level_3, GPIO_OType_PP, GPIO_PuPd_NOPULL}; //Not sure about speed, OType, and PuPd
        GPIO_Init(GPIOB, &spiGPIOinit3);
        GPIO_InitTypeDef spiGPIOinit4 = {15, GPIO_Mode_AF, GPIO_Speed_Level_3, GPIO_OType_PP, GPIO_PuPd_NOPULL};
        GPIO_Init(GPIOB, &spiGPIOinit4);
        GPIO_WriteBit(GPIOB, 12, 1);
        GPIO_PinAFConfig(GPIOB, 13, GPIO_AF_5);
        GPIO_PinAFConfig(GPIOB, 14, GPIO_AF_5);
        GPIO_PinAFConfig(GPIOB, 15, GPIO_AF_5);*/
        __HAL_RCC_SPI2_CLK_ENABLE();
    }
    else if (spi == SPI3) __HAL_RCC_SPI3_CLK_ENABLE();
    else if (spi == SPI4) __HAL_RCC_SPI4_CLK_ENABLE();
    else return false;

    // SPI master, /256 prescaler, CPOL=0, CPHA=0
    spi->I2SCFGR = 0;
    spi->CR1 = (7 << SPI_CR1_BR_Pos) | SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;
    spi->CR2 = SPI_CR2_SSOE | (7 << SPI_CR2_DS_Pos) | SPI_CR2_FRXTH;
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
    //spi->DR = value;
    uint32_t total = (txbuflen > rxbuflen ? txbuflen : rxbuflen);
    uint32_t n_tx = 0;
    uint32_t n_rx = 0;
    while ((n_tx < total) || (n_rx < total)) {
        if (spi->SR & SPI_SR_TXE) {
            *(__IO uint8_t *)&(spi->DR) = (n_tx < txbuflen ? txbuf[n_tx] : 0x00);
            n_tx++;
        }
        if (spi->SR & SPI_SR_RXNE) {
            if (n_rx < rxbuflen) rxbuf[n_rx] = *(__IO uint8_t *)&(spi->DR);
            n_rx++;
        }
    }
    return true;
}
