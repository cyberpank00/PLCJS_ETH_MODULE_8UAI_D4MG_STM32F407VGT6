/**
  ******************************************************************************
  * @file    board_io.c
  * @brief   8UAI thin hardware layer (see board_io.h). Register level; every
  *          function the protection ISR can reach lives in RAM.
  ******************************************************************************
  */
#include "board_io.h"

static uint8_t s_mode = 0xFFu;

void spi_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    (void)RCC->APB2ENR;

    BOARD_SPI->CR1 = 0u;
    BOARD_SPI->CR2 = 0u;
    /* Master, PCLK2/8 = 10 MHz, software NSS, CPOL 0, mode 1, 16-bit. */
    BOARD_SPI->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_1 | SPI_CR1_SSM | SPI_CR1_SSI |
                     SPI_CR1_CPHA | SPI_CR1_DFF;
    BOARD_SPI->CR1 |= SPI_CR1_SPE;
    s_mode = BOARD_SPI_MODE_ADC;

    board_spi_pins_af();
}

BOARD_INLINE void wait_idle(void)
{
    while ((BOARD_SPI->SR & SPI_SR_TXE) == 0u) { }
    while ((BOARD_SPI->SR & SPI_SR_BSY) != 0u) { }
}

BOARD_RAMFUNC void spi_set_mode(uint8_t mode)
{
    if (mode == s_mode) { return; }
    wait_idle();
    uint32_t cr1 = BOARD_SPI->CR1 & ~SPI_CR1_SPE;
    BOARD_SPI->CR1 = cr1;
    cr1 &= ~(SPI_CR1_CPHA | SPI_CR1_DFF);
    if (mode == BOARD_SPI_MODE_ADC) { cr1 |= SPI_CR1_CPHA | SPI_CR1_DFF; }
    BOARD_SPI->CR1 = cr1;
    BOARD_SPI->CR1 = cr1 | SPI_CR1_SPE;
    s_mode = mode;
}

/* Detach the RX DMA request for a polled transaction and drain stale data. */
BOARD_INLINE uint32_t rx_dma_detach(void)
{
    const uint32_t saved = BOARD_SPI->CR2 & SPI_CR2_RXDMAEN;
    BOARD_SPI->CR2 &= ~SPI_CR2_RXDMAEN;
    while ((BOARD_SPI->SR & SPI_SR_RXNE) != 0u) { (void)BOARD_SPI->DR; }
    (void)BOARD_SPI->SR;   /* DR then SR read clears OVR */
    return saved;
}

BOARD_INLINE void rx_dma_restore(uint32_t saved)
{
    BOARD_SPI->CR2 |= saved;
}

BOARD_INLINE uint16_t xchg(uint16_t v)
{
    BOARD_SPI->DR = v;
    while ((BOARD_SPI->SR & SPI_SR_RXNE) == 0u) { }
    return (uint16_t)BOARD_SPI->DR;
}

BOARD_RAMFUNC void spi_xfer(uint8_t mode, const uint8_t* tx, uint8_t* rx, uint16_t len)
{
    spi_set_mode(mode);
    const uint32_t saved = rx_dma_detach();
    if (mode == BOARD_SPI_MODE_ADC) {
        for (uint16_t i = 0u; i + 1u < len; i += 2u) {
            const uint16_t r = xchg((uint16_t)(((uint16_t)tx[i] << 8) | tx[i + 1u]));
            if (rx != NULL) { rx[i] = (uint8_t)(r >> 8); rx[i + 1u] = (uint8_t)r; }
        }
    } else {
        for (uint16_t i = 0u; i < len; i++) {
            const uint16_t r = xchg(tx[i]);
            if (rx != NULL) { rx[i] = (uint8_t)r; }
        }
    }
    wait_idle();
    rx_dma_restore(saved);
}

BOARD_RAMFUNC uint32_t board_adc_frame(uint16_t cmd)
{
    spi_set_mode(BOARD_SPI_MODE_ADC);
    const uint32_t saved = rx_dma_detach();

    gpio_write(BOARD_SIG_CS, 0u);
    board_delay_100ns();
    BOARD_SPI->DR = cmd;
    while ((BOARD_SPI->SR & SPI_SR_TXE) == 0u) { }
    BOARD_SPI->DR = 0x0000u;
    while ((BOARD_SPI->SR & SPI_SR_RXNE) == 0u) { }
    const uint16_t first = (uint16_t)BOARD_SPI->DR;
    while ((BOARD_SPI->SR & SPI_SR_RXNE) == 0u) { }
    const uint16_t data = (uint16_t)BOARD_SPI->DR;
    wait_idle();
    board_delay_100ns();
    gpio_write(BOARD_SIG_CS, 1u);
    board_delay_100ns();

    rx_dma_restore(saved);
    return ((uint32_t)first << 16) | data;
}

BOARD_RAMFUNC void board_keys_write(uint32_t w)
{
    spi_set_mode(BOARD_SPI_MODE_KEYS);
    const uint32_t saved = rx_dma_detach();

    /* U13 (W[23:16]) first: it is the far end of the chain. */
    (void)xchg((uint16_t)((w >> 16) & 0xFFu));
    (void)xchg((uint16_t)((w >> 8) & 0xFFu));
    (void)xchg((uint16_t)(w & 0xFFu));
    wait_idle();
    board_delay_100ns();
    gpio_write(BOARD_SIG_RCLK, 1u);
    board_delay_100ns();
    gpio_write(BOARD_SIG_RCLK, 0u);

    spi_set_mode(BOARD_SPI_MODE_ADC);
    rx_dma_restore(saved);
}
