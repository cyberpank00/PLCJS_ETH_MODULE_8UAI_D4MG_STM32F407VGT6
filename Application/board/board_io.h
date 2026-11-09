/**
  ******************************************************************************
  * @file    board_io.h
  * @brief   8UAI thin hardware layer of the analog front-end (TZ section 1):
  *          gpio_write(sig, level) and spi_xfer(mode, tx, rx, len) plus the two
  *          atomic bus transactions built on them (ADS8688 frame, 24-bit key
  *          write + RCLK). Everything that the protection ISR can reach is
  *          placed in RAM (BOARD_RAMFUNC) so it keeps running while Flash is
  *          being erased.
  *
  *  Bus rules (TZ section 3): CPOL = 0 always; only CPHA changes, only with
  *  CS = 1 and the bus idle. ADS8688 = mode 1 (CPHA 1), 595 chain = mode 0.
  *  SPI1 at PCLK2/8 = 10.0 MHz (SYSCLK 160 MHz, APB2 80 MHz).
  ******************************************************************************
  */
#ifndef APPLICATION_BOARD_IO_H
#define APPLICATION_BOARD_IO_H

#include <stdint.h>

#include "board_pins.h"
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_RAMFUNC   __attribute__((section(".RamFunc"), noinline))
/* Helpers reachable from RAM code must never be emitted out of line in Flash. */
#define BOARD_INLINE    static inline __attribute__((always_inline))

#define BOARD_GPIO(port)  ((GPIO_TypeDef*)(GPIOA_BASE + (uint32_t)(port) * 0x400u))
#define BOARD_SPI         SPI1

typedef enum {
    BOARD_SIG_CS = 0,    /* ADS8688 /CS          */
    BOARD_SIG_NOE,       /* 1 = all keys open    */
    BOARD_SIG_RCLK,      /* 595 latch clock      */
} board_sig_t;

#define BOARD_SPI_MODE_KEYS   0u   /* CPOL 0, CPHA 0, 8-bit  */
#define BOARD_SPI_MODE_ADC    1u   /* CPOL 0, CPHA 1, 16-bit */

/* No tables / switch here: the ISR runs from RAM while Flash may be busy, so
 * it must not read .rodata (constants below compile to immediates). */
BOARD_INLINE void gpio_write(board_sig_t sig, uint8_t level)
{
    const uint32_t up = level ? 0u : 16u;
    if (sig == BOARD_SIG_CS) {
        BOARD_GPIO(BOARD_CS_PORT)->BSRR = 1u << (BOARD_CS_PIN + up);
    } else if (sig == BOARD_SIG_NOE) {
        BOARD_GPIO(BOARD_NOE_PORT)->BSRR = 1u << (BOARD_NOE_PIN + up);
    } else {
        BOARD_GPIO(BOARD_RCLK_PORT)->BSRR = 1u << (BOARD_RCLK_PIN + up);
    }
}

/* BSRR words, used as constant DMA sources by the sequencer. */
#define BOARD_CS_LOW_BSRR    (1u << (BOARD_CS_PIN + 16u))
#define BOARD_CS_HIGH_BSRR   (1u << BOARD_CS_PIN)

/** ≥ 100 ns busy wait (SYSCLK 160 MHz, 24 cycles + call overhead). */
BOARD_INLINE void board_delay_100ns(void)
{
    for (volatile uint8_t i = 0u; i < 6u; i++) { __NOP(); }
}

/** Configure SPI1 (master, CPOL 0, PCLK2/8) and hand the pins to it. */
void spi_init(void);

/** Switch CPHA / frame size. Waits for the bus to go idle. CS must be 1. */
void spi_set_mode(uint8_t mode);

/** Blocking full-duplex transfer of @p len bytes in @p mode (8-bit frames;
 *  in ADC mode bytes are paired MSB first). @p rx may be NULL. CS is the
 *  caller's business. */
void spi_xfer(uint8_t mode, const uint8_t* tx, uint8_t* rx, uint16_t len);

/** One complete ADS8688 frame: CS low, 16-bit command, 16 bits of data,
 *  CS high, with the TZ pauses. Returns (first halfword << 16) | data. */
uint32_t board_adc_frame(uint16_t cmd);

/** Write the 24-bit key word (U13 byte first) in mode 0, then the RCLK pulse;
 *  leaves the bus in ADC mode. CS stays 1. */
void board_keys_write(uint32_t w);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_BOARD_IO_H */
