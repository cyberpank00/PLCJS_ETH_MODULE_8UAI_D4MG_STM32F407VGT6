/**
  ******************************************************************************
  * @file    board_pins.h
  * @brief   8UAI board file — the ONLY place that knows MCU pins, the SPI
  *          instance and the board wiring tables of the analog front-end
  *          (TZ section 1). Verified against DOC/SCH_MAIN_BOARD_8UAI.pdf,
  *          revision of 12.11.2026 (key outputs pin-swapped during layout):
  *          MCU sheet 6: PA5/PA6/PB5, PA8 OE_MCU, PA9 ADC_CS, PC8 RCLK,
  *          AIx_STAT; sheet 3: AIx_P_ADC -> AIN_nP, SDI -> U11 -> U12 -> U13,
  *          DRAIN/Q -> SRC_k/LOAD_k/SH_k per BOARD_KEY_MAP below. Re-check
  *          whenever the layout changes.
  *
  *  Isolated-side signals (through U6 CA-IS3740HB / U15 CA-IS3721LS):
  *    SCLK  PA5  SPI1_SCK  (AF5)   idle 0 (CPOL = 0 always)
  *    MISO  PA6  SPI1_MISO (AF5)   ADS8688 SDO
  *    MOSI  PB5  SPI1_MOSI (AF5)   ADS8688 SDI and the 595 chain U11->U12->U13
  *    CS    PA9  GPIO               ADS8688 /CS, active 0
  *    nOE   PA8  GPIO (net OE_MCU)  1 = all 24 keys open (ext. 10 k pull-up R45)
  *    RCLK  PC8  GPIO               0->1 latches the 24-bit shift register
  *
  *  Channel status LEDs (same pins as 8AIC): AI0 PB15, AI1 PB14, AI2 PB10,
  *  AI3 PE15, AI4 PE14, AI5 PE13, AI6 PE12, AI7 PE11 (active high).
  *
  *  This header holds only constants and tables (no HAL include), so the host
  *  tests can use the wiring tables too. The pin set-up lives in board_pins.c.
  ******************************************************************************
  */
#ifndef APPLICATION_BOARD_PINS_H
#define APPLICATION_BOARD_PINS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- GPIO port/pin numbers (port index: A=0, B=1, C=2, ... ) -------------- */
#define BOARD_PORT_A        0u
#define BOARD_PORT_B        1u
#define BOARD_PORT_C        2u
#define BOARD_PORT_E        4u

#define BOARD_SCLK_PORT     BOARD_PORT_A
#define BOARD_SCLK_PIN      5u
#define BOARD_MISO_PORT     BOARD_PORT_A
#define BOARD_MISO_PIN      6u
#define BOARD_MOSI_PORT     BOARD_PORT_B
#define BOARD_MOSI_PIN      5u
#define BOARD_SPI_AF        5u

#define BOARD_CS_PORT       BOARD_PORT_A
#define BOARD_CS_PIN        9u
#define BOARD_NOE_PORT      BOARD_PORT_A
#define BOARD_NOE_PIN       8u
#define BOARD_RCLK_PORT     BOARD_PORT_C
#define BOARD_RCLK_PIN      8u

/* ---- ADS8688 channel of each input (TZ section 4) -------------------------
 * Inputs are not wired in order. MAN_Ch_n, the range register (0x05 + n) and
 * the pipeline bookkeeping use ADC_MAP[input] = n; nothing else may. */
static const uint8_t BOARD_ADC_MAP[8] = { 5u, 4u, 3u, 2u, 0u, 1u, 7u, 6u };

/* ---- Key bits in the 24-bit chain word W (TZ section 5) -------------------
 * Chain SDI -> U11 -> U12 -> U13, W sent MSB first:
 *   W[23:16] -> U13 74HC595   Q0..Q7     (sent first)  — Q2 keys (SH_k)
 *   W[15:8]  -> U12 TPIC6C595 DRAIN0..7                — Q1/PD keys (SRC_k/LOAD_k)
 *   W[7:0]   -> U11 TPIC6C595 DRAIN0..7  (sent last)   — Q1/PD keys (SRC_k/LOAD_k)
 * "1" = key on in all three registers. After layout pin swapping
 * (schematic of 12.11.2026) the outputs are:
 *   U11 DRAIN0..7: SRC_7 LOAD_7 SRC_6 LOAD_6 LOAD_4 SRC_4 LOAD_5 SRC_5
 *   U12 DRAIN0..7: SRC_3 LOAD_3 LOAD_2 SRC_2 LOAD_1 SRC_1 LOAD_0 SRC_0
 *   U13 Q0..Q7:    SH_7  SH_0   SH_1   SH_2  SH_3   SH_4  SH_6   SH_5
 * Every bit 0..23 is used exactly once; Q1/PD in 0..15, Q2 in 16..23
 * (checked by the host tests). */
typedef struct {
    uint8_t q1;
    uint8_t q2;
    uint8_t pd;
} board_key_bits_t;

static const board_key_bits_t BOARD_KEY_MAP[8] = {
    /*        Q1   Q2   PD */
    /* AI0 */ { 15u, 17u, 14u },
    /* AI1 */ { 13u, 18u, 12u },
    /* AI2 */ { 11u, 19u, 10u },
    /* AI3 */ {  8u, 20u,  9u },
    /* AI4 */ {  5u, 21u,  4u },
    /* AI5 */ {  7u, 23u,  6u },
    /* AI6 */ {  2u, 22u,  3u },
    /* AI7 */ {  0u, 16u,  1u },
};

/* ---- Channel status LEDs ---------------------------------------------------- */
static const uint8_t BOARD_LED_PORT[8] = {
    BOARD_PORT_B, BOARD_PORT_B, BOARD_PORT_B, BOARD_PORT_E,
    BOARD_PORT_E, BOARD_PORT_E, BOARD_PORT_E, BOARD_PORT_E,
};
static const uint8_t BOARD_LED_PIN[8] = { 15u, 14u, 10u, 15u, 14u, 13u, 12u, 11u };

/* ---- Functions (board_pins.c) ---------------------------------------------- */
/** First statement of main(): nOE = 1, CS = 1, RCLK = 0, SCLK = 0 as outputs.
 *  Output level is written to BSRR before the pin is switched to output, so
 *  nOE never glitches low (TZ section 10, step 1). */
void board_early_pins(void);
/** Hand SCLK/MISO/MOSI to SPI1 (AF5). Call after the SPI is configured and
 *  enabled with CPOL = 0, so SCLK stays low across the switch. */
void board_spi_pins_af(void);
/** Channel status LEDs as outputs, off. */
void board_led_init(void);
void board_led_set(uint8_t ch, uint8_t on);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_BOARD_PINS_H */
