/**
  ******************************************************************************
  * @file    board_pins.h
  * @brief   8UAI board file — the ONLY place that knows MCU pins, the SPI
  *          instance and the board wiring tables of the analog front-end
  *          (TZ section 1). Source: schematic PLCJS_ETH_MODULE_8UAI_D4MG
  *          (beta, 09.11.2026). Verified against DOC/SCH_MAIN_BOARD_8UAI.pdf
  *          (MCU sheet 6: PA5/PA6/PB5, PA8 OE_MCU, PA9 ADC_CS, PC8 RCLK,
  *          AIx_STAT; ADC sheet 3: AIx_P_ADC -> AIN_nP, SDI -> U11 -> U12 ->
  *          U13, DRAINk/Qk -> SRC_k/LOAD_k/SH_k). Re-check whenever the
  *          layout changes.
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
 * W[23:16] -> U13 74HC595 Qk     -> Q2 (shunt)      of input k (sent first)
 * W[15:8]  -> U12 TPIC6C595 DRk  -> PD (pull-down)  of input k
 * W[7:0]   -> U11 TPIC6C595 DRk  -> Q1 (excitation) of input k (sent last)
 * "1" = key on in all three registers. */
typedef struct {
    uint8_t q1;
    uint8_t q2;
    uint8_t pd;
} board_key_bits_t;

static const board_key_bits_t BOARD_KEY_MAP[8] = {
    { 0u, 16u,  8u }, { 1u, 17u,  9u }, { 2u, 18u, 10u }, { 3u, 19u, 11u },
    { 4u, 20u, 12u }, { 5u, 21u, 13u }, { 6u, 22u, 14u }, { 7u, 23u, 15u },
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
