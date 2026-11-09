/**
  ******************************************************************************
  * @file    ads8688.h
  * @brief   ADS8688 command / register codes and the blocking start-up driver.
  *
  *  Checked against TI SBAS582C (TZ section 13, item 1):
  *   - Command register (first 16 SCLKs): NO_OP 0x0000, STDBY 0x8200,
  *     PWR_DN 0x8300, RST 0x8500, AUTO_RST 0xA000, MAN_Ch_n 0xC000 + n·0x0400
  *     (Ch0 0xC000 .. Ch7 0xDC00), MAN_AUX 0xE000.
  *   - Program register access: word = addr[15:9] | WR[8] | data[7:0].
  *     A read (WR = 0) returns the register byte on SDO in the 8 SCLKs after
  *     the command, i.e. bits 23..16 of the 32-bit frame = high byte of the
  *     second received halfword.
  *   - Registers: 0x01 AUTO_SEQ_EN, 0x02 channel power-down, 0x03 feature
  *     select (default 0: SDO carries the 16 data bits only), 0x05..0x0C range
  *     select Ch0..Ch7. Range codes 0x00 ±2.5·VREF, 0x01 ±1.25·VREF,
  *     0x02 ±0.625·VREF, 0x05 0..2.5·VREF, 0x06 0..1.25·VREF. Power-on
  *     default 0x00 on every channel.
  *   - Conversion: started by the /CS falling edge for the channel selected
  *     in the PREVIOUS frame; the 16-bit straight-binary result is shifted out
  *     on SCLK 17..32 of the same frame (bipolar 0x8000 = 0 V).
  *   - Input current (V_IN − V_B)/R_IN, R_IN = 1 MΩ: V_B = 2.25 V (±2.5·VREF),
  *     2.00 V (±1.25), 1.60 V (±0.625), 2.50 V (0..2.5·VREF); the TZ value
  *     2.50 V is used for 0..1.25·VREF.
  *   - The datasheet gives no explicit RST recovery time; ≥ 2 ms is used
  *     (TZ: ≥ 1 ms).
  ******************************************************************************
  */
#ifndef APPLICATION_ADS8688_H
#define APPLICATION_ADS8688_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ADS_CMD_NO_OP       0x0000u
#define ADS_CMD_RST         0x8500u
#define ADS_CMD_MAN(n)      ((uint16_t)(0xC000u | ((uint16_t)(n) << 10)))

#define ADS_REG_FEATURE     0x03u
#define ADS_REG_RANGE(n)    ((uint8_t)(0x05u + (n)))

#define ADS_REG_WRITE(addr, val) ((uint16_t)(((uint16_t)(addr) << 9) | 0x0100u | ((val) & 0xFFu)))
#define ADS_REG_READ(addr)       ((uint16_t)((uint16_t)(addr) << 9))
/** Register byte from a board_adc_frame() result of a read command. */
#define ADS_REG_DATA(frame)      ((uint8_t)(((frame) >> 8) & 0xFFu))

#define ADS_RST_WAIT_MS     2u

/** RST, wait, set all eight ranges to @p range_code and read them back.
 *  Up to three attempts. Blocking; thread context, before the sequencer. */
bool ads8688_init(uint8_t range_code);

/** Write one range register (ADS channel @p n) and verify by read-back. */
bool ads8688_set_range(uint8_t n, uint8_t range_code);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_ADS8688_H */
