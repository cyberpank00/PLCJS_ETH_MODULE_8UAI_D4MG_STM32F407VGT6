# PLCJS_ETH_MODULE_8UAI_D4MG_STM32F407VGT6

Firmware for the PLCJS Ethernet module with 8 universal analog inputs
(`STM32F407VGT6`). Per channel: voltage 0–10 / 2–10 V, current 0–20 / 4–20 mA,
Pt1000, KTY or a user RTD table, NTC, or a resistive digital input.

One isolated 16-bit SAR ADC `ADS8688` measures all channels; three keys per
channel (Q1 excitation, Q2 100 Ω shunt, PD 1 MΩ pull-down) set the mode, driven
by a `TPIC6C595 ×2 + 74HC595` chain on the same SPI bus. Values are published
over `Modbus TCP`; settings live in Flash; factory reset and the Ethernet
bootloader (OTA) are supported.

Sources: `DOC/SpecifyDesign/8UI_TZ_firmware.md` ("8UI rev. 6") and
`DOC/SCH_MAIN_BOARD_8UAI.pdf` (beta, 09.11.2026). Network, Modbus server, PDP
discovery, ARP probe, KSZ8863, LED, button and bootloader/OTA come from `8AIC`
fw 1.6; the analog path is new.

## Hardware

| Block | Description |
|---|---|
| MCU | `STM32F407VGT6` at **160 MHz** (APB2 80 MHz → SPI exactly 10 MHz) |
| ADC | `ADS8688`, manual channel selection, 4.096 V reference |
| Keys | U11 `TPIC6C595` (Q1), U12 `TPIC6C595` (PD), U13 `74HC595` (Q2); `/OE` = nOE OR `MCP101` supervisor |
| Watchdog | `IWDG`: 8 s until the analog start-up, then 25 ms, refreshed only by the protection ISR |

Pins (only in `Application/board/board_pins.h`): SPI1 PA5/PA6/PB5, CS_ADC PA9,
nOE PA8, RCLK PC8, channel LEDs PB15 PB14 PB10 PE15..PE11. Inputs map to ADC
channels `AI0..AI7 → 5,4,3,2,0,1,7,6`.

## Modes

| Code | Mode | ADC range | Phase A (Q1 Q2 PD) | Phase B | Result |
|---:|---|---|---|---|---|
| 0 | OFF | — | 000 | 000 | not sampled |
| 1 | V 0–10 | 0..10.24 V | 001 | 001 | V |
| 2 | V 2–10 | 0..10.24 V | 001 | 001 | V, open < 1.8 V |
| 3 | I 0–20 | ±2.56 V | 010 | 010 | mA |
| 4 | I 4–20 | ±2.56 V | 010 | 010 | mA, open < 3.6 mA |
| 5 | PT1000 | ±2.56 V | 100 "on" | 010 "zero" | °C (IEC 60751) |
| 6 | KTY | ±2.56 V | 100 | 010 | °C, table |
| 7 | RTD table | ±2.56 V | 100 | 010 | °C, table |
| 8 | NTC | ±5.12 V | 100 | 010 | °C (B or Steinhart-Hart) |
| 9 | DI | 0..5.12 V | 100 | 100 | 0/1, 1 / 5 kΩ with hysteresis |

## Acquisition and protection

40 µs slots (TIM2); per slot TIM8 + DMA2 run one 4 µs ADC frame per active
channel, then the protection ISR rewrites the 24 key bits + RCLK. A phase is a
13-slot key step (000 → ≥ 0.5 ms → new combination) + `T_BLANK` (1.5 ms) + an
integration window of exactly 20.000 ms (50 Hz) or 16.68 ms (60 Hz); 500
samples per channel per phase with 8 channels.

Every sample is checked in the priority-0 ISR against limits precomputed as
ADC codes: current > 25 mA (OVERLOAD), shunt current > 1 mA in the "zero" phase,
V_n > V_REF + 50 mV or full scale with Q1 on (EXT_VOLTAGE). A trip sets nOE = 1
at once, clears the key, rewrites the chain and releases nOE. Retry after 1 s
(Q1 modes through PRECHECK), LOCKED after 3 trips in a row, counter reset after
60 s. The ISR, the vector table and the Flash writer run from RAM, so
protection keeps working while a settings save erases Flash.

## Modbus map

The full map is the header comment of `Application/modbus/modbus_app.h`.
float32 = 2 registers, high word first.

- Holding `0..63` (group·8 + channel): int16 reading (0.1 °C / mV / µA / 0-1,
  −32768 invalid), mode, filter ms, line mΩ, NTC model, DI on/off Ω, table points.
- Holding `540 + ch·16`: Pt R0, NTC R25, NTC B, SH a/b/c, scale low/high.
- Holding `1000 + ch·128`: RTD/KTY table R[32], t[32].
- Holding `700..723` node model, `730..747` diagnostic thresholds, `133` mains
  Hz, `134` T_BLANK µs, `131` commands (`0xA100|ch` UNLOCK, `0xA200|ch` RESTART,
  `0xA300` APPLY).
- Input `300..363`: value, raw, status, update counter, trips, state.
- IR120 / IR121 firmware version major / minor (**1 / 1**), IR125 module id
  `0x08A7`, IR127 module flags, IR128 samples per phase, IR129/130 save count /
  result.

Status bits: 0 NO_DATA, 1 OVERRANGE, 2 UNDERRANGE, 3 WIRE_BREAK, 4 SHORT,
5 OVERLOAD, 6 EXT_VOLTAGE, 7 LOCKED, 8 NEG_INPUT, 9 ADC_CLIP, 10 OUT_OF_TABLE,
11 CONFIG_ERR, 12 ADC_FAIL, 13 INTERNAL, 14 OFF; any bit but 1/2 → value NaN.

## Image

| Field | Value |
|---|---|
| `product_id` | `0x504C0807` |
| `hw_revision` | `0x0101` |
| `fw_version` | `0x0101` (fw 1.1) |

Settings: sector 10 (`0x080C0000`), `SETTINGS_MAGIC = 0x08A74A57`, version 1.
Sector 11 is unused.

## Build and tests

```powershell
cmake --preset Debug
cmake --build --preset Debug
cmake -S tests -B build/host-tests -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

Flash via OTA, or for RMA/debug with the bootloader's
`scripts/service_flash.ps1 -AppBin <app.bin>` (plain SWD flashing leaves the
board in the bootloader because of the metadata CRC check).

## Status

Builds and passes the host tests. The hardware acceptance campaign (TZ
section 12) has **not** been run yet.
