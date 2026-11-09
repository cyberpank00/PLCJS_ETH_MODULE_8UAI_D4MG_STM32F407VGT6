# AGENTS.md

Firmware for the PLCJS Ethernet 8UAI module (8 universal analog inputs:
0–10 / 2–10 V, 0/4–20 mA, Pt1000, KTY / RTD table, NTC, DI; one `ADS8688` +
a 24-bit `TPIC6C595 ×2 + 74HC595` key chain on one isolated SPI bus,
STM32F407VGT6, KSZ8863 switch, Modbus TCP). This file is the orientation map
for agents; user-facing documentation lives in `README.md` / `README_EN.md`.

Specification: `DOC/SpecifyDesign/8UI_TZ_firmware.md` (TZ, "8UI rev. 6"),
schematic `DOC/SCH_MAIN_BOARD_8UAI.pdf` + `DOC/SCH_LED_BOARD_8I.pdf`. TZ section
numbers are quoted in the code comments.

Derived from 8AIC fw 1.6 (commit `5dfdaf4` there): every shared subsystem
(network, Modbus server, PDP discovery, ARP probe, KSZ8863, LED, button,
fw_header, nanomodbus) came from there unchanged; the analog path is new.

## Build (CMake)

Toolchain: STM32 Arm Clang (`starm-clang`) from STM32CubeCLT, Ninja.
Toolchain file `cmake/starm-clang.cmake`, presets in `CMakePresets.json`.

```
cmake --preset Debug
cmake --build --preset Debug
```

Release: substitute `Release`. Output:
`build/<preset>/PLCJS_ETH_MODULE_8UAI_D4MG_STM32F407VGT6.{elf,hex,bin,map}`.

Host tests (conversion maths + key builder) need a native gcc — on this
workstation Qt's MinGW (`C:\Qt\Tools\mingw1310_64\bin`):

```
cmake -S tests -B build/host-tests -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

"Verified" without hardware means: both presets build warning-free, host tests
pass, and the RAM check below is clean. The TZ section 12 acceptance campaign
(oscilloscope, calibrators, 24 h run) is the only real verification.

### RAM check after touching the ISR path
Everything reachable from `DMA2_Stream0_IRQHandler` (and `ram_erase_program`)
must live in RAM and must not touch Flash:

```
arm-none-eabi-objdump -d -j .data build/Release/<name>.elf > ram.dis
# every bl/b.w target must be a RAM function; no ".word 0x080....." literals
```

## Repository layout

| Path | Owner | Notes |
|---|---|---|
| `Application/` | hand-written | All real logic. |
| `Core/`, `Drivers/`, `Middlewares/`, `LWIP/`, `cmake/stm32cubemx/` | STM32CubeMX | Regenerated from the `.ioc`. |
| `startup_stm32f407xx.s`, `STM32F407XX_FLASH.ld` | hand-edited | See *Linker*. |
| `tests/` | hand-written | Host tests (`test_ai.c`). |
| `DOC/` | assets | TZ, design notes, schematics. |

**CubeMX regeneration hazard.** Regeneration overwrites `Core/`, `LWIP/` etc.
Hand edits outside `USER CODE` guards: `LWIP/Target/ethernetif.c`,
`lwipopts.h`, the pin block of `Core/Inc/main.h` / `Core/Src/gpio.c` (analog
pins removed — they belong to `board/`), `SystemClock_Config` (PLLN 320),
`configTOTAL_HEAP_SIZE` 20480 (also in the `.ioc`). `board_early_pins()` sits in
`USER CODE BEGIN 1` of `main()`.

## Module map (`Application/`)

| Module | Responsibility |
|---|---|
| `app/` | Orchestrator: boot order, factory reset, network, housekeeping loop, AI and LED tasks. Start here. |
| `board/board_pins.*` | **The only file with MCU pins** (TZ 1): SPI1, CS_ADC PA9, nOE PA8, RCLK PC8, channel LEDs, `BOARD_ADC_MAP`, `BOARD_KEY_MAP`. Checked against the schematic. |
| `board/board_io.*` | Thin hardware layer: `gpio_write`, `spi_xfer`, `spi_set_mode`, one ADS8688 frame, key write + RCLK. RAM-resident. |
| `ads8688/` | Command/register codes (checked against SBAS582C, see header), blocking start-up init + range read-back. |
| `keys/` | Key shadow → 24-bit word, forbidden-combination check (host tested). RAM-resident. |
| `seq/` | **Sequencer + protection ISR.** TIM2 slot clock, TIM8 + DMA2 frame engine, phase schedule, per-sample protection, watchdog refresh. Read `seq.h` first. |
| `ai/ai_math.*` | Conversion maths, TZ 8: node model, CVD, NTC, tables, V/I/DI, diagnostics (host tested). |
| `ai/ai.*` | Channel state machine, start-up (TZ 10), plans for the sequencer, published snapshot, LEDs. |
| `ai/ai_types.h` | Shared types: modes, ranges, status bits, `ai_ch_cfg_t` (stored in Flash). |
| `wdg/` | IWDG policy: 8 s at boot, 25 ms once the sequencer runs. |
| `flash_ram/` | Sector erase/program from RAM inside a critical section. |
| `settings/` | Flash-backed settings (sector 10), CRC32. |
| `modbus/modbus_app.c` | Register map. **Documented in the header comment of `modbus_app.h`.** |
| others | `discovery/`, `addr_probe/`, `net_id/`, `ksz8863/`, `led/`, `button/`, `temp/`, `fw_header/`, `third_party/nanomodbus/` — shared with the family, unchanged from 8AIC. |

## Analog path in one page

- **Clock:** SYSCLK 160 MHz so SPI1 = PCLK2/8 = 10.0 MHz exactly (TZ 3: 10 MHz
  ceiling from the isolator round trip). TIM2 = 80 MHz, TIM8 = 160 MHz.
- **Slot (40 µs):** TIM2 TRGO starts TIM8 (one-pulse, RCR = n−1) → n frames of
  4 µs. CC1 → DMA2 S2 writes CS low (BSRR), CC2 → S3 writes MAN_Ch of the next
  frame to SPI DR, CC3 → S4 writes 0x0000, CC4 → S7 writes CS high; SPI RX →
  S0 (ring 2n, TC once per slot). Frame i sends the command for frame i+1,
  so the data of frame i belongs to `list[i]`.
- **ISR (S0 TC, priority 0, RAM):** wait TIM8 stop → rewrite keys (mode 0,
  only if `TIM2->CNT < KEYS_WRITE_LATEST`) → check every sample against the
  guard of the combination that was live during the slot → sums in the window
  → schedule → watchdog.
- **Phase:** 13 slots key step + `T_BLANK` + 500 (50 Hz) / 417 (60 Hz) slots
  window. Boundary: apply the pending plan, step changed channels through 000,
  range writes / frame-list rebuild with the TIM8 trigger disabled (costs a
  slot or two, outside the window), prime MAN_Ch, discard the next slot.
- **Plans:** the AI task builds a complete `seq_plan_t` (ranges, combinations
  A/B, guards in ADC codes, stagger slots, trip acknowledgements) and
  `seq_plan_submit()`s it; the ISR takes it at the next boundary. Phase sums
  come back through `seq_phase_take()` tagged with the plan generation.
- **Trips:** ISR only: nOE = 1 immediately, key bit cleared, rewrite at the next
  key write, nOE = 0; `trip_gen[ch]++`. The channel stays at 000 until a plan
  carries `ack_trip[ch] == trip_gen[ch]`. The AI task does retries / LOCKED.

## Invariants

### The protection ISR is safety-critical
- It is the **only** shunt protection (no hardware limiter). Never add a path
  that turns Q1/Q2 on outside `seq.c`, and never call `board_keys_write()`
  from a thread once `seq_start()` ran.
- **RAM only.** Functions reachable from the ISR carry `BOARD_RAMFUNC` /
  `KEYS_RAMFUNC`, helpers are `BOARD_INLINE` (always_inline). No `.rodata`
  tables, no `switch` jump tables, no libc calls: `board_io.c`, `keys.c`,
  `seq.c`, `flash_ram.c` are built with `-fno-builtin -fno-jump-tables`.
  Constants the ISR needs are copied to RAM (`keys_init`, `s_adc_map`). The
  vector table is relocated to SRAM in `seq_start()`. Run the RAM check.
- No FreeRTOS calls from the ISR (priority 0 is above
  `configMAX_SYSCALL_INTERRUPT_PRIORITY`).
- The settle allowance (`SETTLE_SLOTS`, 0.5 ms after a key turns on) exists
  because the R7·C2 filter lags the node; it only suppresses a trip while
  the pin voltage is moving back towards the allowed band.
- Open Pt1000/KTY with Q1 on clips the ±2.56 V range, indistinguishable from
  external voltage: guard cause `SEQ_TRIP_SCALE_Q1`; the next PRECHECK decides
  (clean → WIRE_BREAK, not counted towards LOCKED; external → EXT_VOLTAGE,
  counted). This deviates from a literal reading of TZ 9 on purpose.

### Watchdog (TZ 9)
- After `wdg_tighten()` only the ISR refreshes the IWDG, and only while the AI
  task (`seq_alive_ai`, 100 ms) and housekeeping (`seq_alive_hk`, 1 s) advance,
  or while `seq_flash_window(true)` is open (≤ 5 s). Threads call `wdg_kick()`
  (no-op once tight) + `seq_alive_hk()`; never `HAL_IWDG_Refresh` in new code.
- Flash writes go through `flash_ram_write_sector()` only (never the HAL erase
  — it polls `HAL_GetTick`, whose interrupt is masked during the erase).

### Start-up order (TZ 10)
`board_early_pins()` first in `main()` → AI task: ≥ 800 ms → ADS8688 RST +
ranges read-back (ADC_FAIL keeps nOE = 1) → W = 0 + RCLK → nOE = 0 →
`seq_start()` → `wdg_tighten()` → PRECHECK → staggered enable. Q1/Q2 must never
be on before `seq_start()`.

### Single sources of truth
- **Module identity** — `Application/fw_header/fw_header.h`:
  `FW_PRODUCT_ID = 0x504C0807`, `FW_HW_REVISION = 0x0101`,
  `FW_VERSION_VALUE = 0x0101`.
- **Module ID** — `MODULE_ID_08UAI = 0x08A7` (IR125).
- **Register map** — header comment of `modbus_app.h` + `MB_*` constants;
  ModbusTool `build8UAI()` mirrors it.
- **Pins / wiring tables** — `board_pins.h` only.

### Version policy — bump the minor on every change
**Mandatory.** Every change to firmware behaviour ships with `FW_VERSION_VALUE`
incremented by one minor (`0x0101` → `0x0102`). Major only with a
`FW_HW_REVISION` major change. Documentation-only commits need no bump.
Bump checklist: `fw_header.h`, the version rows in `README.md` and
`README_EN.md` (image table + IR120/IR121).

Changelog: every version bump also adds an entry (`DATE -- reason`, appended at the end)
to the shared `CHANGELOG.md` / `CHANGELOG.txt` in the bootloader repo
(`BOOTLOADER_PLCJS_ETH_MODULE_STM32F407VGT6`) - one file for all repos.

### Persistence
- `settings_t` (contains `ai_ch_cfg_t ×8`, `ai_model_t`, `ai_diag_t`) layout is
  frozen; any change needs `SETTINGS_VERSION` bumped (currently 1, magic
  `0x08A74A57`). A mismatch reverts units to defaults **with CONFIG_ERR**;
  a blank sector gives defaults without CONFIG_ERR.
- Settings live in sector 10 @ `0x080C0000`. Sector 11 is unused here.
- Modbus writes are staged in `settings_t` and applied via `ai_set_config` /
  `ai_set_common`; `modbus_app_sanitize_staged()` runs before every save so a
  half-written float never reaches Flash.

### Threading
- AI task (`osPriorityHigh`, 2 ms) owns the state machine; Modbus reads go
  through `ai_get()` (critical section — masks tasks, not the ISR).
- LwIP calls in the tcpip thread; deferred Flash writes/resets in the
  housekeeping loop (`*_take_pending_*()`), as in the family.

## Gotchas
- **SWD flashing alone leaves the board in the bootloader** (metadata CRC
  check since bootloader 1.8); use the bootloader's
  `scripts/service_flash.ps1 -AppBin <app.bin>`.
- ADS8688 register read data is the high byte of the second received halfword
  (`ADS_REG_DATA`). A wrong guess here shows up as ADC_FAIL at boot — check this
  first on a new board.
- Changing CPHA needs SPE = 0; SCLK has a weak pull-down so it stays low
  meanwhile. CPOL is never changed.
- Polled SPI transactions detach `RXDMAEN` (`rx_dma_detach`) so the RX ring
  stays aligned; keep that when adding bus operations.
- Status bit 13 INTERNAL = a forbidden key combination refused by
  `keys_build` or a range read-back failure; it is sticky until reboot.
- Shared gotchas (SET_NET probe, IR129/IR130 save bookkeeping, 16-byte name,
  4-slot Modbus server with RST close, KSZ8863 warm-reset policy, HR118 magics)
  are identical to 8AIC — see that repo's AGENTS.md.

## Linker / memory contract with the bootloader
`FLASH` origin `0x08040000` (256 K), `RAM` length `0x1FFF0` (boot-request cell
at `0x2001FFF0`), `.fw_header` at offset `0x200`; `fw_header_t` byte-identical
to the bootloader's `app_validate.h`. `.RamFunc` is placed in `.data` (copied
to RAM at start-up). OTA acceptance: `product_id` exact + `hw_revision` major.

## Open points (TZ 13)
- Hardware acceptance (TZ 12) not run: SPI timing on the isolated side, TIM8
  compare offsets (`TIM8_CC_*` in `seq.c`), `KEYS_WRITE_LATEST` margin, real
  trip latency, 50 Hz rejection, accuracy.
- R_DS1 (0.8 Ω), T_BLANK (1.5 ms), DI thresholds: TZ defaults, service
  registers, to be confirmed on the prototype.
- V_B of 0..1.25·VREF taken as 2.50 V (TZ); datasheet gives none explicitly.

## Multi-repo workspace

| Repo | Role |
|---|---|
| `PLCJS_ETH_MODULE_8UAI_D4MG_...` | This module — `0x504C0807` / IR125 `0x08A7`. |
| `PLCJS_ETH_MODULE_8AIC_D4MG_...` | 8x 4–20 mA, `0x504C0804` / `0x08AC`. Direct ancestor of the shared subsystems. |
| `PLCJS_ETH_MODULE_4RTD_D4MG_...` | 4x RTD, `0x504C0403` / `0x04D1`. |
| `PLCJS_ETH_MODULE_12DI_D4MG_...` / `12DQ_...` / `8AOC_...` | Other family members. |
| `BOOTLOADER_PLCJS_ETH_MODULE_STM32F407VGT6` | Shared bootloader; `scripts/variants.csv` row `8uai,0x504C0807,0x010101`; shared CHANGELOG. |
| `PLCJS_Module_ModbusTool` | Qt6 client; `build8UAI()` in `src/maps/ModuleMaps.cpp`. |

`Application/` subsystems are copy-pasted between variants, not shared — a fix
here is not a fix elsewhere. Lockstep contracts: PDP wire format, `fw_header_t`
/ flash map / boot-request cell, `product_id` (+ `variants.csv`), register map
(+ `ModuleMaps.cpp`).

## Maintaining this file
Update it in the same commit as the change it describes: invariants (above
all the ISR/RAM and watchdog rules), module map, build/linker contract,
register-map changes, version bumps, cross-repo contracts.
