/**
  ******************************************************************************
  * @file    modbus_app.h
  * @brief   Modbus register-map adapter for the 8UAI universal analog-input
  *          module (8 inputs: 0-10 V, 2-10 V, 0/4-20 mA, Pt1000, KTY/table,
  *          NTC, DI).
  *
  *  Float values are IEEE-754 32-bit, transmitted as two 16-bit registers with
  *  the HIGH word first (big-endian word order: register[N] = bits 31..16).
  *  Multi-channel quantities are grouped by quantity: 8 consecutive registers
  *  (or 8 register pairs) are channels AI0..AI7.
  *
  *  ---- Holding Registers (FC03/06/16) — compact per-channel block --------
  *      0..7    reading, int16 (RO): 0.1 °C (RTD/NTC), mV (voltage), µA
  *                (current), 0/1 (DI); physical units, the V/I scale below is
  *                not applied here. −32768 (0x8000) = no valid value.
  *      8..15   mode: 0 OFF, 1 V 0-10, 2 V 2-10, 3 I 0-20, 4 I 4-20,
  *                5 PT1000, 6 KTY (table), 7 RTD table, 8 NTC, 9 DI
  *                (default 1). A change restarts the channel (PRECHECK).
  *      16..23  filter time constant, ms (EMA, 0 = off, default 0)
  *      24..31  line resistance (both wires), mΩ (default 0)
  *      32..39  NTC model: 0 B-parameter, 1 Steinhart-Hart (default 0)
  *      40..47  DI on threshold, Ω  (R below -> 1, default 1000)
  *      48..55  DI off threshold, Ω (R above -> 0, default 5000; > on)
  *      56..63  RTD/KTY table points used, 0..32 (default 0)
  *
  *  ---- Holding Registers — per-channel float32 parameters ----------------
  *      540 + ch*16: +0 Pt R0 Ω (1000), +2 NTC R25 Ω (10000), +4 NTC B K (3950),
  *                   +6/+8/+10 Steinhart-Hart a/b/c, +12 scale low,
  *                   +14 scale high (V/I engineering value at the low/high end
  *                   of the signal range; low == high -> physical units)
  *      1000 + ch*128: RTD/KTY table, +0..63 R[0..31] Ω (float32 pairs),
  *                   +64..127 t[0..31] °C. R strictly increasing over the
  *                   points in use; outside the table -> OUT_OF_TABLE.
  *
  *  ---- Holding Registers — common analog block (service) ------------------
  *      133 mains frequency, Hz (50/60): integration window 20.000 / 16.68 ms
  *      134 T_BLANK, µs (200..10000, default 1500)
  *      700..723 input-node model, float32: 700 V_REF, 702 R_REF, 704 R_DS1,
  *               706 R_SH, 708 R_DS2, 710 R_7, 712 R_IN,
  *               714/716/718/720/722 V_B of ±10.24 / ±5.12 / ±2.56 /
  *               0..10.24 / 0..5.12 V ranges
  *      730..747 diagnostics, float32: RTD open Ω, RTD short Ω, NTC open Ω,
  *               NTC short Ω, V2-10 open V, V over V, V negative V,
  *               I4-20 open mA, I over mA
  *
  *    Writes are staged in the settings and applied at the next phase
  *    boundary once the channel (or common block) is consistent. An FC16
  *    request that leaves it inconsistent is rejected as a whole (exception
  *    3); an FC06 half of a float is staged until the other half arrives.
  *
  *  ---- Input Registers (FC04, read-only) — grouped by quantity ----------
  *      300..315 value, float32 ×8 (°C, V, mA, 0/1 or scaled; NaN if invalid)
  *      316..331 raw, float32 ×8 (R_sensor Ω, V or mA)
  *      332..339 status bits ×8: 0 NO_DATA, 1 OVERRANGE, 2 UNDERRANGE,
  *                 3 WIRE_BREAK, 4 SHORT, 5 OVERLOAD, 6 EXT_VOLTAGE, 7 LOCKED,
  *                 8 NEG_INPUT, 9 ADC_CLIP, 10 OUT_OF_TABLE, 11 CONFIG_ERR,
  *                 12 ADC_FAIL, 13 INTERNAL (key check / range write),
  *                 14 OFF. Any bit except 1/2 -> value NaN.
  *      340..347 update counter ×8 (u16, wraps)
  *      348..355 protection trips since power-up ×8
  *      356..363 channel state ×8: 0 OFF, 1 PRECHECK, 2 READY, 3 RUN,
  *                 4 RETRY, 5 LOCKED, 6 ADC_FAIL
  *    Global:
  *      120 fw major, 121 fw minor, 122/123 uptime s (lo/hi),
  *      125 module id (0x08A7), 126 on-chip temperature (signed 0.1 °C)
  *      127 module flags: bit0 ADC_FAIL, bit1 CONFIG_ERR, bit2 sequencer
  *          running, bit3 25 ms watchdog armed, bit4 internal error
  *      128 samples per channel in the last phase (min over sampled inputs)
  *      129 settings saves since boot (Flash written + read-back), 130 last save
  *      result (0 none, 1 ok, 2 failed)
  *
  *  ---- Holding Registers (FC03/06/16) — global -----------------------------
  *      101 LED mode (0/1/2)                   102 Modbus slave id
  *      103 Modbus TCP port                    104..107 static IP octets
  *      108..111 netmask octets                112..115 gateway octets
  *      116 net mode (0 static/1 DHCP/2 LL)    117 SAVE trigger (0xA5A5)
  *      118 REBOOT (0xB00B) / BOOT (0xB007) / KSZ8863 reset (0x8863)
  *      119 FACTORY RESET trigger (0xDEAD)     130 on-chip temperature (RO)
  *      131 channel command: 0xA100|ch UNLOCK, 0xA200|ch RESTART,
  *          0xA300 APPLY (re-apply the whole stored configuration)
  ******************************************************************************
  */
#ifndef APPLICATION_MODBUS_APP_H
#define APPLICATION_MODBUS_APP_H

#include <stdbool.h>
#include <stdint.h>

#include "nanomodbus.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Magic write triggers (FC06/FC10 to specific holding registers). */
#define MODBUS_TRIG_SAVE            0xA5A5u
#define MODBUS_TRIG_REBOOT          0xB00Bu
#define MODBUS_TRIG_FACTORY_RESET   0xDEADu
#define MODBUS_TRIG_BOOTLOADER      0xB007u
/* Hardware-reset the KSZ8863 Ethernet switch (operator recovery command).
 * Written to MB_HR_TRIG_REBOOT (118). Interrupts pass-through traffic for
 * the duration of the chip reset + port auto-negotiation — use only when the
 * switch shows no signs of life. */
#define MODBUS_TRIG_SWITCH_RESET    0x8863u

/* Channel commands (HR131). */
#define MB_CMD_UNLOCK_BASE          0xA100u
#define MB_CMD_RESTART_BASE         0xA200u
#define MB_CMD_APPLY                0xA300u
#define MB_CMD_CH_MASK              0x00FFu

/* No-init RAM cell shared with the bootloader. */
#define BOOT_REQUEST_FLAG_ADDR      0x2001FFF0u
#define BOOT_REQUEST_MAGIC          0xB007CAFEu

/* ---- Global holding registers ---- */
#define MB_HR_LED_MODE              101u
#define MB_HR_SLAVE_ID              102u
#define MB_HR_TCP_PORT              103u
#define MB_HR_IP_BASE               104u
#define MB_HR_NETMASK_BASE          108u
#define MB_HR_GATEWAY_BASE          112u
#define MB_HR_USE_DHCP              116u
#define MB_HR_TRIG_SAVE             117u
#define MB_HR_TRIG_REBOOT           118u
#define MB_HR_TRIG_FACTORY_RESET    119u
#define MB_HR_TEMPERATURE           130u
#define MB_HR_CH_COMMAND            131u
#define MB_HR_MAINS_HZ              133u
#define MB_HR_T_BLANK_US            134u

/* ---- Global input registers ---- */
#define MB_IR_FW_VER_MAJOR          120u
#define MB_IR_FW_VER_MINOR          121u
#define MB_IR_UPTIME_LO             122u
#define MB_IR_UPTIME_HI             123u
#define MB_IR_MODULE_ID             125u
#define MB_IR_TEMPERATURE           126u
#define MB_IR_MODULE_FLAGS          127u
#define MB_IR_SAMPLES               128u
#define MB_IR_SAVE_COUNT            129u  /* successful settings saves since boot */
#define MB_IR_SAVE_RESULT           130u  /* last save: 0 none, 1 ok, 2 failed    */

#define MB_AI_CHANNELS              8u

/* ---- Compact per-channel block (holding): address = group*8 + ch ---- */
#define MB_HR_CH_BASE               0u
#define MB_HR_CH_GROUP_READING      0u    /* int16, read-only */
#define MB_HR_CH_GROUP_MODE         1u
#define MB_HR_CH_GROUP_FILTER       2u
#define MB_HR_CH_GROUP_RLINE        3u
#define MB_HR_CH_GROUP_NTC_MODEL    4u
#define MB_HR_CH_GROUP_DI_ON        5u
#define MB_HR_CH_GROUP_DI_OFF       6u
#define MB_HR_CH_GROUP_RTD_N        7u
#define MB_HR_CH_GROUPS             8u    /* registers 0..63 */

/* ---- Per-channel float parameters (holding) ---- */
#define MB_HR_CH_FLOAT_BASE         540u
#define MB_HR_CH_FLOAT_STRIDE       16u   /* 8 floats per channel -> 540..667 */
#define MB_HR_CH_FLOAT_COUNT        8u

/* ---- RTD / KTY table (holding) ---- */
#define MB_HR_RTD_BASE              1000u
#define MB_HR_RTD_STRIDE            128u  /* R[32] then t[32] -> 1000..2023 */

/* ---- Common analog block (holding, float32) ---- */
#define MB_HR_MODEL_BASE            700u  /* 12 floats -> 700..723 */
#define MB_HR_MODEL_COUNT           12u
#define MB_HR_DIAG_BASE             730u  /* 9 floats  -> 730..747 */
#define MB_HR_DIAG_COUNT            9u

/* ---- Readings (input registers), grouped by quantity ---- */
#define MB_IR_AI_BASE               300u
#define MB_IR_AI_VALUE              300u  /* float32 ×8 -> 300..315 */
#define MB_IR_AI_RAW                316u  /* float32 ×8 -> 316..331 */
#define MB_IR_AI_STATUS             332u  /* u16 ×8     -> 332..339 */
#define MB_IR_AI_SEQ                340u  /* u16 ×8     -> 340..347 */
#define MB_IR_AI_TRIPS              348u  /* u16 ×8     -> 348..355 */
#define MB_IR_AI_STATE              356u  /* u16 ×8     -> 356..363 */
#define MB_IR_AI_END                364u  /* first address past the block */

/* Module ID (input register 125). */
#define MODULE_ID_08UAI             0x08A7u

/** Initialise the modbus register adapter. Must be called after settings_init(). */
void modbus_app_init(void);

/** Mark a successful modbus transaction (used to drive STAT_LED state). */
void modbus_app_notify_request(void);

/** Get the populated nmbs_callbacks structure for nmbs_server_create(). */
const nmbs_callbacks* modbus_app_get_callbacks(void);

/** Before a settings save: replace staged, inconsistent analog parameters
 *  (e.g. half of a float written by FC06) with the configuration in force,
 *  so an invalid record never reaches Flash. */
void modbus_app_sanitize_staged(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_MODBUS_APP_H */
