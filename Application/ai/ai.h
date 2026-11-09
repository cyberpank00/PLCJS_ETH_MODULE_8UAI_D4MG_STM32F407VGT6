/**
  ******************************************************************************
  * @file    ai.h
  * @brief   8UAI analog-input driver API (TZ section 11).
  *
  *  Contexts: the sequencer ISR samples, protects, accumulates and switches
  *  phases (seq.c); everything here runs in the AI task (ai_poll) or is a
  *  short, lock-protected accessor for the Modbus task.
  *
  *  Channel life cycle (one state machine per input):
  *    OFF -> PRECHECK (1 phase, keys 000, ±10.24 V: external-voltage check)
  *        -> READY (waiting for its turn: voltage at once, current one by one
  *           5 ms apart, max 4 per phase, then Q1 modes)
  *        -> RUN (NO_DATA until the first full A+B cycle)
  *    RUN --trip--> RETRY (1 s; Q1 modes through PRECHECK) --3 in a row--> LOCKED
  *    LOCKED --UNLOCK / RESTART--> PRECHECK.  60 s without trips resets the
  *    consecutive counter.
  ******************************************************************************
  */
#ifndef APPLICATION_AI_H
#define APPLICATION_AI_H

#include <stdbool.h>
#include <stdint.h>

#include "ai_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Channel state (Modbus IR356..363, diagnostic). */
#define AI_STATE_OFF        0u
#define AI_STATE_PRECHECK   1u
#define AI_STATE_READY      2u
#define AI_STATE_RUN        3u
#define AI_STATE_RETRY      4u
#define AI_STATE_LOCKED     5u
#define AI_STATE_ADC_FAIL   6u

/* Commands (ai_command). */
#define AI_CMD_UNLOCK       1u
#define AI_CMD_RESTART      2u
#define AI_CMD_APPLY        3u   /* re-read every channel + common block from settings */

/* Module flags (Modbus IR127). */
#define AI_MOD_ADC_FAIL     0x0001u
#define AI_MOD_CONFIG_ERR   0x0002u
#define AI_MOD_RUNNING      0x0004u   /* sequencer running              */
#define AI_MOD_WDG_TIGHT    0x0008u   /* 25 ms watchdog armed            */
#define AI_MOD_INTERNAL     0x0010u   /* key check / range write refused */

/* int16 view (holding 0..7): 0.1 °C, mV, µA or 0/1; -32768 = no valid value. */
#define AI_I16_INVALID      ((int16_t)-32768)

typedef struct {
    float    value;        /* engineering value, NaN when not valid           */
    float    raw;          /* R_sensor Ω, V or mA                              */
    uint16_t status;       /* AI_ST_*                                          */
    uint16_t seq;          /* update counter                                   */
    uint16_t trip_count;   /* protection trips since power-up                  */
    uint8_t  state;        /* AI_STATE_*                                       */
    uint8_t  reserved;
} ai_data_t;

/** TZ section 10, steps 2..6: wait >= 800 ms from reset, ADS8688 reset and
 *  range read-back, W = 0 + RCLK, nOE = 0, load configuration, start the
 *  sequencer and arm the 25 ms watchdog. Steps 7..9 continue in ai_poll().
 *  Returns false on ADC_FAIL (nOE stays 1, the module reports the error). */
bool ai_init(void);

/** AI task body, call every ~2 ms: phase results, formulas, state machine,
 *  plans for the sequencer, liveness for the watchdog. */
void ai_poll(void);

/** Validate and queue a channel configuration (applied at a phase boundary;
 *  a mode change restarts the channel through PRECHECK). */
bool ai_set_config(uint8_t ch, const ai_ch_cfg_t* cfg);
/** The configuration currently in force. */
void ai_get_config(uint8_t ch, ai_ch_cfg_t* out);
/** Validate and queue the common block (model, diagnostics, mains, T_BLANK). */
bool ai_set_common(const ai_model_t* m, const ai_diag_t* d, uint16_t mains_hz, uint16_t t_blank_us);
bool ai_common_valid(const ai_model_t* m, const ai_diag_t* d, uint16_t mains_hz, uint16_t t_blank_us);
/** The common block currently in force (or queued). */
void ai_get_common(ai_model_t* m, ai_diag_t* d, uint16_t* mains_hz, uint16_t* t_blank_us);

/** Atomic snapshot of a channel (does not mask the protection ISR). */
void    ai_get(uint8_t ch, ai_data_t* out);
int16_t ai_get_int16(uint8_t ch);

bool     ai_command(uint8_t ch, uint8_t cmd);
uint16_t ai_module_flags(void);
uint16_t ai_samples_per_phase(void);   /* min over sampled channels, last phase */

/** Channel LEDs: on = RUN with data, slow blink = no data yet, fast blink =
 *  fault / locked, off = OFF. Call from a 10 ms tick. */
void ai_led_tick(uint16_t period_ms);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_AI_H */
