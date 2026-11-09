/**
  ******************************************************************************
  * @file    seq.h
  * @brief   8UAI acquisition sequencer + per-sample protection (TZ 7, 9).
  *
  *  Timing (SYSCLK 160 MHz, SPI 10 MHz):
  *   - Slot = 40 µs, clocked by TIM2 (TRGO on update). Each slot TIM8 (one-pulse,
  *     RCR = n-1, slave-triggered by TIM2) produces n ADC frames of 4 µs. Inside
  *     a frame its compare events drive DMA2:
  *         CC1 @ 0.05 µs  Stream2/ch7  GPIOA->BSRR <- CS low
  *         CC2 @ 0.20 µs  Stream3/ch7  SPI1->DR    <- MAN_Ch of the next frame
  *         CC3 @ 0.60 µs  Stream4/ch7  SPI1->DR    <- 0x0000 (data clocks)
  *         CC4 @ 3.75 µs  Stream7/ch7  GPIOA->BSRR <- CS high
  *     and SPI1 RX -> Stream0/ch3 collects 2 halfwords per frame (ring of 2n).
  *   - The Stream0 transfer-complete interrupt (once per slot, priority 0,
  *     above FreeRTOS) is the protection ISR: it checks every sample against
  *     precomputed code limits, writes the 24-bit key word + RCLK in the gap
  *     after the frames (mode 0), accumulates the integration sums, runs the
  *     phase schedule and feeds the watchdog.
  *   - Phase = 13 slots key step (000 -> >= 0.5 ms -> new combination) +
  *     blanking + integration window (500 slots = 20.000 ms at 50 Hz,
  *     417 slots = 16.68 ms at 60 Hz). Phases alternate A / B.
  *
  *  The ISR and everything it calls run from RAM (vector table relocated to
  *  SRAM) and read no Flash constants, so protection keeps working while the
  *  settings sector is being erased.
  *
  *  Thread <-> ISR: the thread submits a complete plan for the next phase
  *  (seq_plan_submit) and collects published phase sums (seq_phase_take).
  *  A protection trip is handled by the ISR alone (nOE = 1 at once, key bit
  *  cleared, rewrite, nOE = 0) and keeps the channel at 000 until the thread
  *  acknowledges the trip generation in a later plan.
  ******************************************************************************
  */
#ifndef APPLICATION_SEQ_H
#define APPLICATION_SEQ_H

#include <stdbool.h>
#include <stdint.h>

#include "ai_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SEQ_SLOT_US          40u
#define SEQ_SLOTS_PER_MS     25u
#define SEQ_KEY_STEP_SLOTS   13u     /* 000 -> new combination: 13 x 40 = 520 µs */
#define SEQ_INT_SLOTS_50HZ   500u    /* 20.000 ms                                */
#define SEQ_INT_SLOTS_60HZ   417u    /* 16.680 ms (16.667 nominal, -0.08 %)      */

/* Trip causes (per channel, seq_trip_cause). */
#define SEQ_TRIP_NONE        0u
#define SEQ_TRIP_OVERLOAD    1u   /* current mode: I > 25 mA or code at a limit   */
#define SEQ_TRIP_EXT_ZERO    2u   /* RTD/NTC "zero" phase: shunt current > 1 mA   */
#define SEQ_TRIP_EXT_Q1      3u   /* Q1 on: V_n > V_REF + 50 mV or code at limit  */
#define SEQ_TRIP_SCALE_Q1    4u   /* Q1 on: ADC full scale (V_REF + 50 mV is not
                                     representable in this range — open wire or
                                     external voltage; see ai.c)                  */

/** Per-sample limit: trip if code < lo or code > hi. {0, 65535} = never. */
typedef struct {
    uint16_t lo;
    uint16_t hi;
    uint8_t  cause;
    uint8_t  reserved;
} seq_guard_t;

/** Everything the ISR needs for the next phases. Applied at a phase boundary. */
typedef struct {
    uint32_t    gen;                  /* plan id, echoed in the phase results   */
    uint16_t    blank_slots;          /* blanking after the key step            */
    uint16_t    int_slots;            /* integration window                     */
    uint8_t     active_mask;          /* inputs sampled (bit = input)           */
    uint8_t     range_code[AI_CHANNELS];      /* ADS8688 range per input         */
    uint8_t     combo[2][AI_CHANNELS];        /* keys in phase A / B             */
    uint16_t    enable_slot[AI_CHANNELS];     /* slot of the new combination,
                                                 >= SEQ_KEY_STEP_SLOTS (stagger) */
    seq_guard_t guard_q1[AI_CHANNELS];        /* applies while Q1 is on          */
    seq_guard_t guard_q2[AI_CHANNELS];        /* applies while Q2 is on          */
    uint8_t     ack_trip[AI_CHANNELS];        /* trip generation acknowledged    */
} seq_plan_t;

/** Integration result of one phase. */
typedef struct {
    uint32_t seq;                     /* phase counter                           */
    uint32_t plan_gen;                /* plan in force                           */
    uint8_t  phase;                   /* 0 = A, 1 = B                            */
    uint8_t  valid;                   /* no trip / nOE / lost slot / stagger     */
    uint8_t  active_mask;
    uint8_t  reserved;
    uint32_t sum[AI_CHANNELS];        /* sum of codes in the window              */
    uint16_t count[AI_CHANNELS];
    uint16_t clip[AI_CHANNELS];       /* samples at 0x0000 / 0xFFFF (whole phase)*/
} seq_phase_t;

/** Configure TIM2/TIM8/DMA2, relocate the vector table, prime the ADC
 *  pipeline and start. Keys must already be at a valid state (W written). */
void seq_start(const seq_plan_t* first);
bool seq_running(void);

/** Queue a plan for the next boundary. False if the previous one is still
 *  pending (try again on the next poll). */
bool seq_plan_submit(const seq_plan_t* p);

/** Copy the newest published phase if its seq differs from @p last_seq. */
bool seq_phase_take(seq_phase_t* out, uint32_t last_seq);

uint8_t  seq_trip_gen(uint8_t ch);
uint8_t  seq_trip_cause(uint8_t ch);
uint8_t  seq_bad_mask(void);        /* channels refused by the key check (sticky) */
uint8_t  seq_range_err_mask(void);  /* range write read-back failures (sticky)    */
uint32_t seq_slots(void);           /* slots since start (40 µs units)            */
uint16_t seq_lost_slots(void);      /* slots lost inside a window (diagnostic)    */

/** Liveness counters for the watchdog (TZ 9): the ISR refreshes the IWDG only
 *  while the AI loop advanced within 100 ms and the housekeeping loop within
 *  1 s, or while a Flash write window is open (max 5 s). */
void seq_alive_ai(void);
void seq_alive_hk(void);
void seq_flash_window(bool on);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_SEQ_H */
