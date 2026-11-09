/**
  ******************************************************************************
  * @file    seq.c
  * @brief   8UAI acquisition sequencer + protection ISR (see seq.h).
  *
  *  RAM rule: every function reachable from DMA2_Stream0_IRQHandler carries
  *  BOARD_RAMFUNC / KEYS_RAMFUNC, uses only RAM data (.data/.bss) and no
  *  switch tables or library calls (the file is built with -fno-builtin
  *  -fno-jump-tables). Check the .map after changes: nothing in .text may be
  *  called from here.
  ******************************************************************************
  */
#include "seq.h"

#include <string.h>

#include "ads8688.h"
#include "board_io.h"
#include "keys.h"
#include "stm32f4xx.h"

/* ---- timing constants ------------------------------------------------------ */
#define TIM2_ARR            3199u   /* 80 MHz / 3200 = 25 kHz -> 40 µs slot      */
#define TIM8_ARR            639u    /* 160 MHz / 640 -> 4 µs frame               */
#define TIM8_CC_CS_LOW      8u      /* 0.05 µs                                   */
#define TIM8_CC_CMD         32u     /* 0.20 µs: first halfword into SPI DR       */
#define TIM8_CC_DATA        96u     /* 0.60 µs: second halfword                  */
#define TIM8_CC_CS_HIGH     600u    /* 3.75 µs: >= 150 ns after the last SCLK    */
/* Last TIM2 count at which a key write (~3.5 µs) still ends before the next
 * slot's first CS edge. */
#define KEYS_WRITE_LATEST   2850u
#define SLOT_CYCLES         6400u   /* SYSCLK cycles per slot                    */

/* Transient allowance after a key turns on: for SETTLE_SLOTS a limit violation
 * trips only if the pin voltage is not moving back towards the allowed band
 * (the R7·C2 filter, tau 70..120 µs, lags the node after a switch). */
#define SETTLE_SLOTS        13u
#define SETTLE_MARGIN       64u

#define WDG_AI_SLOTS        (100u  * SEQ_SLOTS_PER_MS)
#define WDG_HK_SLOTS        (1000u * SEQ_SLOTS_PER_MS)
#define WDG_FLASH_SLOTS     (5000u * SEQ_SLOTS_PER_MS)

#define RAMF                BOARD_RAMFUNC

/* ---- DMA sources / buffers (RAM) ------------------------------------------ */
static uint32_t s_cs_low  = BOARD_CS_LOW_BSRR;
static uint32_t s_cs_high = BOARD_CS_HIGH_BSRR;
static uint16_t s_zero;
static uint16_t s_tx_cmd[AI_CHANNELS];
static uint16_t s_rx[2u * AI_CHANNELS];

/* Vector table copy in SRAM: 16 + 82 entries, aligned to its power-of-two size. */
static uint32_t s_vtor[128] __attribute__((aligned(512)));

/* ---- plan / results -------------------------------------------------------- */
static seq_plan_t          s_plan;            /* ISR-owned            */
static seq_plan_t          s_plan_next;       /* thread -> ISR        */
static volatile uint8_t    s_plan_pending;

static seq_phase_t         s_ph[2];
static volatile uint8_t    s_ph_pub;
static volatile uint32_t   s_ph_pub_seq;
static uint8_t             s_ph_w;
static uint32_t            s_phase_seq;

/* ---- ISR state -------------------------------------------------------------- */
static uint8_t  s_adc_map[AI_CHANNELS];       /* RAM copy of BOARD_ADC_MAP     */
static uint8_t  s_list[AI_CHANNELS];          /* input sampled by each frame   */
static uint8_t  s_n;                          /* frames per slot               */
static uint8_t  s_list_mask;
static uint8_t  s_dummy;                      /* nothing active: frame ignored */
static uint8_t  s_list_built;
static uint8_t  s_range_cur[AI_CHANNELS];     /* range programmed in the ADC   */

static uint8_t  s_combo[AI_CHANNELS];         /* shadow for the next write     */
static uint8_t  s_live[AI_CHANNELS];          /* combination last written      */
static uint8_t  s_target[AI_CHANNELS];        /* combination of this phase     */
static uint8_t  s_pending_en;                 /* waiting for enable_slot       */
static uint8_t  s_settle[AI_CHANNELS];
static uint16_t s_prev[AI_CHANNELS];

static uint16_t s_slot;                       /* slot index inside the phase   */
static uint8_t  s_phase;
static uint8_t  s_discard;
static uint8_t  s_noe_restore;
static volatile uint8_t s_running;

static volatile uint8_t s_trip_gen[AI_CHANNELS];
static volatile uint8_t s_trip_cause[AI_CHANNELS];
static volatile uint8_t s_bad;
static volatile uint8_t s_range_err;
static volatile uint32_t s_slots;
static volatile uint16_t s_lost;
static uint32_t s_last_cyc;

static volatile uint32_t s_alive_ai, s_alive_hk;
static uint32_t s_seen_ai, s_seen_hk, s_ai_slot, s_hk_slot;
static volatile uint8_t  s_flash_on;
static volatile uint32_t s_flash_slot;
static uint8_t  s_flash_was;

/* ---- helpers ---------------------------------------------------------------- */
BOARD_INLINE void stream_off(DMA_Stream_TypeDef* s)
{
    s->CR &= ~DMA_SxCR_EN;
    while ((s->CR & DMA_SxCR_EN) != 0u) { }
}

BOARD_INLINE uint16_t win_start(void)
{
    return (uint16_t)(SEQ_KEY_STEP_SLOTS + 1u + s_plan.blank_slots);
}

BOARD_INLINE uint16_t phase_len(void)
{
    return (uint16_t)(win_start() + s_plan.int_slots);
}

RAMF static void phase_clear(seq_phase_t* ph)
{
    ph->valid = 1u;
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        ph->sum[ch] = 0u;
        ph->count[ch] = 0u;
        ph->clip[ch] = 0u;
    }
}

/* Rebuild the frame list, the MAN_Ch ring and the DMA/TIM8 lengths.
 * Only with TIM8 stopped and its trigger disabled. */
RAMF static void rebuild_list(void)
{
    uint8_t n = 0u;
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        if ((s_plan.active_mask & (1u << ch)) != 0u) { s_list[n++] = ch; }
    }
    s_dummy = (n == 0u) ? 1u : 0u;
    if (s_dummy) { s_list[0] = 0u; n = 1u; }
    s_n = n;
    s_list_mask = s_plan.active_mask;
    s_list_built = 1u;

    /* Frame i sends the command for frame i+1; its data is list[i]. */
    for (uint8_t i = 0u; i < n; i++) {
        const uint8_t next = (uint8_t)((i + 1u == n) ? 0u : i + 1u);
        s_tx_cmd[i] = ADS_CMD_MAN(s_adc_map[s_list[next]]);
    }

    stream_off(DMA2_Stream0);
    stream_off(DMA2_Stream2);
    stream_off(DMA2_Stream3);
    stream_off(DMA2_Stream4);
    stream_off(DMA2_Stream7);
    DMA2->LIFCR = 0x0F7D0F7Du;
    DMA2->HIFCR = 0x0F7D0F7Du;
    DMA2_Stream0->NDTR = 2u * n;
    DMA2_Stream3->NDTR = n;
    DMA2_Stream2->NDTR = 1u;
    DMA2_Stream4->NDTR = 1u;
    DMA2_Stream7->NDTR = 1u;
    DMA2_Stream0->CR |= DMA_SxCR_EN;
    DMA2_Stream2->CR |= DMA_SxCR_EN;
    DMA2_Stream3->CR |= DMA_SxCR_EN;
    DMA2_Stream4->CR |= DMA_SxCR_EN;
    DMA2_Stream7->CR |= DMA_SxCR_EN;

    const uint32_t dier = TIM8->DIER;
    TIM8->DIER = 0u;
    TIM8->RCR  = (uint32_t)n - 1u;
    TIM8->EGR  = TIM_EGR_UG;       /* load RCR; no DMA request with DIER = 0 */
    TIM8->SR   = 0u;
    TIM8->DIER = dier;
}

/* Boundary work that needs the bus: key rewrite, range writes with read-back,
 * frame-list rebuild, pipeline prime. Costs one or more slots (outside the
 * integration window). */
RAMF static void polled_reconfig(uint8_t range_chg, uint8_t list_chg)
{
    TIM8->SMCR &= ~TIM_SMCR_SMS;                  /* ignore TIM2 triggers */
    while ((TIM8->CR1 & TIM_CR1_CEN) != 0u) { }   /* let a started slot end */

    uint8_t bad;
    const uint32_t w = keys_build(s_combo, &bad);
    s_bad |= bad;
    board_keys_write(w);
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) { s_live[ch] = s_combo[ch]; }

    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        if ((range_chg & (1u << ch)) == 0u) { continue; }
        const uint8_t n = s_adc_map[ch], code = s_plan.range_code[ch];
        (void)board_adc_frame(ADS_REG_WRITE(ADS_REG_RANGE(n), code));
        const uint32_t f = board_adc_frame(ADS_REG_READ(ADS_REG_RANGE(n)));
        if (ADS_REG_DATA(f) == code) { s_range_cur[ch] = code; }
        else                         { s_range_err |= (uint8_t)(1u << ch); }
    }
    if (list_chg) { rebuild_list(); }

    (void)board_adc_frame(ADS_CMD_MAN(s_adc_map[s_list[0]]));
    s_discard = 1u;
    TIM8->SMCR |= (6u << TIM_SMCR_SMS_Pos);       /* trigger mode again */
}

RAMF static void boundary(void)
{
    if (s_plan_pending) {
        const volatile uint32_t* src = (const volatile uint32_t*)&s_plan_next;
        uint32_t* dst = (uint32_t*)&s_plan;
        for (uint32_t i = 0u; i < sizeof(seq_plan_t) / 4u; i++) { dst[i] = src[i]; }
        s_plan_pending = 0u;
    }

    uint8_t range_chg = 0u;
    s_pending_en = 0u;
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        const uint8_t bit = (uint8_t)(1u << ch);
        uint8_t tgt = s_plan.combo[s_phase][ch];
        if ((s_plan.active_mask & bit) == 0u || s_trip_gen[ch] != s_plan.ack_trip[ch]) { tgt = 0u; }
        s_target[ch] = tgt;
        if (tgt != s_combo[ch]) {
            s_combo[ch] = 0u;                     /* step through 000 */
            if (tgt != 0u) { s_pending_en |= bit; }
        }
        if ((s_plan.active_mask & bit) != 0u && s_plan.range_code[ch] != s_range_cur[ch]) {
            range_chg |= bit;
        }
    }
    const uint8_t list_chg = (!s_list_built || s_plan.active_mask != s_list_mask) ? 1u : 0u;
    if (range_chg != 0u || list_chg != 0u) {
        polled_reconfig(range_chg, list_chg);
    }
}

RAMF static void publish(void)
{
    seq_phase_t* ph = &s_ph[s_ph_w];
    ph->seq         = ++s_phase_seq;
    ph->plan_gen    = s_plan.gen;
    ph->phase       = s_phase;
    ph->active_mask = s_dummy ? 0u : s_list_mask;
    s_ph_pub = s_ph_w;
    __DMB();
    s_ph_pub_seq = ph->seq;
    s_ph_w ^= 1u;
    phase_clear(&s_ph[s_ph_w]);
    s_phase ^= 1u;
}

RAMF static void trip(uint8_t ch, uint8_t key, uint8_t cause)
{
    gpio_write(BOARD_SIG_NOE, 1u);                /* all keys open, no bus needed */
    s_noe_restore = 1u;
    s_combo[ch] &= (uint8_t)~key;
    s_target[ch] = 0u;
    s_pending_en &= (uint8_t)~(1u << ch);
    s_trip_gen[ch]++;
    s_trip_cause[ch] = cause;
    s_ph[s_ph_w].valid = 0u;
}

RAMF static void check_sample(uint8_t ch, uint8_t combo, uint16_t code)
{
    const seq_guard_t* g;
    uint8_t key;
    if ((combo & AI_KEY_Q2) != 0u)      { g = &s_plan.guard_q2[ch]; key = AI_KEY_Q2; }
    else if ((combo & AI_KEY_Q1) != 0u) { g = &s_plan.guard_q1[ch]; key = AI_KEY_Q1; }
    else { s_prev[ch] = code; return; }

    bool bad = false;
    if (code > g->hi) {
        bad = (s_settle[ch] == 0u) || ((uint32_t)code + SETTLE_MARGIN >= s_prev[ch]);
    } else if (code < g->lo) {
        bad = (s_settle[ch] == 0u) || (code <= (uint32_t)s_prev[ch] + SETTLE_MARGIN);
    }
    s_prev[ch] = code;
    if (bad) { trip(ch, key, g->cause); }
}

RAMF static void watchdog(void)
{
    const uint32_t now = s_slots;
    const uint32_t a = s_alive_ai, h = s_alive_hk;
    if (a != s_seen_ai) { s_seen_ai = a; s_ai_slot = now; }
    if (h != s_seen_hk) { s_seen_hk = h; s_hk_slot = now; }
    const uint8_t fl = s_flash_on;
    if (s_flash_was && !fl) { s_ai_slot = now; s_hk_slot = now; }   /* grace after a Flash write */
    s_flash_was = fl;

    const bool alive = ((now - s_ai_slot) < WDG_AI_SLOTS) && ((now - s_hk_slot) < WDG_HK_SLOTS);
    const bool flash = fl && ((now - s_flash_slot) < WDG_FLASH_SLOTS);
    if (alive || flash) { IWDG->KR = 0xAAAAu; }
}

/* ---- the protection ISR ----------------------------------------------------- */
RAMF void DMA2_Stream0_IRQHandler(void)
{
    DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 |
                  DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;
    while ((TIM8->CR1 & TIM_CR1_CEN) != 0u) { }   /* last CS-high written */

    const uint32_t cyc = DWT->CYCCNT;
    const bool lost = (cyc - s_last_cyc) > (SLOT_CYCLES + SLOT_CYCLES / 2u);
    s_last_cyc = cyc;
    s_slots++;

    seq_phase_t* ph = &s_ph[s_ph_w];
    const uint16_t ws = win_start();
    const bool in_win = (s_slot >= ws) && (s_slot < (uint16_t)(ws + s_plan.int_slots));
    if (lost && in_win) { ph->valid = 0u; s_lost++; }

    /* 1. Key rewrite (every slot: refreshes latches, applies changes). The
     *    samples of this slot were taken with the previous combination. */
    uint8_t prev[AI_CHANNELS];
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) { prev[ch] = s_live[ch]; }
    if (TIM2->CNT < KEYS_WRITE_LATEST) {
        uint8_t bad;
        const uint32_t w = keys_build(s_combo, &bad);
        s_bad |= bad;
        board_keys_write(w);
        for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
            if ((s_combo[ch] & ~s_live[ch]) != 0u) { s_settle[ch] = SETTLE_SLOTS; }
            s_live[ch] = s_combo[ch];
        }
        if (s_noe_restore) {
            gpio_write(BOARD_SIG_NOE, 0u);
            s_noe_restore = 0u;
        }
    }

    /* 2. Samples: protection on every one, sums inside the window. */
    if (!s_discard && !s_dummy) {
        for (uint8_t i = 0u; i < s_n; i++) {
            const uint8_t  ch   = s_list[i];
            const uint16_t code = s_rx[2u * i + 1u];
            if (code == 0x0000u || code == 0xFFFFu) { ph->clip[ch]++; }
            check_sample(ch, prev[ch], code);
            if (in_win) {
                ph->sum[ch] += code;
                ph->count[ch]++;
            }
        }
    }
    s_discard = 0u;
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        if (s_settle[ch] != 0u) { s_settle[ch]--; }
    }

    /* 3. Phase schedule. */
    if (s_pending_en != 0u) {
        for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
            const uint8_t bit = (uint8_t)(1u << ch);
            if ((s_pending_en & bit) != 0u && s_slot >= s_plan.enable_slot[ch]) {
                s_combo[ch] = s_target[ch];
                s_pending_en &= (uint8_t)~bit;
                if (s_plan.enable_slot[ch] > SEQ_KEY_STEP_SLOTS) { ph->valid = 0u; }
            }
        }
    }
    if ((uint16_t)(s_slot + 1u) >= phase_len()) {
        publish();
        boundary();
        s_slot = 0u;
    } else {
        s_slot++;
    }

    watchdog();
}

/* ---- thread API ------------------------------------------------------------- */
static void dma_setup(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
    (void)RCC->AHB1ENR;

    const uint32_t ch7 = 7u << DMA_SxCR_CHSEL_Pos, ch3 = 3u << DMA_SxCR_CHSEL_Pos;
    const uint32_t m2p = DMA_SxCR_DIR_0, hi = 2u << DMA_SxCR_PL_Pos, vhi = 3u << DMA_SxCR_PL_Pos;
    const uint32_t w32 = (2u << DMA_SxCR_MSIZE_Pos) | (2u << DMA_SxCR_PSIZE_Pos);
    const uint32_t w16 = (1u << DMA_SxCR_MSIZE_Pos) | (1u << DMA_SxCR_PSIZE_Pos);
    volatile uint32_t* bsrr = &BOARD_GPIO(BOARD_CS_PORT)->BSRR;

    stream_off(DMA2_Stream0); stream_off(DMA2_Stream2); stream_off(DMA2_Stream3);
    stream_off(DMA2_Stream4); stream_off(DMA2_Stream7);

    DMA2_Stream2->CR   = ch7 | hi | w32 | m2p | DMA_SxCR_CIRC;          /* CS low   */
    DMA2_Stream2->PAR  = (uint32_t)bsrr;
    DMA2_Stream2->M0AR = (uint32_t)&s_cs_low;
    DMA2_Stream7->CR   = ch7 | hi | w32 | m2p | DMA_SxCR_CIRC;          /* CS high  */
    DMA2_Stream7->PAR  = (uint32_t)bsrr;
    DMA2_Stream7->M0AR = (uint32_t)&s_cs_high;
    DMA2_Stream3->CR   = ch7 | hi | w16 | m2p | DMA_SxCR_CIRC | DMA_SxCR_MINC;   /* MAN_Ch */
    DMA2_Stream3->PAR  = (uint32_t)&BOARD_SPI->DR;
    DMA2_Stream3->M0AR = (uint32_t)s_tx_cmd;
    DMA2_Stream4->CR   = ch7 | hi | w16 | m2p | DMA_SxCR_CIRC;          /* data clocks */
    DMA2_Stream4->PAR  = (uint32_t)&BOARD_SPI->DR;
    DMA2_Stream4->M0AR = (uint32_t)&s_zero;
    DMA2_Stream0->CR   = ch3 | vhi | w16 | DMA_SxCR_CIRC | DMA_SxCR_MINC | DMA_SxCR_TCIE;   /* RX */
    DMA2_Stream0->PAR  = (uint32_t)&BOARD_SPI->DR;
    DMA2_Stream0->M0AR = (uint32_t)s_rx;
}

static void tim_setup(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_TIM8EN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;

    TIM8->CR1 = 0u; TIM8->CR2 = 0u; TIM8->SMCR = 0u; TIM8->DIER = 0u;
    TIM8->CCMR1 = 0u; TIM8->CCMR2 = 0u; TIM8->CCER = 0u;
    TIM8->PSC  = 0u;
    TIM8->ARR  = TIM8_ARR;
    TIM8->CCR1 = TIM8_CC_CS_LOW;
    TIM8->CCR2 = TIM8_CC_CMD;
    TIM8->CCR3 = TIM8_CC_DATA;
    TIM8->CCR4 = TIM8_CC_CS_HIGH;
    TIM8->RCR  = 0u;
    TIM8->CR1  = TIM_CR1_OPM;
    TIM8->EGR  = TIM_EGR_UG;
    TIM8->SR   = 0u;
    TIM8->SMCR = (1u << TIM_SMCR_TS_Pos) | (6u << TIM_SMCR_SMS_Pos);   /* ITR1 = TIM2 TRGO */
    TIM8->DIER = TIM_DIER_CC1DE | TIM_DIER_CC2DE | TIM_DIER_CC3DE | TIM_DIER_CC4DE;

    TIM2->CR1 = 0u;
    TIM2->PSC = 0u;
    TIM2->ARR = TIM2_ARR;
    TIM2->CR2 = TIM_CR2_MMS_1;                                          /* TRGO = update */
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR  = 0u;
}

void seq_start(const seq_plan_t* first)
{
    /* Vector table into SRAM (the Flash copy is unreadable during an erase). */
    const uint32_t* src = (const uint32_t*)SCB->VTOR;
    for (uint32_t i = 0u; i < 128u; i++) { s_vtor[i] = (i < 98u) ? src[i] : 0u; }
    __disable_irq();
    SCB->VTOR = (uint32_t)s_vtor;
    __DSB();
    __enable_irq();

    keys_init();
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        s_adc_map[ch]   = BOARD_ADC_MAP[ch];
        s_range_cur[ch] = 0x00u;   /* ads8688_init() left every range at 0x00 */
        s_combo[ch] = s_live[ch] = s_target[ch] = 0u;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    dma_setup();
    tim_setup();
    BOARD_SPI->CR2 |= SPI_CR2_RXDMAEN;

    phase_clear(&s_ph[0]);
    phase_clear(&s_ph[1]);
    memcpy(&s_plan, first, sizeof(s_plan));
    s_list_built = 0u;              /* force a list build */
    s_slot = 0u;
    s_phase = 0u;
    boundary();                     /* ranges, frame list, prime */

    NVIC_SetPriority(DMA2_Stream0_IRQn, 0u);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);
    s_last_cyc = DWT->CYCCNT;
    s_running = 1u;
    TIM2->CR1 |= TIM_CR1_CEN;
}

bool seq_running(void) { return s_running != 0u; }

bool seq_plan_submit(const seq_plan_t* p)
{
    if (s_plan_pending) { return false; }
    memcpy(&s_plan_next, p, sizeof(s_plan_next));
    __DMB();
    s_plan_pending = 1u;
    return true;
}

bool seq_phase_take(seq_phase_t* out, uint32_t last_seq)
{
    const uint32_t s1 = s_ph_pub_seq;
    if (s1 == 0u || s1 == last_seq) { return false; }
    memcpy(out, &s_ph[s_ph_pub], sizeof(*out));
    __DMB();
    return (s_ph_pub_seq == s1) && (out->seq == s1);
}

uint8_t  seq_trip_gen(uint8_t ch)   { return s_trip_gen[ch]; }
uint8_t  seq_trip_cause(uint8_t ch) { return s_trip_cause[ch]; }
uint8_t  seq_bad_mask(void)         { return s_bad; }
uint8_t  seq_range_err_mask(void)   { return s_range_err; }
uint32_t seq_slots(void)            { return s_slots; }
uint16_t seq_lost_slots(void)       { return s_lost; }

void seq_alive_ai(void) { s_alive_ai++; }
void seq_alive_hk(void) { s_alive_hk++; }

void seq_flash_window(bool on)
{
    if (on) {
        s_flash_slot = s_slots;
        __DMB();
        s_flash_on = 1u;
    } else {
        s_flash_on = 0u;
    }
}
