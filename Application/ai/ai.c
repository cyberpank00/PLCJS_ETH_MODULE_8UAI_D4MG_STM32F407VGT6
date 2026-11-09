/**
  ******************************************************************************
  * @file    ai.c
  * @brief   8UAI analog-input driver: start-up, channel state machine,
  *          sequencer plans, result evaluation (see ai.h).
  ******************************************************************************
  */
#include "ai.h"

#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "ads8688.h"
#include "ai_math.h"
#include "board_io.h"
#include "cmsis_os.h"
#include "seq.h"
#include "settings.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "wdg.h"

#define AI_BOOT_WAIT_MS        800u     /* MCP101 tRPU 700 ms + isolators       */
#define AI_RETRY_MS            1000u
#define AI_CONSEC_RESET_MS     60000u
#define AI_LOCK_TRIPS          3u
#define AI_STAGGER_SLOTS       (5u * SEQ_SLOTS_PER_MS)   /* 5 ms between shunts */
#define AI_MAX_STAGGER         4u
#define AI_WAIT_PENDING        0xFFFFFFFFu

typedef struct {
    ai_ch_cfg_t cfg;
    uint8_t  state;
    uint32_t wait_gen;      /* first plan gen whose phases count for this state */
    uint8_t  ack_trip;      /* trip generation acknowledged to the ISR          */
    uint8_t  seen_trip;     /* trip generation already counted                  */
    uint8_t  consec;
    uint8_t  open_suspect;  /* last trip: full scale with Q1 (open wire?)       */
    uint16_t trip_count;
    uint32_t last_trip_ms;
    uint32_t retry_at_ms;
    uint16_t prot_status;   /* protection status kept until valid data          */
    uint8_t  evaluated;     /* at least one result since RUN                    */
    uint8_t  have_a;
    uint8_t  di_state;
    uint8_t  filt_init;
    uint32_t a_seq;
    float    vp_a;
    float    filt;
    aim_result_t res;
    uint16_t res_seq;
} ch_rt_t;

static ch_rt_t     s_rt[AI_CHANNELS];
static ai_model_t  s_model;
static ai_diag_t   s_diag;
static uint16_t    s_mains_hz;
static uint16_t    s_blank_us;
static uint32_t    s_gen;
static bool        s_dirty;
static uint32_t    s_last_phase;
static bool        s_adc_fail;
static bool        s_cfg_err;
static bool        s_started;
static uint16_t    s_samples;

/* Published snapshot (copied under a FreeRTOS critical section, which masks
 * the tasks but not the priority-0 protection ISR). */
static ai_data_t   s_out[AI_CHANNELS];

/* Requests from other tasks. */
static ai_ch_cfg_t s_pend_cfg[AI_CHANNELS];
static volatile uint8_t s_pend_mask;
static ai_model_t  s_pend_model;
static ai_diag_t   s_pend_diag;
static uint16_t    s_pend_mains, s_pend_blank;
static volatile bool s_pend_common;
static volatile uint8_t s_cmd_unlock, s_cmd_restart;
static volatile bool s_cmd_apply;

/* ---- helpers ------------------------------------------------------------- */
static uint32_t now_ms(void) { return HAL_GetTick(); }

static void enter(ch_rt_t* r, uint8_t state)
{
    r->state     = state;
    r->wait_gen  = AI_WAIT_PENDING;
    r->have_a    = 0u;
    r->evaluated = 0u;
    s_dirty = true;
}

static void restart_channel(ch_rt_t* r)
{
    r->consec       = 0u;
    r->prot_status  = 0u;
    r->open_suspect = 0u;
    r->ack_trip     = r->seen_trip;
    r->filt_init    = 0u;
    r->di_state     = 0u;
    enter(r, (r->cfg.mode == AI_MODE_OFF) ? AI_STATE_OFF : AI_STATE_PRECHECK);
}

static uint16_t phase_ms_x10(void)
{
    const uint32_t slots = SEQ_KEY_STEP_SLOTS + 1u + (s_blank_us + SEQ_SLOT_US - 1u) / SEQ_SLOT_US +
                           ((s_mains_hz == 60u) ? SEQ_INT_SLOTS_60HZ : SEQ_INT_SLOTS_50HZ);
    return (uint16_t)(slots * 10u / SEQ_SLOTS_PER_MS);
}

static seq_guard_t guard(uint16_t lo, uint16_t hi, uint8_t cause)
{
    const seq_guard_t g = { lo, hi, cause, 0u };
    return g;
}

/* Per-sample limits in ADC codes (TZ section 9); the ISR does no maths. */
static void fill_guards(seq_plan_t* p, uint8_t ch, uint8_t mode)
{
    const ai_range_t r = aim_mode_range(mode);
    p->guard_q1[ch] = guard(0u, 0xFFFFu, SEQ_TRIP_NONE);
    p->guard_q2[ch] = guard(0u, 0xFFFFu, SEQ_TRIP_NONE);

    if (aim_mode_is_current(mode)) {
        uint16_t hi = aim_vn_to_code(aim_vn_for_current(AIM_TRIP_I_MA, s_model.v_b[r], &s_model), r, &s_model);
        if (hi > 0xFFFEu) { hi = 0xFFFEu; }
        p->guard_q2[ch] = guard(1u, hi, SEQ_TRIP_OVERLOAD);
    }
    if (aim_mode_two_phase(mode)) {
        uint16_t hi = aim_vn_to_code(AIM_TRIP_ZERO_VN, r, &s_model);
        if (hi > 0xFFFEu) { hi = 0xFFFEu; }
        p->guard_q2[ch] = guard(1u, hi, SEQ_TRIP_EXT_ZERO);
    }
    if (aim_mode_uses_q1(mode)) {
        const uint16_t hi = aim_vn_to_code(s_model.v_ref + AIM_TRIP_Q1_MARGIN_V, r, &s_model);
        p->guard_q1[ch] = (hi >= 0xFFFFu) ? guard(1u, 0xFFFEu, SEQ_TRIP_SCALE_Q1)
                                          : guard(1u, hi, SEQ_TRIP_EXT_Q1);
    }
}

/* Build the next plan. @p promote gets the READY channels switched on by it. */
static void build_plan(seq_plan_t* p, uint8_t* promote)
{
    memset(p, 0, sizeof(*p));
    p->gen         = s_gen + 1u;
    p->blank_slots = (uint16_t)((s_blank_us + SEQ_SLOT_US - 1u) / SEQ_SLOT_US);
    p->int_slots   = (s_mains_hz == 60u) ? SEQ_INT_SLOTS_60HZ : SEQ_INT_SLOTS_50HZ;

    /* Start-up order (TZ 10, step 8): voltage at once, current one by one
     * 5 ms apart, Q1 modes once no current channel is still waiting. */
    uint8_t cur_waiting = 0u;
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        if (s_rt[ch].state == AI_STATE_READY && aim_mode_is_current(s_rt[ch].cfg.mode)) { cur_waiting++; }
    }
    uint8_t cur_on = 0u;
    *promote = 0u;

    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        const ch_rt_t* r = &s_rt[ch];
        const uint8_t mode = r->cfg.mode;
        const uint8_t bit  = (uint8_t)(1u << ch);
        p->ack_trip[ch]    = r->ack_trip;
        p->enable_slot[ch] = SEQ_KEY_STEP_SLOTS;
        fill_guards(p, ch, mode);

        bool run = (r->state == AI_STATE_RUN);
        if (r->state == AI_STATE_READY) {
            if (aim_mode_is_current(mode)) {
                if (cur_on < AI_MAX_STAGGER) {
                    p->enable_slot[ch] = (uint16_t)(SEQ_KEY_STEP_SLOTS + cur_on * AI_STAGGER_SLOTS);
                    cur_on++;
                    run = true;
                }
            } else if (aim_mode_is_voltage(mode) || cur_waiting == 0u) {
                run = true;
            }
            if (run) { *promote |= bit; }
        }

        if (run) {
            p->active_mask   |= bit;
            p->range_code[ch] = aim_range_code(aim_mode_range(mode));
            p->combo[0][ch]   = aim_mode_keys(mode, 0u);
            p->combo[1][ch]   = aim_mode_keys(mode, 1u);
        } else if (r->state == AI_STATE_PRECHECK || r->state == AI_STATE_READY) {
            p->active_mask   |= bit;
            p->range_code[ch] = aim_range_code(AI_RANGE_PM10);
        }
    }
}

/* Called after a plan was accepted by the sequencer. */
static void commit_plan(const seq_plan_t* p, uint8_t promote)
{
    s_gen = p->gen;
    bool more = false;
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        ch_rt_t* r = &s_rt[ch];
        if ((promote & (1u << ch)) != 0u) {
            r->state     = AI_STATE_RUN;
            r->wait_gen  = p->gen;
            r->have_a    = 0u;
            r->evaluated = 0u;
        } else if (r->wait_gen == AI_WAIT_PENDING) {
            r->wait_gen = p->gen;
        }
        if (r->state == AI_STATE_READY) { more = true; }
    }
    s_dirty = more;
}

/* ---- events ----------------------------------------------------------------- */
static void handle_trip(ch_rt_t* r, uint8_t ch, uint32_t t)
{
    const uint8_t g = seq_trip_gen(ch);
    const uint8_t n = (uint8_t)(g - r->seen_trip);
    r->seen_trip = g;
    r->trip_count = (uint16_t)(r->trip_count + n);
    r->last_trip_ms = t;

    if (seq_trip_cause(ch) == SEQ_TRIP_SCALE_Q1) {
        /* Full scale with Q1 on in a range that cannot show V_REF + 50 mV:
         * an open sensor or an external voltage. PRECHECK decides. */
        r->open_suspect = 1u;
        r->prot_status  = AI_ST_WIRE_BREAK;
    } else {
        r->open_suspect = 0u;
        r->consec = (uint8_t)(r->consec + n);
        r->prot_status = (seq_trip_cause(ch) == SEQ_TRIP_OVERLOAD) ? AI_ST_OVERLOAD : AI_ST_EXT_VOLTAGE;
    }
    if (r->consec >= AI_LOCK_TRIPS) {
        enter(r, AI_STATE_LOCKED);
    } else {
        enter(r, AI_STATE_RETRY);
        r->retry_at_ms = t + AI_RETRY_MS;
    }
}

static void precheck_result(ch_rt_t* r, const seq_phase_t* ph, uint8_t ch, uint32_t t)
{
    const float vp = aim_code_to_vp((float)ph->sum[ch] / (float)ph->count[ch], AI_RANGE_PM10, s_model.v_ref);
    const float vn = aim_vn(vp, s_model.v_b[AI_RANGE_PM10], &s_model);
    const bool ext = ph->clip[ch] != 0u || vn > AIM_PRECHECK_HI_V || vn < AIM_PRECHECK_LO_V;

    if (ext && aim_mode_uses_q1(r->cfg.mode)) {
        if (r->open_suspect) {             /* the full-scale trip was external voltage */
            r->open_suspect = 0u;
            r->consec++;
            if (r->consec >= AI_LOCK_TRIPS) { r->prot_status = AI_ST_EXT_VOLTAGE; enter(r, AI_STATE_LOCKED); return; }
        }
        r->prot_status = AI_ST_EXT_VOLTAGE;
        enter(r, AI_STATE_RETRY);
        r->retry_at_ms = t + AI_RETRY_MS;
        return;
    }
    if (r->state == AI_STATE_PRECHECK) {
        if (!r->open_suspect) { r->prot_status &= (uint16_t)~AI_ST_EXT_VOLTAGE; }
        enter(r, AI_STATE_READY);
    }
}

static void publish_result(ch_rt_t* r, const aim_result_t* res)
{
    r->res = *res;
    float v = res->value;
    const uint16_t tau = r->cfg.filter_tau_ms;
    if (tau != 0u && isfinite(v)) {
        const float dt = (float)phase_ms_x10() / 10.0f * (aim_mode_two_phase(r->cfg.mode) ? 2.0f : 1.0f);
        if (!r->filt_init) { r->filt = v; r->filt_init = 1u; }
        else               { r->filt += dt / ((float)tau + dt) * (v - r->filt); }
        v = r->filt;
    } else if (!isfinite(v)) {
        r->filt_init = 0u;
    }
    r->res.value = v;
    r->res_seq++;
    r->evaluated = 1u;
    if ((res->status & (uint16_t)~AI_ST_VALUE_OK_MASK) == 0u) { r->prot_status = 0u; r->open_suspect = 0u; }
}

static void run_result(ch_rt_t* r, const seq_phase_t* ph, uint8_t ch)
{
    if (!ph->valid || ph->count[ch] == 0u) { r->have_a = 0u; return; }
    const ai_range_t rg = aim_mode_range(r->cfg.mode);
    const float vp = aim_code_to_vp((float)ph->sum[ch] / (float)ph->count[ch], rg, s_model.v_ref);
    const bool clip = ph->clip[ch] != 0u;
    aim_result_t res;

    if (aim_mode_two_phase(r->cfg.mode)) {
        if (ph->phase == 0u) {
            r->vp_a = vp; r->a_seq = ph->seq; r->have_a = clip ? 0u : 1u;
            if (!clip) { return; }
            res.value = NAN; res.raw = NAN; res.status = AI_ST_ADC_CLIP;
            publish_result(r, &res);
            return;
        }
        if (!r->have_a || r->a_seq + 1u != ph->seq) { return; }
        aim_eval_two_phase(&r->cfg, &s_model, &s_diag, r->vp_a, vp, &res);
    } else {
        aim_eval_single(&r->cfg, &s_model, &s_diag, vp, &r->di_state, &res);
    }
    if (clip) { res.status |= AI_ST_ADC_CLIP; res.value = NAN; }
    publish_result(r, &res);
}

static void process_phase(const seq_phase_t* ph)
{
    const uint32_t t = now_ms();
    uint16_t min_count = 0xFFFFu;
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        ch_rt_t* r = &s_rt[ch];
        if ((ph->active_mask & (1u << ch)) != 0u && ph->count[ch] != 0u && ph->count[ch] < min_count) {
            min_count = ph->count[ch];
        }
        if (r->wait_gen == AI_WAIT_PENDING || ph->plan_gen < r->wait_gen) { continue; }
        if ((ph->active_mask & (1u << ch)) == 0u) { continue; }

        if (r->state == AI_STATE_PRECHECK || r->state == AI_STATE_READY) {
            if (ph->valid && ph->count[ch] != 0u) { precheck_result(r, ph, ch, t); }
        } else if (r->state == AI_STATE_RUN) {
            run_result(r, ph, ch);
        }
    }
    if (min_count != 0xFFFFu) { s_samples = min_count; }
}

/* ---- requests from other tasks ---------------------------------------------- */
static void take_requests(void)
{
    if (s_cmd_apply) {
        s_cmd_apply = false;
        const settings_t* s = settings_get();
        for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) { (void)ai_set_config(ch, &s->ch[ch]); }
        (void)ai_set_common(&s->model, &s->diag, s->mains_hz, s->t_blank_us);
    }

    static ai_ch_cfg_t cfg[AI_CHANNELS];   /* 2.4 KB: not on the task stack */
    taskENTER_CRITICAL();
    const uint8_t mask = s_pend_mask;
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        if ((mask & (1u << ch)) != 0u) { cfg[ch] = s_pend_cfg[ch]; }
    }
    s_pend_mask = 0u;
    const bool common = s_pend_common;
    if (common) {
        s_model = s_pend_model; s_diag = s_pend_diag;
        s_mains_hz = s_pend_mains; s_blank_us = s_pend_blank;
        s_pend_common = false;
    }
    const uint8_t unlock = s_cmd_unlock, restart = s_cmd_restart;
    s_cmd_unlock = 0u; s_cmd_restart = 0u;
    taskEXIT_CRITICAL();

    if (common) { s_dirty = true; }
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        ch_rt_t* r = &s_rt[ch];
        const uint8_t bit = (uint8_t)(1u << ch);
        if ((mask & bit) != 0u) {
            const bool mode_change = cfg[ch].mode != r->cfg.mode;
            r->cfg = cfg[ch];
            if (mode_change) { restart_channel(r); }
            else             { r->filt_init = 0u; s_dirty = true; }
        }
        if ((restart & bit) != 0u) { restart_channel(r); }
        if ((unlock & bit) != 0u && r->state == AI_STATE_LOCKED) { restart_channel(r); }
    }
}

/* ---- snapshot ------------------------------------------------------------------ */
static void snapshot(void)
{
    ai_data_t out[AI_CHANNELS];
    const uint8_t internal = seq_bad_mask() | seq_range_err_mask();
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        const ch_rt_t* r = &s_rt[ch];
        uint16_t st;
        float value = NAN, raw = NAN;
        uint8_t state = r->state;

        if (s_adc_fail) {
            st = AI_ST_ADC_FAIL | AI_ST_NO_DATA;
            state = AI_STATE_ADC_FAIL;
        } else if (r->state == AI_STATE_OFF) {
            st = AI_ST_OFF;
        } else if (r->state == AI_STATE_LOCKED) {
            st = AI_ST_LOCKED | r->prot_status;
        } else if (r->state == AI_STATE_RUN && r->evaluated) {
            st = r->res.status;
            value = r->res.value;
            raw = r->res.raw;
        } else {
            st = AI_ST_NO_DATA | r->prot_status;
        }
        if (s_cfg_err) { st |= AI_ST_CONFIG_ERR; }
        if ((internal & (1u << ch)) != 0u) { st |= AI_ST_INTERNAL; }
        if ((st & (uint16_t)~AI_ST_VALUE_OK_MASK) != 0u) { value = NAN; }

        out[ch].value      = value;
        out[ch].raw        = raw;
        out[ch].status     = st;
        out[ch].seq        = r->res_seq;
        out[ch].trip_count = r->trip_count;
        out[ch].state      = state;
        out[ch].reserved   = 0u;
    }
    taskENTER_CRITICAL();
    memcpy(s_out, out, sizeof(s_out));
    taskEXIT_CRITICAL();
}

/* ---- public API -------------------------------------------------------------- */
bool ai_init(void)
{
    const settings_t* s = settings_get();
    s_cfg_err  = settings_config_error();
    s_model    = s->model;
    s_diag     = s->diag;
    s_mains_hz = s->mains_hz;
    s_blank_us = s->t_blank_us;
    if (!ai_common_valid(&s_model, &s_diag, s_mains_hz, s_blank_us)) {
        aim_model_defaults(&s_model);
        aim_diag_defaults(&s_diag);
        s_mains_hz = SETTINGS_DEF_MAINS_HZ;
        s_blank_us = SETTINGS_DEF_T_BLANK_US;
        s_cfg_err  = true;
    }
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        memset(&s_rt[ch], 0, sizeof(s_rt[ch]));
        s_rt[ch].cfg = s->ch[ch];
        if (!aim_ch_valid(&s_rt[ch].cfg)) { aim_ch_defaults(&s_rt[ch].cfg); s_cfg_err = true; }
    }

    board_led_init();
    spi_init();

    /* Step 2: >= 800 ms from power-up (MCP101 holds the keys open up to 700 ms). */
    while (HAL_GetTick() < AI_BOOT_WAIT_MS) {
        wdg_kick();
        osDelay(10u);
    }

    /* Step 3: ADS8688 reset, all ranges ±2.5·VREF, read-back (3 attempts). */
    if (!ads8688_init(aim_range_code(AI_RANGE_PM10))) {
        s_adc_fail = true;
        snapshot();
        return false;
    }

    /* Step 4: valid W (all keys off) latched, then enable the outputs. */
    board_keys_write(0u);
    gpio_write(BOARD_SIG_NOE, 0u);

    /* Steps 5..6: configuration loaded above; every channel starts with a
     * PRECHECK phase; then the sequencer + protection and the watchdog. */
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) { restart_channel(&s_rt[ch]); }
    seq_plan_t plan;
    uint8_t promote;
    build_plan(&plan, &promote);
    seq_start(&plan);
    commit_plan(&plan, promote);
    s_started = true;
    wdg_tighten();
    snapshot();
    return true;
}

void ai_poll(void)
{
    seq_alive_ai();
    if (!s_started) { snapshot(); return; }

    take_requests();

    seq_phase_t ph;
    if (seq_phase_take(&ph, s_last_phase)) {
        s_last_phase = ph.seq;
        process_phase(&ph);
    }

    const uint32_t t = now_ms();
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        ch_rt_t* r = &s_rt[ch];
        if (seq_trip_gen(ch) != r->seen_trip) { handle_trip(r, ch, t); }
        if (r->state == AI_STATE_RETRY && (int32_t)(t - r->retry_at_ms) >= 0) {
            r->ack_trip = r->seen_trip;
            enter(r, aim_mode_uses_q1(r->cfg.mode) ? AI_STATE_PRECHECK : AI_STATE_READY);
        }
        if (r->consec != 0u && (t - r->last_trip_ms) >= AI_CONSEC_RESET_MS) { r->consec = 0u; }
    }

    if (s_dirty) {
        seq_plan_t plan;
        uint8_t promote;
        build_plan(&plan, &promote);
        if (seq_plan_submit(&plan)) { commit_plan(&plan, promote); }
    }
    snapshot();
}

bool ai_set_config(uint8_t ch, const ai_ch_cfg_t* cfg)
{
    if (ch >= AI_CHANNELS || !aim_ch_valid(cfg)) { return false; }
    taskENTER_CRITICAL();
    s_pend_cfg[ch] = *cfg;
    s_pend_mask |= (uint8_t)(1u << ch);
    taskEXIT_CRITICAL();
    return true;
}

void ai_get_config(uint8_t ch, ai_ch_cfg_t* out)
{
    if (ch >= AI_CHANNELS) { return; }
    taskENTER_CRITICAL();
    *out = ((s_pend_mask & (1u << ch)) != 0u) ? s_pend_cfg[ch] : s_rt[ch].cfg;
    taskEXIT_CRITICAL();
}

static bool fin_pos(float v) { return isfinite(v) && v > 0.0f; }

bool ai_common_valid(const ai_model_t* m, const ai_diag_t* d, uint16_t mains_hz, uint16_t t_blank_us)
{
    if (mains_hz != 50u && mains_hz != 60u) { return false; }
    if (t_blank_us < SETTINGS_T_BLANK_US_MIN || t_blank_us > SETTINGS_T_BLANK_US_MAX) { return false; }
    if (!(m->v_ref > 3.0f && m->v_ref < 5.0f)) { return false; }
    if (!fin_pos(m->r_ref) || !isfinite(m->r_ds1) || m->r_ds1 < 0.0f || !fin_pos(m->r_sh) ||
        !isfinite(m->r_ds2) || m->r_ds2 < 0.0f || !isfinite(m->r_7) || m->r_7 < 0.0f || !fin_pos(m->r_in)) {
        return false;
    }
    for (uint8_t i = 0u; i < AI_RANGE_COUNT; i++) {
        if (!isfinite(m->v_b[i]) || m->v_b[i] < 0.0f || m->v_b[i] > 5.0f) { return false; }
    }
    const float* f = &d->rtd_open_ohm;
    for (uint8_t i = 0u; i < sizeof(*d) / sizeof(float); i++) {
        if (!isfinite(f[i])) { return false; }
    }
    return true;
}

bool ai_set_common(const ai_model_t* m, const ai_diag_t* d, uint16_t mains_hz, uint16_t t_blank_us)
{
    if (!ai_common_valid(m, d, mains_hz, t_blank_us)) { return false; }
    taskENTER_CRITICAL();
    s_pend_model = *m;
    s_pend_diag  = *d;
    s_pend_mains = mains_hz;
    s_pend_blank = t_blank_us;
    s_pend_common = true;
    taskEXIT_CRITICAL();
    return true;
}

void ai_get_common(ai_model_t* m, ai_diag_t* d, uint16_t* mains_hz, uint16_t* t_blank_us)
{
    taskENTER_CRITICAL();
    if (s_pend_common) {
        *m = s_pend_model; *d = s_pend_diag; *mains_hz = s_pend_mains; *t_blank_us = s_pend_blank;
    } else {
        *m = s_model; *d = s_diag; *mains_hz = s_mains_hz; *t_blank_us = s_blank_us;
    }
    taskEXIT_CRITICAL();
}

void ai_get(uint8_t ch, ai_data_t* out)
{
    if (ch >= AI_CHANNELS) { memset(out, 0, sizeof(*out)); return; }
    taskENTER_CRITICAL();
    *out = s_out[ch];
    taskEXIT_CRITICAL();
}

int16_t ai_get_int16(uint8_t ch)
{
    ai_data_t d;
    ai_get(ch, &d);
    if ((d.status & (uint16_t)~AI_ST_VALUE_OK_MASK) != 0u || !isfinite(d.value)) { return AI_I16_INVALID; }
    const uint8_t mode = s_rt[ch].cfg.mode;
    float v;
    if (aim_mode_is_voltage(mode) || aim_mode_is_current(mode)) { v = d.raw * 1000.0f; }   /* mV / µA */
    else if (mode == AI_MODE_DI)                                { v = d.value; }
    else                                                        { v = d.value * 10.0f; }  /* 0.1 °C  */
    if (!isfinite(v)) { return AI_I16_INVALID; }
    if (v > 32767.0f)  { return 32767; }
    if (v < -32767.0f) { return -32767; }
    return (int16_t)lrintf(v);
}

bool ai_command(uint8_t ch, uint8_t cmd)
{
    if (cmd == AI_CMD_APPLY) { s_cmd_apply = true; return true; }
    if (ch >= AI_CHANNELS) { return false; }
    taskENTER_CRITICAL();
    if (cmd == AI_CMD_UNLOCK)       { s_cmd_unlock  |= (uint8_t)(1u << ch); }
    else if (cmd == AI_CMD_RESTART) { s_cmd_restart |= (uint8_t)(1u << ch); }
    taskEXIT_CRITICAL();
    return cmd == AI_CMD_UNLOCK || cmd == AI_CMD_RESTART;
}

uint16_t ai_module_flags(void)
{
    uint16_t f = 0u;
    if (s_adc_fail)            { f |= AI_MOD_ADC_FAIL; }
    if (s_cfg_err)             { f |= AI_MOD_CONFIG_ERR; }
    if (seq_running())         { f |= AI_MOD_RUNNING; }
    if (wdg_is_tight())        { f |= AI_MOD_WDG_TIGHT; }
    if ((seq_bad_mask() | seq_range_err_mask()) != 0u) { f |= AI_MOD_INTERNAL; }
    return f;
}

uint16_t ai_samples_per_phase(void) { return s_samples; }

void ai_led_tick(uint16_t period_ms)
{
    static uint16_t t;
    t = (uint16_t)((t + period_ms) % 1000u);
    const bool slow = t < 500u;
    const bool fast = (t % 250u) < 125u;
    for (uint8_t ch = 0u; ch < AI_CHANNELS; ch++) {
        ai_data_t d;
        ai_get(ch, &d);
        bool on;
        if (d.state == AI_STATE_OFF)                                        { on = false; }
        else if ((d.status & (uint16_t)~(AI_ST_NO_DATA | AI_ST_VALUE_OK_MASK)) != 0u) { on = fast; }
        else if ((d.status & AI_ST_NO_DATA) != 0u)                         { on = slow; }
        else                                                               { on = true; }
        board_led_set(ch, on ? 1u : 0u);
    }
}
