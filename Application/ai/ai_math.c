/**
  ******************************************************************************
  * @file    ai_math.c
  * @brief   8UAI conversion maths — see ai_math.h. Pure C, host tested.
  ******************************************************************************
  */
#include "ai_math.h"

#include <math.h>
#include <string.h>

#define AIM_NAN       (__builtin_nanf(""))
#define AIM_FINITE(x) (__builtin_isfinite(x))
#define AIM_T0_K      273.15f
#define AIM_T25_K     298.15f

/* ---- ranges ------------------------------------------------------------ */
typedef struct {
    uint8_t code;
    uint8_t bipolar;
    float   fs_mul;      /* FS = fs_mul · V_REF */
} range_desc_t;

static const range_desc_t k_ranges[AI_RANGE_COUNT] = {
    [AI_RANGE_PM10] = { 0x00u, 1u, 2.5f   },
    [AI_RANGE_PM5]  = { 0x01u, 1u, 1.25f  },
    [AI_RANGE_PM2]  = { 0x02u, 1u, 0.625f },
    [AI_RANGE_P10]  = { 0x05u, 0u, 2.5f   },
    [AI_RANGE_P5]   = { 0x06u, 0u, 1.25f  },
};

uint8_t aim_range_code(ai_range_t r)            { return k_ranges[r].code; }
float   aim_range_fs(ai_range_t r, float v_ref) { return k_ranges[r].fs_mul * v_ref; }
bool    aim_range_bipolar(ai_range_t r)         { return k_ranges[r].bipolar != 0u; }

float aim_code_to_vp(float code, ai_range_t r, float v_ref)
{
    const float fs = aim_range_fs(r, v_ref);
    return aim_range_bipolar(r) ? (code - 32768.0f) * fs / 32768.0f
                                : code * fs / 65536.0f;
}

uint16_t aim_vp_to_code(float vp, ai_range_t r, float v_ref)
{
    const float fs = aim_range_fs(r, v_ref);
    float c = aim_range_bipolar(r) ? vp * 32768.0f / fs + 32768.0f
                                   : vp * 65536.0f / fs;
    c += 0.5f;
    if (!(c > 0.0f))     { return 0u; }
    if (c >= 65535.0f)   { return 65535u; }
    return (uint16_t)c;
}

/* ---- modes ------------------------------------------------------------- */
ai_range_t aim_mode_range(uint8_t mode)
{
    switch (mode) {
    case AI_MODE_V_0_10:
    case AI_MODE_V_2_10:    return AI_RANGE_P10;
    case AI_MODE_I_0_20:
    case AI_MODE_I_4_20:
    case AI_MODE_PT1000:
    case AI_MODE_KTY:
    case AI_MODE_RTD_TABLE: return AI_RANGE_PM2;
    case AI_MODE_NTC:       return AI_RANGE_PM5;
    case AI_MODE_DI:        return AI_RANGE_P5;
    default:                return AI_RANGE_PM10;
    }
}

uint8_t aim_mode_keys(uint8_t mode, uint8_t phase)
{
    switch (mode) {
    case AI_MODE_V_0_10:
    case AI_MODE_V_2_10:    return AI_KEY_PD;
    case AI_MODE_I_0_20:
    case AI_MODE_I_4_20:    return AI_KEY_Q2;
    case AI_MODE_PT1000:
    case AI_MODE_KTY:
    case AI_MODE_RTD_TABLE:
    case AI_MODE_NTC:       return (phase == 0u) ? AI_KEY_Q1 : AI_KEY_Q2;
    case AI_MODE_DI:        return AI_KEY_Q1;
    default:                return 0u;
    }
}

bool aim_mode_two_phase(uint8_t mode)
{
    return mode == AI_MODE_PT1000 || mode == AI_MODE_KTY ||
           mode == AI_MODE_RTD_TABLE || mode == AI_MODE_NTC;
}
bool aim_mode_uses_q1(uint8_t mode)    { return aim_mode_two_phase(mode) || mode == AI_MODE_DI; }
bool aim_mode_is_current(uint8_t mode) { return mode == AI_MODE_I_0_20 || mode == AI_MODE_I_4_20; }
bool aim_mode_is_voltage(uint8_t mode) { return mode == AI_MODE_V_0_10 || mode == AI_MODE_V_2_10; }
bool aim_mode_is_table(uint8_t mode)   { return mode == AI_MODE_KTY || mode == AI_MODE_RTD_TABLE; }

/* ---- input-node model ---------------------------------------------------- */
static float r_load(const ai_model_t* m) { return m->r_7 + m->r_in; }
static float r_zero(const ai_model_t* m) { return m->r_sh + m->r_ds2; }

float aim_vn(float vp, float vb, const ai_model_t* m)
{
    return vb + (vp - vb) * r_load(m) / m->r_in;
}

float aim_vp_from_vn(float vn, float vb, const ai_model_t* m)
{
    return vb + (vn - vb) * m->r_in / r_load(m);
}

float aim_vp0(float vb, const ai_model_t* m)
{
    return vb - vb * m->r_in / (r_zero(m) + r_load(m));
}

bool aim_rs(float vn, float vb, const ai_model_t* m, float* rs)
{
    const float rx  = m->r_ref + m->r_ds1;
    const float den = (m->v_ref - vn) / rx - (vn - vb) / r_load(m);
    if (!(den > 0.0f)) { return false; }
    *rs = vn / den;
    return true;
}

float aim_current_ma(float vn, float vb, const ai_model_t* m)
{
    return 1000.0f * (vn / r_zero(m) + (vn - vb) / r_load(m));
}

float aim_vn_for_current(float i_ma, float vb, const ai_model_t* m)
{
    const float rl = r_load(m);
    return (i_ma / 1000.0f + vb / rl) / (1.0f / r_zero(m) + 1.0f / rl);
}

uint16_t aim_vn_to_code(float vn, ai_range_t r, const ai_model_t* m)
{
    return aim_vp_to_code(aim_vp_from_vn(vn, m->v_b[r], m), r, m->v_ref);
}

/* ---- physical conversions --------------------------------------------- */
float aim_pt_res(float t, float r0)
{
    float k = 1.0f + AIM_CVD_A * t + AIM_CVD_B * t * t;
    if (t < 0.0f) { k += AIM_CVD_C * (t - 100.0f) * t * t * t; }
    return r0 * k;
}

float aim_pt_temp(float r, float r0)
{
    const float d = AIM_CVD_A * AIM_CVD_A - 4.0f * AIM_CVD_B * (1.0f - r / r0);
    if (!(d >= 0.0f) || !(r0 > 0.0f)) { return AIM_NAN; }
    float t = (-AIM_CVD_A + sqrtf(d)) / (2.0f * AIM_CVD_B);
    if (r >= r0) { return t; }

    /* t < 0: the C term applies — Newton from the quadratic solution. */
    for (int i = 0; i < 20; i++) {
        const float f  = aim_pt_res(t, r0) - r;
        const float fp = r0 * (AIM_CVD_A + 2.0f * AIM_CVD_B * t +
                               AIM_CVD_C * (4.0f * t * t * t - 300.0f * t * t));
        const float dt = f / fp;
        t -= dt;
        if (fabsf(dt) < 1e-5f) { break; }
    }
    return t;
}

bool aim_table_temp(float r, const ai_ch_cfg_t* c, float* t)
{
    const uint8_t n = c->rtd_n;
    if (n < 2u || n > AI_RTD_TABLE_MAX) { return false; }
    if (!(r >= c->rtd_r[0]) || !(r <= c->rtd_r[n - 1u])) { return false; }
    for (uint8_t i = 1u; i < n; i++) {
        if (r <= c->rtd_r[i]) {
            const float r0 = c->rtd_r[i - 1u], r1 = c->rtd_r[i];
            const float t0 = c->rtd_t[i - 1u], t1 = c->rtd_t[i];
            *t = t0 + (r - r0) * (t1 - t0) / (r1 - r0);
            return true;
        }
    }
    return false;
}

float aim_ntc_temp_b(float r, float r25, float b)
{
    if (!(r > 0.0f) || !(r25 > 0.0f) || !(b > 0.0f)) { return AIM_NAN; }
    return 1.0f / (1.0f / AIM_T25_K + logf(r / r25) / b) - AIM_T0_K;
}

float aim_ntc_temp_sh(float r, float a, float b, float c)
{
    if (!(r > 0.0f)) { return AIM_NAN; }
    const float l   = logf(r);
    const float inv = a + b * l + c * l * l * l;
    if (!(inv > 0.0f)) { return AIM_NAN; }
    return 1.0f / inv - AIM_T0_K;
}

/* ---- evaluation ---------------------------------------------------------- */
float aim_scale(const ai_ch_cfg_t* c, float x)
{
    if (c->scale_lo == c->scale_hi) { return x; }
    float s0, s1;
    switch (c->mode) {
    case AI_MODE_V_0_10: s0 = 0.0f; s1 = 10.0f; break;
    case AI_MODE_V_2_10: s0 = 2.0f; s1 = 10.0f; break;
    case AI_MODE_I_0_20: s0 = 0.0f; s1 = 20.0f; break;
    case AI_MODE_I_4_20: s0 = 4.0f; s1 = 20.0f; break;
    default:             return x;
    }
    return c->scale_lo + (x - s0) * (c->scale_hi - c->scale_lo) / (s1 - s0);
}

static void finish(aim_result_t* out, float value)
{
    out->value = ((out->status & (uint16_t)~AI_ST_VALUE_OK_MASK) != 0u) ? AIM_NAN : value;
}

void aim_eval_single(const ai_ch_cfg_t* c, const ai_model_t* m, const ai_diag_t* d,
                     float vp, uint8_t* di_state, aim_result_t* out)
{
    const ai_range_t r  = aim_mode_range(c->mode);
    const float      vb = m->v_b[r];
    const float      vn = aim_vn(vp, vb, m);
    float value = AIM_NAN;
    out->status = 0u;

    switch (c->mode) {
    case AI_MODE_V_0_10:
    case AI_MODE_V_2_10:
        out->raw = vn;
        if (vn > d->v_over_v) { out->status |= AI_ST_OVERRANGE; }
        if (c->mode == AI_MODE_V_2_10) {
            if (vn < d->v210_open_v) { out->status |= AI_ST_WIRE_BREAK; }
        } else if (vn < d->v_neg_v) {
            out->status |= AI_ST_NEG_INPUT;
        }
        value = aim_scale(c, vn);
        break;

    case AI_MODE_I_0_20:
    case AI_MODE_I_4_20: {
        const float i = aim_current_ma(vn, vb, m);
        out->raw = i;
        if (i > d->i_over_ma) { out->status |= AI_ST_OVERRANGE; }
        if (c->mode == AI_MODE_I_4_20 && i < d->i420_open_ma) { out->status |= AI_ST_WIRE_BREAK; }
        value = aim_scale(c, i);
        break;
    }

    case AI_MODE_DI: {
        float rs;
        const float rsen = aim_rs(vn, vb, m, &rs) ? rs - (float)c->r_line_mohm / 1000.0f
                                                  : __builtin_inff();
        out->raw = rsen;
        if (rsen < c->di_r_on)       { *di_state = 1u; }
        else if (rsen > c->di_r_off) { *di_state = 0u; }
        value = (float)*di_state;
        break;
    }

    default:
        out->raw = AIM_NAN;
        out->status |= AI_ST_OFF;
        break;
    }
    finish(out, value);
}

void aim_eval_two_phase(const ai_ch_cfg_t* c, const ai_model_t* m, const ai_diag_t* d,
                        float vp_a, float vp_b, aim_result_t* out)
{
    const ai_range_t r  = aim_mode_range(c->mode);
    const float      vb = m->v_b[r];
    float value = AIM_NAN;
    out->status = 0u;
    out->raw    = AIM_NAN;

    /* Step 2: ADC offset from the "zero" phase, removed from the "on" phase. */
    const float vos = vp_b - aim_vp0(vb, m);
    const float vn  = aim_vn(vp_a - vos, vb, m);

    float rs;
    if (!aim_rs(vn, vb, m, &rs)) {
        out->status |= AI_ST_WIRE_BREAK;
        finish(out, value);
        return;
    }
    const float rsen = rs - (float)c->r_line_mohm / 1000.0f;
    out->raw = rsen;

    const bool ntc = (c->mode == AI_MODE_NTC);
    if (rs > (ntc ? d->ntc_open_ohm : d->rtd_open_ohm))       { out->status |= AI_ST_WIRE_BREAK; }
    else if (rs < (ntc ? d->ntc_short_ohm : d->rtd_short_ohm)) { out->status |= AI_ST_SHORT; }

    if (c->mode == AI_MODE_PT1000) {
        value = aim_pt_temp(rsen, c->pt_r0);
    } else if (ntc) {
        value = (c->ntc_model == AI_NTC_MODEL_SH)
                    ? aim_ntc_temp_sh(rsen, c->ntc_sh_a, c->ntc_sh_b, c->ntc_sh_c)
                    : aim_ntc_temp_b(rsen, c->ntc_r25, c->ntc_b);
    } else if (!aim_table_temp(rsen, c, &value)) {
        out->status |= AI_ST_OUT_OF_TABLE;
    }
    if (!AIM_FINITE(value) && out->status == 0u) { out->status |= AI_ST_OVERRANGE; }
    finish(out, value);
}

/* ---- defaults / validation ----------------------------------------------- */
void aim_model_defaults(ai_model_t* m)
{
    m->v_ref = 4.096f;
    m->r_ref = 4990.0f;
    m->r_ds1 = 0.8f;
    m->r_sh  = 100.0f;
    m->r_ds2 = 0.075f;
    m->r_7   = 2200.0f;
    m->r_in  = 1.0e6f;
    /* ADS8688 SBAS582C, input current (V_IN - V_B) / R_IN per range. */
    m->v_b[AI_RANGE_PM10] = 2.25f;
    m->v_b[AI_RANGE_PM5]  = 2.00f;
    m->v_b[AI_RANGE_PM2]  = 1.60f;
    m->v_b[AI_RANGE_P10]  = 2.50f;
    m->v_b[AI_RANGE_P5]   = 2.50f;
}

void aim_diag_defaults(ai_diag_t* d)
{
    d->rtd_open_ohm  = 5000.0f;
    d->rtd_short_ohm = 20.0f;
    d->ntc_open_ohm  = 2.0e6f;
    d->ntc_short_ohm = 50.0f;
    d->v210_open_v   = 1.8f;
    d->v_over_v      = 10.2f;
    d->v_neg_v       = -0.1f;
    d->i420_open_ma  = 3.6f;
    d->i_over_ma     = 21.0f;
}

void aim_ch_defaults(ai_ch_cfg_t* c)
{
    memset(c, 0, sizeof(*c));
    c->mode      = AI_MODE_V_0_10;
    c->ntc_model = AI_NTC_MODEL_B;
    c->pt_r0     = 1000.0f;
    c->ntc_r25   = 10000.0f;
    c->ntc_b     = 3950.0f;
    /* Typical 10 kΩ NTC Steinhart-Hart set (used only if ntc_model = SH). */
    c->ntc_sh_a  = 1.009249522e-3f;
    c->ntc_sh_b  = 2.378405444e-4f;
    c->ntc_sh_c  = 2.019202697e-7f;
    c->di_r_on   = 1000.0f;
    c->di_r_off  = 5000.0f;
}

static bool in_range(float v, float lo, float hi) { return AIM_FINITE(v) && v >= lo && v <= hi; }

bool aim_ch_valid(const ai_ch_cfg_t* c)
{
    if (c->mode >= AI_MODE_COUNT || c->ntc_model > AI_NTC_MODEL_SH) { return false; }
    if (!in_range(c->pt_r0, 10.0f, 100000.0f))     { return false; }
    if (!in_range(c->ntc_r25, 1.0f, 1.0e7f))       { return false; }
    if (!in_range(c->ntc_b, 100.0f, 20000.0f))     { return false; }
    if (!AIM_FINITE(c->ntc_sh_a) || !AIM_FINITE(c->ntc_sh_b) || !AIM_FINITE(c->ntc_sh_c)) { return false; }
    if (!in_range(c->di_r_on, 0.0f, 1.0e7f) || !in_range(c->di_r_off, 0.0f, 1.0e7f) ||
        !(c->di_r_off > c->di_r_on))               { return false; }
    if (!AIM_FINITE(c->scale_lo) || !AIM_FINITE(c->scale_hi)) { return false; }
    if (c->rtd_n > AI_RTD_TABLE_MAX)               { return false; }
    for (uint8_t i = 0u; i < c->rtd_n; i++) {
        if (!AIM_FINITE(c->rtd_r[i]) || !AIM_FINITE(c->rtd_t[i])) { return false; }
        if (i > 0u && !(c->rtd_r[i] > c->rtd_r[i - 1u]))         { return false; }
    }
    return true;
}
