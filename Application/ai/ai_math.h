/**
  ******************************************************************************
  * @file    ai_math.h
  * @brief   8UAI conversion maths (TZ sections 4, 6, 8). Pure C, no HAL:
  *          built into the firmware and into the host tests (tests/).
  *
  *  Signal path:  ADC code -> V_p (pin voltage, aim_code_to_vp)
  *                -> V_n (node voltage, full input-node model, aim_vn)
  *                -> R_S / I / V (aim_rs, aim_current_ma)
  *                -> physical value + diagnostics (aim_eval_*).
  *
  *  Everything here runs in thread context (float, logf, sqrtf). The
  *  protection ISR never calls it; thresholds for the ISR are converted to
  *  ADC codes up front with aim_vn_to_code().
  ******************************************************************************
  */
#ifndef APPLICATION_AI_MATH_H
#define APPLICATION_AI_MATH_H

#include <stdbool.h>
#include <stdint.h>

#include "ai_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* IEC 60751 Callendar-Van Dusen coefficients. */
#define AIM_CVD_A   3.9083e-3f
#define AIM_CVD_B  (-5.775e-7f)
#define AIM_CVD_C  (-4.183e-12f)

/* Per-sample protection limits (TZ section 9). */
#define AIM_TRIP_I_MA         25.0f   /* current mode, Q2 on                */
#define AIM_TRIP_ZERO_VN      0.1f    /* RTD/NTC "zero" phase: >1 mA shunt  */
#define AIM_TRIP_Q1_MARGIN_V  0.05f   /* Q1 on: V_n > V_REF + 50 mV         */
#define AIM_PRECHECK_HI_V     4.5f    /* PRECHECK, Hi-Z node                */
#define AIM_PRECHECK_LO_V    (-0.5f)

/* ---- ranges ------------------------------------------------------------ */
uint8_t aim_range_code(ai_range_t r);              /* ADS8688 range register value */
float   aim_range_fs(ai_range_t r, float v_ref);   /* upper full-scale bound, V    */
bool    aim_range_bipolar(ai_range_t r);
/** Code (may be a fractional average) -> pin voltage. Straight binary:
 *  bipolar V = (code - 32768)·FS/32768, unipolar V = code·FS/65536. */
float    aim_code_to_vp(float code, ai_range_t r, float v_ref);
/** Pin voltage -> nearest code, clamped to 0..65535. */
uint16_t aim_vp_to_code(float vp, ai_range_t r, float v_ref);

/* ---- modes ------------------------------------------------------------- */
ai_range_t aim_mode_range(uint8_t mode);
/** Key combination (AI_KEY_*) of @p mode in phase 0 (A) or 1 (B). */
uint8_t aim_mode_keys(uint8_t mode, uint8_t phase);
bool aim_mode_two_phase(uint8_t mode);   /* RTD/NTC: "on" and "zero" phases */
bool aim_mode_uses_q1(uint8_t mode);     /* RTD/NTC/DI: needs PRECHECK      */
bool aim_mode_is_current(uint8_t mode);
bool aim_mode_is_voltage(uint8_t mode);
bool aim_mode_is_table(uint8_t mode);    /* KTY / RTD_TABLE                 */

/* ---- input-node model (TZ section 8) ------------------------------------ */
/** Step 1: node voltage from pin voltage. R_L = R_7 + R_IN. */
float aim_vn(float vp, float vb, const ai_model_t* m);
/** Inverse of step 1. */
float aim_vp_from_vn(float vn, float vb, const ai_model_t* m);
/** Step 2: expected phase-B ("zero", shunt on) pin voltage without ADC offset. */
float aim_vp0(float vb, const ai_model_t* m);
/** Step 3: sensor-loop resistance R_S from the node voltage. Returns false if
 *  the denominator is <= 0 (open circuit). */
bool  aim_rs(float vn, float vb, const ai_model_t* m, float* rs);
/** Loop current in current mode, mA. */
float aim_current_ma(float vn, float vb, const ai_model_t* m);
/** Node voltage at which the current-mode loop current equals @p i_ma. */
float aim_vn_for_current(float i_ma, float vb, const ai_model_t* m);
/** Node voltage -> ADC code in range @p r (protection thresholds). */
uint16_t aim_vn_to_code(float vn, ai_range_t r, const ai_model_t* m);

/* ---- physical conversions --------------------------------------------- */
float aim_pt_res(float t, float r0);                 /* CVD forward, Ω  */
float aim_pt_temp(float r, float r0);                /* CVD inverse, °C */
bool  aim_table_temp(float r, const ai_ch_cfg_t* c, float* t);
float aim_ntc_temp_b(float r, float r25, float b);
float aim_ntc_temp_sh(float r, float a, float b, float c);

/* ---- evaluation ---------------------------------------------------------- */
typedef struct {
    float    value;    /* engineering value (°C, V, mA or scaled, 0/1); NaN if invalid */
    float    raw;      /* R_sensor Ω, V or mA                                           */
    uint16_t status;   /* AI_ST_* result bits                                           */
} aim_result_t;

/** One-phase modes (V, I, DI): @p vp = phase-average pin voltage.
 *  @p di_state carries the DI hysteresis state between calls. */
void aim_eval_single(const ai_ch_cfg_t* c, const ai_model_t* m, const ai_diag_t* d,
                     float vp, uint8_t* di_state, aim_result_t* out);
/** RTD/NTC: @p vp_a = "on" phase average, @p vp_b = "zero" phase average. */
void aim_eval_two_phase(const ai_ch_cfg_t* c, const ai_model_t* m, const ai_diag_t* d,
                        float vp_a, float vp_b, aim_result_t* out);

/** Linear V/I engineering scaling (scale_lo == scale_hi -> identity). */
float aim_scale(const ai_ch_cfg_t* c, float x);

/** Defaults (TZ tables). */
void aim_model_defaults(ai_model_t* m);
void aim_diag_defaults(ai_diag_t* d);
void aim_ch_defaults(ai_ch_cfg_t* c);
/** Range/consistency check of a channel configuration. */
bool aim_ch_valid(const ai_ch_cfg_t* c);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_AI_MATH_H */
