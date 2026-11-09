/* Host tests for the 8UAI conversion maths (ai_math.c) and key builder
 * (keys.c). Reference values: IEC 60751 tables, TZ section 8 examples, and an
 * independent forward model of the input node computed in double. */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ai_math.h"
#include "board_pins.h"
#include "keys.h"

static int g_fail;
static int g_checks;

#define CHECK(cond) do { g_checks++; if (!(cond)) { g_fail++; \
    printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b, tol) do { g_checks++; const double _a = (a), _b = (b); \
    if (!(fabs(_a - _b) <= (tol))) { g_fail++; \
    printf("FAIL %s:%d: %s = %.6f, expected %.6f (tol %g)\n", __FILE__, __LINE__, #a, _a, _b, (double)(tol)); } } while (0)

static ai_model_t M;
static ai_diag_t  D;

/* Independent forward model (double): node voltage for sensor R_S in a
 * Q1-on phase, and the resulting pin voltage. */
static double node_vn(double rs, double vb)
{
    const double rx = M.r_ref + M.r_ds1, rl = M.r_7 + M.r_in;
    return (M.v_ref / rx + vb / rl) / (1.0 / rx + 1.0 / rs + 1.0 / rl);
}
static double pin_vp(double vn, double vb)
{
    const double rl = M.r_7 + M.r_in;
    return vb + (vn - vb) * M.r_in / rl;
}

static void test_codes(void)
{
    CHECK_NEAR(aim_code_to_vp(32768.0f, AI_RANGE_PM2, 4.096f), 0.0, 1e-6);
    CHECK_NEAR(aim_code_to_vp(0.0f, AI_RANGE_PM2, 4.096f), -2.56, 1e-6);
    CHECK_NEAR(aim_code_to_vp(32768.0f, AI_RANGE_P10, 4.096f), 5.12, 1e-5);
    CHECK_NEAR(aim_code_to_vp(65535.0f, AI_RANGE_PM10, 4.096f), 10.24 - 10.24 / 32768.0, 1e-5);
    CHECK(aim_vp_to_code(0.0f, AI_RANGE_PM5, 4.096f) == 32768u);
    CHECK(aim_vp_to_code(99.0f, AI_RANGE_PM5, 4.096f) == 65535u);
    CHECK(aim_vp_to_code(-99.0f, AI_RANGE_PM5, 4.096f) == 0u);
    CHECK(aim_vp_to_code(2.5f, AI_RANGE_P5, 4.096f) == 32000u);
    CHECK(aim_range_code(AI_RANGE_PM10) == 0x00u && aim_range_code(AI_RANGE_PM5) == 0x01u &&
          aim_range_code(AI_RANGE_PM2) == 0x02u && aim_range_code(AI_RANGE_P10) == 0x05u &&
          aim_range_code(AI_RANGE_P5) == 0x06u);
}

static void test_modes(void)
{
    CHECK(aim_mode_keys(AI_MODE_V_0_10, 0) == AI_KEY_PD && aim_mode_keys(AI_MODE_V_2_10, 1) == AI_KEY_PD);
    CHECK(aim_mode_keys(AI_MODE_I_4_20, 0) == AI_KEY_Q2 && aim_mode_keys(AI_MODE_I_0_20, 1) == AI_KEY_Q2);
    CHECK(aim_mode_keys(AI_MODE_PT1000, 0) == AI_KEY_Q1 && aim_mode_keys(AI_MODE_PT1000, 1) == AI_KEY_Q2);
    CHECK(aim_mode_keys(AI_MODE_NTC, 0) == AI_KEY_Q1 && aim_mode_keys(AI_MODE_NTC, 1) == AI_KEY_Q2);
    CHECK(aim_mode_keys(AI_MODE_DI, 0) == AI_KEY_Q1 && aim_mode_keys(AI_MODE_DI, 1) == AI_KEY_Q1);
    CHECK(aim_mode_keys(AI_MODE_OFF, 0) == 0u);
    CHECK(aim_mode_range(AI_MODE_NTC) == AI_RANGE_PM5 && aim_mode_range(AI_MODE_KTY) == AI_RANGE_PM2 &&
          aim_mode_range(AI_MODE_DI) == AI_RANGE_P5 && aim_mode_range(AI_MODE_V_2_10) == AI_RANGE_P10 &&
          aim_mode_range(AI_MODE_I_0_20) == AI_RANGE_PM2);
}

static void test_model(void)
{
    /* TZ example: ±2.56 V range, V_B = 1.6 V -> V_p0 ≈ 3.5 mV. */
    const float vp0 = aim_vp0(1.6f, &M);
    CHECK(vp0 > 0.0034f && vp0 < 0.0039f);

    /* Step 1 is the exact inverse of the divider. */
    CHECK_NEAR(aim_vn(aim_vp_from_vn(1.234f, 2.0f, &M), 2.0f, &M), 1.234, 1e-5);

    /* R_S recovered from the forward model across the RTD and NTC spans. */
    const double rs_list[] = { 20.0, 803.0, 1000.0, 1385.0, 2470.0, 10000.0, 336000.0 };
    for (unsigned i = 0; i < sizeof rs_list / sizeof rs_list[0]; i++) {
        const double vb = 2.0;
        const double vn = node_vn(rs_list[i], vb);
        float rs = 0.0f;
        CHECK(aim_rs((float)vn, (float)vb, &M, &rs));
        CHECK_NEAR(rs, rs_list[i], rs_list[i] * 2e-5 + 1e-3);
    }
    float rs;
    CHECK(!aim_rs(4.2f, 2.0f, &M, &rs));     /* above Vex: open */

    /* Current: the 20 mA node voltage maps back to 20 mA. */
    const float vn20 = aim_vn_for_current(20.0f, 1.6f, &M);
    CHECK_NEAR(aim_current_ma(vn20, 1.6f, &M), 20.0, 1e-4);
    CHECK_NEAR(vn20, 0.020 * 100.075, 0.01);
}

static void test_pt(void)
{
    /* IEC 60751 Pt1000 table values. */
    CHECK_NEAR(aim_pt_res(0.0f, 1000.0f), 1000.0, 1e-3);
    CHECK_NEAR(aim_pt_res(100.0f, 1000.0f), 1385.055, 0.01);
    CHECK_NEAR(aim_pt_res(-100.0f, 1000.0f), 602.56, 0.01);
    CHECK_NEAR(aim_pt_res(-200.0f, 1000.0f), 185.20, 0.01);
    CHECK_NEAR(aim_pt_res(400.0f, 1000.0f), 2470.92, 0.02);
    for (int t = -200; t <= 850; t += 25) {
        CHECK_NEAR(aim_pt_temp(aim_pt_res((float)t, 1000.0f), 1000.0f), t, 0.002);
    }
    CHECK_NEAR(aim_pt_temp(100.0f * 1.385055f, 100.0f), 100.0, 0.002);   /* R0 parameter */
}

static void test_ntc(void)
{
    CHECK_NEAR(aim_ntc_temp_b(10000.0f, 10000.0f, 3950.0f), 25.0, 1e-3);
    const double t = 1.0 / (1.0 / 298.15 + log(32600.0 / 10000.0) / 3950.0) - 273.15;
    CHECK_NEAR(aim_ntc_temp_b(32600.0f, 10000.0f, 3950.0f), t, 2e-3);
    /* Steinhart-Hart against the formula in double. */
    const double l = log(10000.0);
    const double tsh = 1.0 / (1.009249522e-3 + 2.378405444e-4 * l + 2.019202697e-7 * l * l * l) - 273.15;
    CHECK_NEAR(aim_ntc_temp_sh(10000.0f, 1.009249522e-3f, 2.378405444e-4f, 2.019202697e-7f), tsh, 2e-3);
    CHECK(isnan(aim_ntc_temp_b(-1.0f, 10000.0f, 3950.0f)));
}

static void test_table(void)
{
    ai_ch_cfg_t c;
    aim_ch_defaults(&c);
    c.mode = AI_MODE_KTY;
    c.rtd_n = 3;
    c.rtd_r[0] = 1000.0f; c.rtd_t[0] = 0.0f;
    c.rtd_r[1] = 2000.0f; c.rtd_t[1] = 100.0f;
    c.rtd_r[2] = 4000.0f; c.rtd_t[2] = 150.0f;
    float t = 0.0f;
    CHECK(aim_table_temp(1500.0f, &c, &t)); CHECK_NEAR(t, 50.0, 1e-4);
    CHECK(aim_table_temp(3000.0f, &c, &t)); CHECK_NEAR(t, 125.0, 1e-4);
    CHECK(aim_table_temp(1000.0f, &c, &t)); CHECK_NEAR(t, 0.0, 1e-4);
    CHECK(!aim_table_temp(999.0f, &c, &t));
    CHECK(!aim_table_temp(4001.0f, &c, &t));
    CHECK(aim_ch_valid(&c));
    c.rtd_r[2] = 1500.0f;              /* not increasing */
    CHECK(!aim_ch_valid(&c));
    c.rtd_n = 1;
    CHECK(!aim_table_temp(1000.0f, &c, &t));
}

static void test_eval_two_phase(void)
{
    ai_ch_cfg_t c;
    aim_ch_defaults(&c);
    c.mode = AI_MODE_PT1000;
    const float vb = M.v_b[AI_RANGE_PM2];

    /* A 2 mV ADC offset on both phases must cancel. */
    const double os = 0.002;
    const double rs_list[] = { 803.0, 1000.0, 1385.0, 1758.0, 2470.0 };
    for (unsigned i = 0; i < sizeof rs_list / sizeof rs_list[0]; i++) {
        const double vp_a = pin_vp(node_vn(rs_list[i], vb), vb) + os;
        const double vp_b = aim_vp0(vb, &M) + os;
        aim_result_t r;
        aim_eval_two_phase(&c, &M, &D, (float)vp_a, (float)vp_b, &r);
        CHECK(r.status == 0u);
        CHECK_NEAR(r.raw, rs_list[i], rs_list[i] * 1e-4);
        CHECK_NEAR(r.value, aim_pt_temp((float)rs_list[i], 1000.0f), 0.03);
    }

    /* Line resistance is subtracted from R_S. */
    c.r_line_mohm = 1500u;
    aim_result_t r;
    aim_eval_two_phase(&c, &M, &D, (float)pin_vp(node_vn(1001.5, vb), vb), aim_vp0(vb, &M), &r);
    CHECK_NEAR(r.raw, 1000.0, 0.05);
    CHECK_NEAR(r.value, 0.0, 0.02);
    c.r_line_mohm = 0u;

    /* Open circuit (V_n at Vex) and short. */
    aim_eval_two_phase(&c, &M, &D, 2.55f, aim_vp0(vb, &M), &r);
    CHECK((r.status & AI_ST_WIRE_BREAK) != 0u && isnan(r.value));
    aim_eval_two_phase(&c, &M, &D, (float)pin_vp(node_vn(5.0, vb), vb), aim_vp0(vb, &M), &r);
    CHECK((r.status & AI_ST_SHORT) != 0u && isnan(r.value));

    /* NTC 10 k at 25 °C on the ±5.12 V range. */
    c.mode = AI_MODE_NTC;
    const float vbn = M.v_b[AI_RANGE_PM5];
    aim_eval_two_phase(&c, &M, &D, (float)pin_vp(node_vn(10000.0, vbn), vbn), aim_vp0(vbn, &M), &r);
    CHECK(r.status == 0u);
    CHECK_NEAR(r.value, 25.0, 0.01);
    /* 336 kΩ (-40 °C class) stays inside the NTC limits. */
    aim_eval_two_phase(&c, &M, &D, (float)pin_vp(node_vn(336000.0, vbn), vbn), aim_vp0(vbn, &M), &r);
    CHECK(r.status == 0u);
    CHECK_NEAR(r.raw, 336000.0, 336000.0 * 2e-4);

    /* Table mode outside the table. */
    c.mode = AI_MODE_RTD_TABLE;
    c.rtd_n = 0;
    aim_eval_two_phase(&c, &M, &D, (float)pin_vp(node_vn(1000.0, vb), vb), aim_vp0(vb, &M), &r);
    CHECK((r.status & AI_ST_OUT_OF_TABLE) != 0u && isnan(r.value));
}

static void test_eval_single(void)
{
    ai_ch_cfg_t c;
    aim_ch_defaults(&c);
    aim_result_t r;
    uint8_t di = 0;
    const float vb = M.v_b[AI_RANGE_P10];

    c.mode = AI_MODE_V_0_10;
    aim_eval_single(&c, &M, &D, aim_vp_from_vn(5.0f, vb, &M), &di, &r);
    CHECK(r.status == 0u); CHECK_NEAR(r.value, 5.0, 1e-4); CHECK_NEAR(r.raw, 5.0, 1e-4);
    aim_eval_single(&c, &M, &D, aim_vp_from_vn(10.3f, vb, &M), &di, &r);
    CHECK(r.status == AI_ST_OVERRANGE); CHECK_NEAR(r.value, 10.3, 1e-4);   /* value kept */
    aim_eval_single(&c, &M, &D, aim_vp_from_vn(-0.2f, vb, &M), &di, &r);
    CHECK((r.status & AI_ST_NEG_INPUT) != 0u && isnan(r.value));

    c.mode = AI_MODE_V_2_10;
    aim_eval_single(&c, &M, &D, aim_vp_from_vn(1.2f, vb, &M), &di, &r);
    CHECK((r.status & AI_ST_WIRE_BREAK) != 0u && isnan(r.value));
    c.scale_lo = 0.0f; c.scale_hi = 100.0f;                 /* 2..10 V -> 0..100 % */
    aim_eval_single(&c, &M, &D, aim_vp_from_vn(6.0f, vb, &M), &di, &r);
    CHECK(r.status == 0u); CHECK_NEAR(r.value, 50.0, 1e-3);
    c.scale_lo = c.scale_hi = 0.0f;

    const float vbi = M.v_b[AI_RANGE_PM2];
    c.mode = AI_MODE_I_4_20;
    aim_eval_single(&c, &M, &D, aim_vp_from_vn(aim_vn_for_current(12.0f, vbi, &M), vbi, &M), &di, &r);
    CHECK(r.status == 0u); CHECK_NEAR(r.value, 12.0, 1e-3);
    aim_eval_single(&c, &M, &D, aim_vp_from_vn(aim_vn_for_current(2.0f, vbi, &M), vbi, &M), &di, &r);
    CHECK((r.status & AI_ST_WIRE_BREAK) != 0u);
    aim_eval_single(&c, &M, &D, aim_vp_from_vn(aim_vn_for_current(22.0f, vbi, &M), vbi, &M), &di, &r);
    CHECK(r.status == AI_ST_OVERRANGE);
    c.mode = AI_MODE_I_0_20;
    aim_eval_single(&c, &M, &D, aim_vp_from_vn(aim_vn_for_current(0.5f, vbi, &M), vbi, &M), &di, &r);
    CHECK(r.status == 0u); CHECK_NEAR(r.value, 0.5, 1e-3);

    /* DI hysteresis: < 1 kΩ -> 1, > 5 kΩ -> 0, in between unchanged. */
    const float vbd = M.v_b[AI_RANGE_P5];
    c.mode = AI_MODE_DI;
    di = 0;
    aim_eval_single(&c, &M, &D, (float)pin_vp(node_vn(3000.0, vbd), vbd), &di, &r);
    CHECK(r.value == 0.0f);
    aim_eval_single(&c, &M, &D, (float)pin_vp(node_vn(500.0, vbd), vbd), &di, &r);
    CHECK(r.value == 1.0f); CHECK_NEAR(r.raw, 500.0, 0.5);
    aim_eval_single(&c, &M, &D, (float)pin_vp(node_vn(3000.0, vbd), vbd), &di, &r);
    CHECK(r.value == 1.0f);
    aim_eval_single(&c, &M, &D, (float)pin_vp(node_vn(8000.0, vbd), vbd), &di, &r);
    CHECK(r.value == 0.0f);
    aim_eval_single(&c, &M, &D, 4.2f, &di, &r);                   /* open */
    CHECK(r.value == 0.0f && r.status == 0u);
}

static void test_thresholds(void)
{
    /* 25 mA trip code on the ±2.56 V range lies inside the scale. */
    const float vn = aim_vn_for_current(AIM_TRIP_I_MA, M.v_b[AI_RANGE_PM2], &M);
    const uint16_t code = aim_vn_to_code(vn, AI_RANGE_PM2, &M);
    CHECK(code > 32768u + 30000u && code < 65535u);
    const float back = aim_current_ma(aim_vn(aim_code_to_vp(code, AI_RANGE_PM2, M.v_ref),
                                             M.v_b[AI_RANGE_PM2], &M), M.v_b[AI_RANGE_PM2], &M);
    CHECK_NEAR(back, 25.0, 0.01);
    /* PRECHECK 4.5 V on ±10.24 V. */
    const uint16_t pc = aim_vn_to_code(AIM_PRECHECK_HI_V, AI_RANGE_PM10, &M);
    CHECK_NEAR(aim_vn(aim_code_to_vp(pc, AI_RANGE_PM10, M.v_ref), M.v_b[AI_RANGE_PM10], &M), 4.5, 0.001);
}

static void test_keys(void)
{
    keys_init();
    uint8_t combo[8] = { AI_KEY_Q1, AI_KEY_Q2, AI_KEY_PD, 0, AI_KEY_Q1 | AI_KEY_Q2,
                         AI_KEY_PD | AI_KEY_Q1, AI_KEY_PD | AI_KEY_Q2, AI_KEY_Q1 };
    uint8_t bad = 0;
    const uint32_t w = keys_build(combo, &bad);
    CHECK(bad == 0x70u);
    CHECK(combo[4] == 0u && combo[5] == 0u && combo[6] == 0u && combo[0] == AI_KEY_Q1);
    /* ch0 Q1 -> W0, ch1 Q2 -> W17, ch2 PD -> W10, ch7 Q1 -> W7. */
    CHECK(w == ((1u << 0) | (1u << 17) | (1u << 10) | (1u << 7)));
    CHECK(keys_combo_forbidden(0x08u));
    for (uint8_t ch = 0; ch < 8; ch++) {
        uint8_t one[8] = { 0 };
        one[ch] = AI_KEY_Q2;
        CHECK(keys_build(one, &bad) == (1u << (16 + ch)) && bad == 0u);
    }
    /* ADC map is a permutation of 0..7. */
    uint8_t seen = 0;
    for (uint8_t i = 0; i < 8; i++) { seen |= (uint8_t)(1u << BOARD_ADC_MAP[i]); }
    CHECK(seen == 0xFFu);
}

static void test_valid(void)
{
    ai_ch_cfg_t c;
    aim_ch_defaults(&c);
    CHECK(aim_ch_valid(&c));
    c.di_r_off = 500.0f;
    CHECK(!aim_ch_valid(&c));
    aim_ch_defaults(&c);
    c.mode = AI_MODE_COUNT;
    CHECK(!aim_ch_valid(&c));
    aim_ch_defaults(&c);
    c.pt_r0 = __builtin_nanf("");
    CHECK(!aim_ch_valid(&c));
}

int main(void)
{
    aim_model_defaults(&M);
    aim_diag_defaults(&D);
    test_codes();
    test_modes();
    test_model();
    test_pt();
    test_ntc();
    test_table();
    test_eval_two_phase();
    test_eval_single();
    test_thresholds();
    test_keys();
    test_valid();
    printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
