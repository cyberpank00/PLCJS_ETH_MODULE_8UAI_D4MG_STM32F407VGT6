/**
  ******************************************************************************
  * @file    ai_types.h
  * @brief   8UAI analog front-end: shared types (no HAL dependency).
  *
  *  Used by the persistent settings, the conversion maths (ai_math, host
  *  tested), the acquisition state machine (ai) and the Modbus adapter.
  *  Channel numbers everywhere are input numbers AI0..AI7; the ADS8688 channel
  *  number never leaves board_pins.h / ai.c.
  ******************************************************************************
  */
#ifndef APPLICATION_AI_TYPES_H
#define APPLICATION_AI_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AI_CHANNELS         8u
#define AI_RTD_TABLE_MAX    32u

/* Channel mode (TZ section 6). Stored in Flash and on Modbus: keep values. */
typedef enum {
    AI_MODE_OFF       = 0,
    AI_MODE_V_0_10    = 1,
    AI_MODE_V_2_10    = 2,
    AI_MODE_I_0_20    = 3,
    AI_MODE_I_4_20    = 4,
    AI_MODE_PT1000    = 5,
    AI_MODE_KTY       = 6,
    AI_MODE_RTD_TABLE = 7,
    AI_MODE_NTC       = 8,
    AI_MODE_DI        = 9,
    AI_MODE_COUNT
} ai_mode_t;

/* ADS8688 input ranges used by the module (index into the range table). */
typedef enum {
    AI_RANGE_PM10 = 0,   /* ±2.5·VREF   code 0x00  ±10.24 V  V_B 2.25 V  PRECHECK */
    AI_RANGE_PM5  = 1,   /* ±1.25·VREF  code 0x01  ±5.12 V   V_B 2.00 V  NTC      */
    AI_RANGE_PM2  = 2,   /* ±0.625·VREF code 0x02  ±2.56 V   V_B 1.60 V  I, RTD   */
    AI_RANGE_P10  = 3,   /* 0..2.5·VREF code 0x05  0..10.24  V_B 2.50 V  V        */
    AI_RANGE_P5   = 4,   /* 0..1.25·VREF code 0x06 0..5.12   V_B 2.50 V  DI       */
    AI_RANGE_COUNT
} ai_range_t;

/* Per-channel key combination bits (logical, mapped to W bits by KEY_MAP). */
#define AI_KEY_Q1   0x1u   /* excitation: node -> Vex via Rref  */
#define AI_KEY_Q2   0x2u   /* shunt 100 Ω node -> AGND           */
#define AI_KEY_PD   0x4u   /* 1 MΩ pull-down (voltage mode only) */

/* NTC model selector. */
#define AI_NTC_MODEL_B   0u
#define AI_NTC_MODEL_SH  1u

/* Channel status bits (Modbus IR332..339). Any bit except OVERRANGE /
 * UNDERRANGE makes the value NaN. */
#define AI_ST_NO_DATA       0x0001u
#define AI_ST_OVERRANGE     0x0002u
#define AI_ST_UNDERRANGE    0x0004u
#define AI_ST_WIRE_BREAK    0x0008u
#define AI_ST_SHORT         0x0010u
#define AI_ST_OVERLOAD      0x0020u
#define AI_ST_EXT_VOLTAGE   0x0040u
#define AI_ST_LOCKED        0x0080u
#define AI_ST_NEG_INPUT     0x0100u
#define AI_ST_ADC_CLIP      0x0200u
#define AI_ST_OUT_OF_TABLE  0x0400u
#define AI_ST_CONFIG_ERR    0x0800u
#define AI_ST_ADC_FAIL      0x1000u
#define AI_ST_INTERNAL      0x2000u   /* forbidden key combination refused */
#define AI_ST_OFF           0x4000u   /* channel mode OFF                  */
#define AI_ST_VALUE_OK_MASK (AI_ST_OVERRANGE | AI_ST_UNDERRANGE)

/* Input-node model constants (TZ section 2). Service parameters. */
typedef struct {
    float v_ref;    /* ADC reference = Vex, V            (4.096)  */
    float r_ref;    /* Rref, Ω                           (4990)   */
    float r_ds1;    /* Q1 on-resistance, Ω               (0.8)    */
    float r_sh;     /* shunt, Ω                          (100)    */
    float r_ds2;    /* Q2 on-resistance, Ω               (0.075)  */
    float r_7;      /* node -> AINx resistor, Ω          (2200)   */
    float r_in;     /* ADS8688 input resistance, Ω       (1e6)    */
    float v_b[AI_RANGE_COUNT];   /* ADC input bias per range, V    */
} ai_model_t;

/* Result-diagnostic thresholds (TZ section 8). Service parameters. */
typedef struct {
    float rtd_open_ohm;     /* 5000    */
    float rtd_short_ohm;    /* 20      */
    float ntc_open_ohm;     /* 2e6     */
    float ntc_short_ohm;    /* 50      */
    float v210_open_v;      /* 1.8     */
    float v_over_v;         /* 10.2    */
    float v_neg_v;          /* -0.1    */
    float i420_open_ma;     /* 3.6     */
    float i_over_ma;        /* 21.0    */
} ai_diag_t;

/* Per-channel configuration (TZ section 11). Stored in Flash: layout frozen. */
typedef struct {
    uint8_t  mode;            /* ai_mode_t                                 */
    uint8_t  ntc_model;       /* AI_NTC_MODEL_B / _SH                      */
    uint16_t filter_tau_ms;   /* EMA time constant, 0 = off                */
    uint32_t r_line_mohm;     /* resistance of both wires, mΩ              */
    float    pt_r0;           /* Pt R0, Ω                     (1000)       */
    float    ntc_r25;         /* Ω                            (10000)      */
    float    ntc_b;           /* K                            (3950)       */
    float    ntc_sh_a;        /* Steinhart-Hart a, b, c                    */
    float    ntc_sh_b;
    float    ntc_sh_c;
    float    di_r_on;         /* Ω, below -> 1                (1000)       */
    float    di_r_off;        /* Ω, above -> 0                (5000)       */
    float    scale_lo;        /* engineering value at the signal low end   */
    float    scale_hi;        /* ... at the high end; lo == hi -> physical */
    uint8_t  rtd_n;           /* table points used, 0..32                  */
    uint8_t  reserved[3];
    float    rtd_r[AI_RTD_TABLE_MAX];   /* Ω, strictly increasing          */
    float    rtd_t[AI_RTD_TABLE_MAX];   /* °C                              */
} ai_ch_cfg_t;

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_AI_TYPES_H */
