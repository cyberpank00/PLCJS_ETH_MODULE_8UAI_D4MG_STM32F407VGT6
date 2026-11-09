/**
  ******************************************************************************
  * @file    modbus_app.c
  * @brief   Modbus register-map adapter for the 8UAI module (see modbus_app.h).
  ******************************************************************************
  */

#include "modbus_app.h"

#include <string.h>

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "ai.h"
#include "ai_math.h"
#include "fw_header.h"
#include "led_module.h"
#include "settings.h"
#include "temp_module.h"

/* ---------------------------------------------------------------------------
 * Firmware version — derived from FW_VERSION_VALUE (encoding
 * (major << 8) | minor) so there is a single source of truth shared with the
 * firmware header (IR120/IR121).
 * ------------------------------------------------------------------------- */
#define FW_VER_MAJOR    ((FW_VERSION_VALUE >> 8u) & 0xFFu)
#define FW_VER_MINOR    (FW_VERSION_VALUE & 0xFFu)

/* ---------------------------------------------------------------------------
 * Pending action flags driven by special holding-register triggers.
 * ------------------------------------------------------------------------- */
static volatile uint8_t s_pending_save           = 0u;
static volatile uint8_t s_pending_reboot         = 0u;
static volatile uint8_t s_pending_factory_reset  = 0u;
static volatile uint8_t s_pending_bootloader     = 0u;
static volatile uint8_t s_pending_switch_reset   = 0u;
static volatile uint32_t s_last_request_tick     = 0u;

/* Analog parameters touched by the current request. */
static uint8_t s_touched_ch;
static bool    s_touched_common;

/* Backup of the analog part of the settings, restored when an FC16 request
 * leaves a channel or the common block inconsistent. Only the Modbus task
 * uses it. */
static ai_ch_cfg_t s_bak_ch[MB_AI_CHANNELS];
static ai_model_t  s_bak_model;
static ai_diag_t   s_bak_diag;
static uint16_t    s_bak_mains, s_bak_blank;

void modbus_app_init(void)
{
    s_pending_save          = 0u;
    s_pending_reboot        = 0u;
    s_pending_factory_reset = 0u;
    s_pending_bootloader    = 0u;
    s_pending_switch_reset  = 0u;
    s_last_request_tick     = 0u;
}

void modbus_app_notify_request(void)      { s_last_request_tick = HAL_GetTick(); }
uint32_t modbus_app_last_request_tick(void) { return s_last_request_tick; }

uint8_t modbus_app_take_pending_save(void)
{
    uint8_t v = s_pending_save; s_pending_save = 0u; return v;
}
uint8_t modbus_app_take_pending_reboot(void)
{
    uint8_t v = s_pending_reboot; s_pending_reboot = 0u; return v;
}
uint8_t modbus_app_take_pending_factory_reset(void)
{
    uint8_t v = s_pending_factory_reset; s_pending_factory_reset = 0u; return v;
}
uint8_t modbus_app_take_pending_bootloader(void)
{
    uint8_t v = s_pending_bootloader; s_pending_bootloader = 0u; return v;
}
uint8_t modbus_app_take_pending_switch_reset(void)
{
    uint8_t v = s_pending_switch_reset; s_pending_switch_reset = 0u; return v;
}

/* ---------------------------------------------------------------------------
 * float32 <-> register-pair helpers (HIGH word first).
 * ------------------------------------------------------------------------- */
static uint16_t float_word(float f, uint8_t word)
{
    uint32_t b;
    memcpy(&b, &f, sizeof(b));
    return (word == 0u) ? (uint16_t)(b >> 16) : (uint16_t)(b & 0xFFFFu);
}

static float float_set_word(float f, uint8_t word, uint16_t v)
{
    uint32_t b;
    memcpy(&b, &f, sizeof(b));
    if (word == 0u) {
        b = (b & 0x0000FFFFu) | ((uint32_t)v << 16);
    } else {
        b = (b & 0xFFFF0000u) | (uint32_t)v;
    }
    float o;
    memcpy(&o, &b, sizeof(o));
    return o;
}

/* ---------------------------------------------------------------------------
 * Address decoding -> pointer to the float behind a register pair.
 * ------------------------------------------------------------------------- */
static bool in_ch_block(uint16_t a, uint8_t* group, uint8_t* ch)
{
    if (a >= MB_HR_CH_BASE + MB_HR_CH_GROUPS * MB_AI_CHANNELS) { return false; }
    const uint16_t rel = (uint16_t)(a - MB_HR_CH_BASE);
    *group = (uint8_t)(rel / MB_AI_CHANNELS);
    *ch    = (uint8_t)(rel % MB_AI_CHANNELS);
    return true;
}

static float* ch_float(ai_ch_cfg_t* c, uint8_t idx)
{
    switch (idx) {
    case 0u: return &c->pt_r0;
    case 1u: return &c->ntc_r25;
    case 2u: return &c->ntc_b;
    case 3u: return &c->ntc_sh_a;
    case 4u: return &c->ntc_sh_b;
    case 5u: return &c->ntc_sh_c;
    case 6u: return &c->scale_lo;
    default: return &c->scale_hi;
    }
}

/* Resolve a holding address to a float of the analog configuration. Sets
 * *ch to the channel (or 0xFF for the common block) and *word. */
static float* float_reg(settings_t* s, uint16_t a, uint8_t* ch, uint8_t* word)
{
    *word = (uint8_t)((a - MB_HR_CH_FLOAT_BASE) & 1u);
    if (a >= MB_HR_CH_FLOAT_BASE &&
        a < MB_HR_CH_FLOAT_BASE + MB_AI_CHANNELS * MB_HR_CH_FLOAT_STRIDE) {
        const uint16_t rel = (uint16_t)(a - MB_HR_CH_FLOAT_BASE);
        *ch = (uint8_t)(rel / MB_HR_CH_FLOAT_STRIDE);
        return ch_float(&s->ch[*ch], (uint8_t)((rel % MB_HR_CH_FLOAT_STRIDE) / 2u));
    }
    if (a >= MB_HR_RTD_BASE && a < MB_HR_RTD_BASE + MB_AI_CHANNELS * MB_HR_RTD_STRIDE) {
        const uint16_t rel = (uint16_t)(a - MB_HR_RTD_BASE);
        *ch = (uint8_t)(rel / MB_HR_RTD_STRIDE);
        const uint16_t idx = (uint16_t)(rel % MB_HR_RTD_STRIDE);
        *word = (uint8_t)(idx & 1u);
        return (idx < 64u) ? &s->ch[*ch].rtd_r[idx / 2u] : &s->ch[*ch].rtd_t[(idx - 64u) / 2u];
    }
    *ch = 0xFFu;
    if (a >= MB_HR_MODEL_BASE && a < MB_HR_MODEL_BASE + 2u * MB_HR_MODEL_COUNT) {
        *word = (uint8_t)((a - MB_HR_MODEL_BASE) & 1u);
        return &((float*)&s->model)[(a - MB_HR_MODEL_BASE) / 2u];
    }
    if (a >= MB_HR_DIAG_BASE && a < MB_HR_DIAG_BASE + 2u * MB_HR_DIAG_COUNT) {
        *word = (uint8_t)((a - MB_HR_DIAG_BASE) & 1u);
        return &((float*)&s->diag)[(a - MB_HR_DIAG_BASE) / 2u];
    }
    return NULL;
}

_Static_assert(sizeof(ai_model_t) == MB_HR_MODEL_COUNT * sizeof(float), "model block layout");
_Static_assert(sizeof(ai_diag_t) == MB_HR_DIAG_COUNT * sizeof(float), "diag block layout");

/* ---------------------------------------------------------------------------
 * Reads
 * ------------------------------------------------------------------------- */
static uint16_t read_input(uint16_t address)
{
    if (address >= MB_IR_AI_BASE && address < MB_IR_AI_END) {
        ai_data_t d;
        if (address < MB_IR_AI_STATUS) {
            const uint16_t rel  = (uint16_t)(address - MB_IR_AI_VALUE);
            const uint16_t grp  = (uint16_t)(rel / (2u * MB_AI_CHANNELS));
            const uint8_t  ch   = (uint8_t)((rel % (2u * MB_AI_CHANNELS)) / 2u);
            ai_get(ch, &d);
            return float_word((grp == 0u) ? d.value : d.raw, (uint8_t)(rel & 1u));
        }
        const uint16_t rel = (uint16_t)(address - MB_IR_AI_STATUS);
        ai_get((uint8_t)(rel % MB_AI_CHANNELS), &d);
        switch (rel / MB_AI_CHANNELS) {
        case 0u:  return d.status;
        case 1u:  return d.seq;
        case 2u:  return d.trip_count;
        default:  return d.state;
        }
    }

    switch (address) {
    case MB_IR_FW_VER_MAJOR: return FW_VER_MAJOR;
    case MB_IR_FW_VER_MINOR: return FW_VER_MINOR;
    case MB_IR_UPTIME_LO:    return (uint16_t)((HAL_GetTick() / 1000u) & 0xFFFFu);
    case MB_IR_UPTIME_HI:    return (uint16_t)(((HAL_GetTick() / 1000u) >> 16u) & 0xFFFFu);
    case MB_IR_MODULE_ID:    return MODULE_ID_08UAI;
    case MB_IR_TEMPERATURE:  return (uint16_t)temp_module_read_decicelsius();
    case MB_IR_MODULE_FLAGS: return ai_module_flags();
    case MB_IR_SAMPLES:      return ai_samples_per_phase();
    case MB_IR_SAVE_COUNT:   return settings_save_count();
    case MB_IR_SAVE_RESULT:  return settings_save_result();
    default:                 return 0u;
    }
}

static bool input_address_valid(uint16_t address)
{
    if (address >= MB_IR_AI_BASE && address < MB_IR_AI_END) { return true; }
    return (address >= MB_IR_FW_VER_MAJOR && address <= MB_IR_UPTIME_HI) ||
           (address >= MB_IR_MODULE_ID && address <= MB_IR_SAVE_RESULT);
}

static uint16_t read_holding(uint16_t address)
{
    settings_t* s = settings_get();
    uint8_t group, ch, word;

    if (in_ch_block(address, &group, &ch)) {
        const ai_ch_cfg_t* c = &s->ch[ch];
        switch (group) {
        case MB_HR_CH_GROUP_READING:   return (uint16_t)ai_get_int16(ch);
        case MB_HR_CH_GROUP_MODE:      return c->mode;
        case MB_HR_CH_GROUP_FILTER:    return c->filter_tau_ms;
        case MB_HR_CH_GROUP_RLINE:     return (c->r_line_mohm > 0xFFFFu) ? 0xFFFFu : (uint16_t)c->r_line_mohm;
        case MB_HR_CH_GROUP_NTC_MODEL: return c->ntc_model;
        case MB_HR_CH_GROUP_DI_ON:     return (uint16_t)(c->di_r_on > 65535.0f ? 65535.0f : c->di_r_on);
        case MB_HR_CH_GROUP_DI_OFF:    return (uint16_t)(c->di_r_off > 65535.0f ? 65535.0f : c->di_r_off);
        default:                       return c->rtd_n;
        }
    }
    const float* f = float_reg(s, address, &ch, &word);
    if (f != NULL) { return float_word(*f, word); }

    switch (address) {
    case MB_HR_LED_MODE:          return s->led_mode;
    case MB_HR_SLAVE_ID:          return s->modbus_slave_id;
    case MB_HR_TCP_PORT:          return s->modbus_tcp_port;
    case MB_HR_MAINS_HZ:          return s->mains_hz;
    case MB_HR_T_BLANK_US:        return s->t_blank_us;

    case MB_HR_IP_BASE + 0u:      return s->ip[0];
    case MB_HR_IP_BASE + 1u:      return s->ip[1];
    case MB_HR_IP_BASE + 2u:      return s->ip[2];
    case MB_HR_IP_BASE + 3u:      return s->ip[3];

    case MB_HR_NETMASK_BASE + 0u: return s->netmask[0];
    case MB_HR_NETMASK_BASE + 1u: return s->netmask[1];
    case MB_HR_NETMASK_BASE + 2u: return s->netmask[2];
    case MB_HR_NETMASK_BASE + 3u: return s->netmask[3];

    case MB_HR_GATEWAY_BASE + 0u: return s->gateway[0];
    case MB_HR_GATEWAY_BASE + 1u: return s->gateway[1];
    case MB_HR_GATEWAY_BASE + 2u: return s->gateway[2];
    case MB_HR_GATEWAY_BASE + 3u: return s->gateway[3];

    case MB_HR_USE_DHCP:          return s->use_dhcp;
    case MB_HR_TEMPERATURE:       return (uint16_t)temp_module_read_decicelsius();
    default:                      return 0u;   /* triggers / commands read 0 */
    }
}

/* ---------------------------------------------------------------------------
 * Writes
 * ------------------------------------------------------------------------- */
static nmbs_error write_ch_block(settings_t* s, uint8_t group, uint8_t ch, uint16_t value)
{
    ai_ch_cfg_t* c = &s->ch[ch];
    switch (group) {
    case MB_HR_CH_GROUP_MODE:
        if (value >= AI_MODE_COUNT) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        c->mode = (uint8_t)value;
        break;
    case MB_HR_CH_GROUP_FILTER:    c->filter_tau_ms = value; break;
    case MB_HR_CH_GROUP_RLINE:     c->r_line_mohm = value; break;
    case MB_HR_CH_GROUP_NTC_MODEL:
        if (value > AI_NTC_MODEL_SH) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        c->ntc_model = (uint8_t)value;
        break;
    case MB_HR_CH_GROUP_DI_ON:     c->di_r_on = (float)value; break;
    case MB_HR_CH_GROUP_DI_OFF:    c->di_r_off = (float)value; break;
    case MB_HR_CH_GROUP_RTD_N:
        if (value > AI_RTD_TABLE_MAX) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        c->rtd_n = (uint8_t)value;
        break;
    default:
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;   /* reading is read-only */
    }
    s_touched_ch |= (uint8_t)(1u << ch);
    return NMBS_ERROR_NONE;
}

static nmbs_error apply_holding_write(uint16_t address, uint16_t value)
{
    settings_t* s = settings_get();
    uint8_t group, ch, word;

    if (in_ch_block(address, &group, &ch)) {
        return write_ch_block(s, group, ch, value);
    }

    float* f = float_reg(s, address, &ch, &word);
    if (f != NULL) {
        *f = float_set_word(*f, word, value);
        if (ch == 0xFFu) { s_touched_common = true; }
        else             { s_touched_ch |= (uint8_t)(1u << ch); }
        return NMBS_ERROR_NONE;
    }

    switch (address) {
    case MB_HR_MAINS_HZ:
        if (value != 50u && value != 60u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->mains_hz = value;
        s_touched_common = true;
        break;

    case MB_HR_T_BLANK_US:
        if (value < SETTINGS_T_BLANK_US_MIN || value > SETTINGS_T_BLANK_US_MAX) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
        }
        s->t_blank_us = value;
        s_touched_common = true;
        break;

    case MB_HR_CH_COMMAND: {
        const uint16_t base = (uint16_t)(value & ~MB_CMD_CH_MASK);
        const uint8_t  cch  = (uint8_t)(value & MB_CMD_CH_MASK);
        bool ok;
        if (value == MB_CMD_APPLY)                                  { ok = ai_command(0u, AI_CMD_APPLY); }
        else if (base == MB_CMD_UNLOCK_BASE && cch < MB_AI_CHANNELS)  { ok = ai_command(cch, AI_CMD_UNLOCK); }
        else if (base == MB_CMD_RESTART_BASE && cch < MB_AI_CHANNELS) { ok = ai_command(cch, AI_CMD_RESTART); }
        else                                                        { ok = (value == 0u); }
        if (!ok) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        break;
    }

    case MB_HR_LED_MODE:
        if (value > (uint16_t)LED_MODE_STATE_MACHINE) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->led_mode = value;
        led_module_set_mode((uint8_t)value);
        break;

    case MB_HR_SLAVE_ID:
        if (value < 1u || value > 247u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->modbus_slave_id = (uint8_t)value;
        break;

    case MB_HR_TCP_PORT:
        if (value == 0u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->modbus_tcp_port = value;
        break;

    case MB_HR_IP_BASE + 0u: case MB_HR_IP_BASE + 1u:
    case MB_HR_IP_BASE + 2u: case MB_HR_IP_BASE + 3u:
        if (value > 255u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->ip[address - MB_HR_IP_BASE] = (uint8_t)value;
        break;

    case MB_HR_NETMASK_BASE + 0u: case MB_HR_NETMASK_BASE + 1u:
    case MB_HR_NETMASK_BASE + 2u: case MB_HR_NETMASK_BASE + 3u:
        if (value > 255u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->netmask[address - MB_HR_NETMASK_BASE] = (uint8_t)value;
        break;

    case MB_HR_GATEWAY_BASE + 0u: case MB_HR_GATEWAY_BASE + 1u:
    case MB_HR_GATEWAY_BASE + 2u: case MB_HR_GATEWAY_BASE + 3u:
        if (value > 255u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->gateway[address - MB_HR_GATEWAY_BASE] = (uint8_t)value;
        break;

    case MB_HR_USE_DHCP:
        /* Network mode: 0=static, 1=DHCP, 2=link-local (see NET_MODE_* in settings.h). */
        if (value > NET_MODE_LINKLOCAL) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        s->use_dhcp = (uint8_t)value;
        break;

    case MB_HR_TRIG_SAVE:
        if (value == MODBUS_TRIG_SAVE) { s_pending_save = 1u; }
        else if (value != 0u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        break;

    case MB_HR_TRIG_REBOOT:
        if (value == MODBUS_TRIG_REBOOT) { s_pending_reboot = 1u; }
        else if (value == MODBUS_TRIG_BOOTLOADER) { s_pending_bootloader = 1u; }
        else if (value == MODBUS_TRIG_SWITCH_RESET) { s_pending_switch_reset = 1u; }
        else if (value != 0u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        break;

    case MB_HR_TRIG_FACTORY_RESET:
        if (value == MODBUS_TRIG_FACTORY_RESET) { s_pending_factory_reset = 1u; }
        else if (value != 0u) { return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE; }
        break;

    default:
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }

    return NMBS_ERROR_NONE;
}

static bool holding_address_valid(uint16_t address)
{
    uint8_t a, b;
    if (in_ch_block(address, &a, &b)) { return true; }
    if (float_reg(settings_get(), address, &a, &b) != NULL) { return true; }
    if (address >= MB_HR_LED_MODE && address <= MB_HR_TRIG_FACTORY_RESET) { return true; }
    return address == MB_HR_TEMPERATURE || address == MB_HR_CH_COMMAND ||
           address == MB_HR_MAINS_HZ || address == MB_HR_T_BLANK_US;
}

/* ---------------------------------------------------------------------------
 * Staging -> driver
 * ------------------------------------------------------------------------- */
static void backup_analog(void)
{
    const settings_t* s = settings_get();
    memcpy(s_bak_ch, s->ch, sizeof(s_bak_ch));
    s_bak_model = s->model;
    s_bak_diag  = s->diag;
    s_bak_mains = s->mains_hz;
    s_bak_blank = s->t_blank_us;
    s_touched_ch = 0u;
    s_touched_common = false;
}

static void restore_analog(void)
{
    settings_t* s = settings_get();
    memcpy(s->ch, s_bak_ch, sizeof(s_bak_ch));
    s->model      = s_bak_model;
    s->diag       = s_bak_diag;
    s->mains_hz   = s_bak_mains;
    s->t_blank_us = s_bak_blank;
}

/* Hand consistent touched blocks to the driver. With @p strict an
 * inconsistent block fails the request (FC16 writes whole floats). */
static bool commit_analog(bool strict)
{
    const settings_t* s = settings_get();
    uint8_t ok_ch = 0u;
    for (uint8_t ch = 0u; ch < MB_AI_CHANNELS; ch++) {
        if ((s_touched_ch & (1u << ch)) != 0u && aim_ch_valid(&s->ch[ch])) { ok_ch |= (uint8_t)(1u << ch); }
    }
    const bool ok_common = s_touched_common &&
                           ai_common_valid(&s->model, &s->diag, s->mains_hz, s->t_blank_us);
    if (strict && (ok_ch != s_touched_ch || ok_common != s_touched_common)) { return false; }

    for (uint8_t ch = 0u; ch < MB_AI_CHANNELS; ch++) {
        if ((ok_ch & (1u << ch)) != 0u) { (void)ai_set_config(ch, &s->ch[ch]); }
    }
    if (ok_common) { (void)ai_set_common(&s->model, &s->diag, s->mains_hz, s->t_blank_us); }
    return true;
}

void modbus_app_sanitize_staged(void)
{
    settings_t* s = settings_get();
    for (uint8_t ch = 0u; ch < MB_AI_CHANNELS; ch++) {
        if (!aim_ch_valid(&s->ch[ch])) { ai_get_config(ch, &s->ch[ch]); }
    }
    if (!ai_common_valid(&s->model, &s->diag, s->mains_hz, s->t_blank_us)) {
        ai_get_common(&s->model, &s->diag, &s->mains_hz, &s->t_blank_us);
    }
}

/* ---------------------------------------------------------------------------
 * nanoMODBUS callbacks
 * ------------------------------------------------------------------------- */
static nmbs_error cb_read_input_registers(uint16_t address, uint16_t quantity,
                                          uint16_t* registers_out)
{
    for (uint16_t i = 0; i < quantity; i++) {
        const uint16_t a = (uint16_t)(address + i);
        if (!input_address_valid(a)) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
        registers_out[i] = read_input(a);
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_read_holding_registers(uint16_t address, uint16_t quantity,
                                            uint16_t* registers_out)
{
    for (uint16_t i = 0; i < quantity; i++) {
        const uint16_t a = (uint16_t)(address + i);
        if (!holding_address_valid(a)) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
        registers_out[i] = read_holding(a);
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_write_single_register(uint16_t address, uint16_t value)
{
    if (!holding_address_valid(address)) {
        return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
    }
    backup_analog();
    const nmbs_error e = apply_holding_write(address, value);
    if (e != NMBS_ERROR_NONE) { restore_analog(); return e; }
    (void)commit_analog(false);
    return NMBS_ERROR_NONE;
}

static nmbs_error cb_write_multiple_registers(uint16_t address, uint16_t quantity,
                                              const uint16_t* registers)
{
    for (uint16_t i = 0; i < quantity; i++) {
        if (!holding_address_valid((uint16_t)(address + i))) {
            return NMBS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
        }
    }
    backup_analog();
    for (uint16_t i = 0; i < quantity; i++) {
        const nmbs_error e = apply_holding_write((uint16_t)(address + i), registers[i]);
        if (e != NMBS_ERROR_NONE) {
            restore_analog();
            return e;
        }
    }
    if (!commit_analog(true)) {
        restore_analog();
        return NMBS_EXCEPTION_ILLEGAL_DATA_VALUE;
    }
    return NMBS_ERROR_NONE;
}

static const nmbs_callbacks s_callbacks = {
    .read_coils                = NULL,
    .read_discrete_inputs      = NULL,
    .read_holding_registers    = cb_read_holding_registers,
    .read_input_registers      = cb_read_input_registers,
    .write_single_coil         = NULL,
    .write_single_register     = cb_write_single_register,
    .write_multiple_coils      = NULL,
    .write_multiple_registers  = cb_write_multiple_registers,
};

const nmbs_callbacks* modbus_app_get_callbacks(void)
{
    return &s_callbacks;
}
