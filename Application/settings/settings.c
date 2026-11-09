/**
  ******************************************************************************
  * @file    settings.c
  * @brief   Persistent settings in Flash sector 10 (0x080C0000, 128 KiB).
  *
  * On boot the structure is read and validated by CRC32; on failure the
  * defaults are applied. Saving erases the whole sector and rewrites the
  * fresh image — from RAM (flash_ram.c), so the analog protection keeps
  * running during the erase.
  ******************************************************************************
  */

#include "settings.h"

#include <string.h>

#include "ai_math.h"
#include "flash_ram.h"

/* ---------------------------------------------------------------------------
 * Storage location
 * ------------------------------------------------------------------------- */
#define SETTINGS_FLASH_SECTOR   10u
#define SETTINGS_FLASH_ADDR     0x080C0000u

_Static_assert(sizeof(settings_t) % 4u == 0u, "settings_t must be word sized");
_Static_assert(sizeof(settings_t) < 16384u, "settings_t too large");

/* ---------------------------------------------------------------------------
 * Static state
 * ------------------------------------------------------------------------- */
static settings_t s_settings;
static bool s_config_error;

/* ---------------------------------------------------------------------------
 * CRC32 (IEEE 802.3, software)
 * ------------------------------------------------------------------------- */
static uint32_t crc32_update(uint32_t crc, const uint8_t* data, uint32_t len)
{
    crc = ~crc;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint32_t k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (-(int32_t)(crc & 1u)));
        }
    }
    return ~crc;
}

static uint32_t settings_crc(const settings_t* s)
{
    /* CRC over everything except the trailing crc32 field. */
    const uint32_t len = (uint32_t)((const uint8_t*)&s->crc32 - (const uint8_t*)s);
    return crc32_update(0u, (const uint8_t*)s, len);
}

/* ---------------------------------------------------------------------------
 * Defaults
 * ------------------------------------------------------------------------- */
void settings_reset_to_defaults(void)
{
    memset(&s_settings, 0, sizeof(s_settings));

    s_settings.magic            = SETTINGS_MAGIC;
    s_settings.version          = SETTINGS_VERSION;

    s_settings.led_mode         = SETTINGS_DEF_LED_MODE;

    s_settings.modbus_tcp_port  = SETTINGS_DEF_TCP_PORT;
    s_settings.modbus_slave_id  = SETTINGS_DEF_SLAVE_ID;
    s_settings.use_dhcp         = SETTINGS_DEF_USE_DHCP;

    s_settings.ip[0]            = SETTINGS_DEF_IP0;
    s_settings.ip[1]            = SETTINGS_DEF_IP1;
    s_settings.ip[2]            = SETTINGS_DEF_IP2;
    s_settings.ip[3]            = SETTINGS_DEF_IP3;

    s_settings.netmask[0]       = SETTINGS_DEF_MASK0;
    s_settings.netmask[1]       = SETTINGS_DEF_MASK1;
    s_settings.netmask[2]       = SETTINGS_DEF_MASK2;
    s_settings.netmask[3]       = SETTINGS_DEF_MASK3;

    s_settings.gateway[0]       = SETTINGS_DEF_GW0;
    s_settings.gateway[1]       = SETTINGS_DEF_GW1;
    s_settings.gateway[2]       = SETTINGS_DEF_GW2;
    s_settings.gateway[3]       = SETTINGS_DEF_GW3;

    s_settings.mains_hz         = SETTINGS_DEF_MAINS_HZ;
    s_settings.t_blank_us       = SETTINGS_DEF_T_BLANK_US;
    aim_model_defaults(&s_settings.model);
    aim_diag_defaults(&s_settings.diag);
    for (uint8_t ch = 0; ch < SETTINGS_AI_CHANNELS; ch++) {
        aim_ch_defaults(&s_settings.ch[ch]);   /* V_0_10 */
    }

    s_settings.crc32            = settings_crc(&s_settings);
}

/* ---------------------------------------------------------------------------
 * Init / accessors
 * ------------------------------------------------------------------------- */
bool settings_init(void)
{
    const settings_t* nv = (const settings_t*)SETTINGS_FLASH_ADDR;

    if (nv->magic == SETTINGS_MAGIC &&
        nv->version == SETTINGS_VERSION &&
        nv->crc32   == settings_crc(nv)) {
        memcpy(&s_settings, nv, sizeof(s_settings));
        s_config_error = false;
        for (uint8_t ch = 0; ch < SETTINGS_AI_CHANNELS; ch++) {
            if (!aim_ch_valid(&s_settings.ch[ch])) {
                aim_ch_defaults(&s_settings.ch[ch]);
                s_config_error = true;
            }
        }
        return !s_config_error;
    }

    /* A blank (erased) sector is a factory state, not a configuration error. */
    s_config_error = (nv->magic != 0xFFFFFFFFu);
    settings_reset_to_defaults();
    return false;
}

bool settings_config_error(void) { return s_config_error; }

settings_t* settings_get(void)
{
    return &s_settings;
}

/* ---------------------------------------------------------------------------
 * Save: erase sector, program 32-bit words, verify (all from RAM)
 * ------------------------------------------------------------------------- */
static bool settings_save_flash(void)
{
    s_settings.crc32 = settings_crc(&s_settings);
    return flash_ram_write_sector(SETTINGS_FLASH_SECTOR, SETTINGS_FLASH_ADDR,
                                  (const uint32_t*)&s_settings,
                                  sizeof(s_settings) / 4u);
}

/* Save bookkeeping since boot, reported over Modbus (IR129/IR130): a tool reads
 * the counter, triggers a save and sees it advance only when the Flash write and
 * its read-back succeeded. result: 0 = no save yet, 1 = last save OK, 2 = failed. */
static uint16_t s_save_count;
static uint16_t s_save_result;

bool settings_save(void)
{
    const bool ok = settings_save_flash();
    s_save_result = ok ? 1u : 2u;
    if (ok) { s_save_count++; }
    return ok;
}

uint16_t settings_save_count(void)  { return s_save_count; }
uint16_t settings_save_result(void) { return s_save_result; }
