/**
  ******************************************************************************
  * @file    keys.c
  * @brief   8UAI key shadow -> chain word (see keys.h).
  ******************************************************************************
  */
#include "keys.h"

#include "ai_types.h"
#include "board_pins.h"

/* RAM copy of BOARD_KEY_MAP (.data, not .rodata). */
static uint32_t s_bit_q1[8];
static uint32_t s_bit_q2[8];
static uint32_t s_bit_pd[8];

void keys_init(void)
{
    for (uint8_t ch = 0u; ch < 8u; ch++) {
        s_bit_q1[ch] = 1uL << BOARD_KEY_MAP[ch].q1;
        s_bit_q2[ch] = 1uL << BOARD_KEY_MAP[ch].q2;
        s_bit_pd[ch] = 1uL << BOARD_KEY_MAP[ch].pd;
    }
}

KEYS_RAMFUNC bool keys_combo_forbidden(uint8_t combo)
{
    const bool q1 = (combo & AI_KEY_Q1) != 0u;
    const bool q2 = (combo & AI_KEY_Q2) != 0u;
    const bool pd = (combo & AI_KEY_PD) != 0u;
    return (q1 && q2) || (pd && (q1 || q2)) || (combo & ~(AI_KEY_Q1 | AI_KEY_Q2 | AI_KEY_PD)) != 0u;
}

KEYS_RAMFUNC uint32_t keys_build(uint8_t combo[8], uint8_t* bad_mask)
{
    uint32_t w = 0u;
    uint8_t bad = 0u;
    for (uint8_t ch = 0u; ch < 8u; ch++) {
        if (keys_combo_forbidden(combo[ch])) {
            combo[ch] = 0u;
            bad |= (uint8_t)(1u << ch);
        }
        if (combo[ch] & AI_KEY_Q1) { w |= s_bit_q1[ch]; }
        if (combo[ch] & AI_KEY_Q2) { w |= s_bit_q2[ch]; }
        if (combo[ch] & AI_KEY_PD) { w |= s_bit_pd[ch]; }
    }
    *bad_mask = bad;
    return w;
}
