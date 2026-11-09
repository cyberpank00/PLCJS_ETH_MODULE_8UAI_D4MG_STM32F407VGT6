/**
  ******************************************************************************
  * @file    keys.h
  * @brief   8UAI key shadow -> 24-bit chain word (TZ section 5). Pure C (host
  *          tested); on target the builder runs from RAM inside the
  *          protection ISR.
  *
  *  The shadow is one key combination (AI_KEY_Q1/Q2/PD) per input. The chain
  *  word W is built only from it, after the forbidden-state check:
  *  Q1 together with Q2, or PD together with Q1 or Q2 -> the channel is
  *  forced to 000 and reported in @p bad_mask (internal error flag).
  ******************************************************************************
  */
#ifndef APPLICATION_KEYS_H
#define APPLICATION_KEYS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(__arm__) && !defined(AI_HOST_BUILD)
#define KEYS_RAMFUNC __attribute__((section(".RamFunc"), noinline))
#else
#define KEYS_RAMFUNC
#endif

/** Copy the board key map into RAM (the ISR must not read Flash). */
void keys_init(void);

bool keys_combo_forbidden(uint8_t combo);

/** Sanitize @p combo[8] in place and return W. @p bad_mask gets bit ch set
 *  for every channel that was forced to 000. */
uint32_t keys_build(uint8_t combo[8], uint8_t* bad_mask);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_KEYS_H */
