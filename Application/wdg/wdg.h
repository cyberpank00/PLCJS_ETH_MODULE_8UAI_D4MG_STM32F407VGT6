/**
  ******************************************************************************
  * @file    wdg.h
  * @brief   8UAI watchdog policy (TZ section 9).
  *
  *  Boot: MX_IWDG_Init() starts the IWDG at ~8 s like every module of the
  *  family (it also takes over a counter left running by the bootloader), and
  *  threads refresh it through wdg_kick().
  *  Once the sequencer runs, wdg_tighten() reprograms it to /4, reload 200 =
  *  25 ms nominal (17..47 ms over the LSI spread, <= 50 ms) and from then on
  *  ONLY the protection ISR refreshes it, and only while the AI and
  *  housekeeping loops are alive (see seq.h). wdg_kick() becomes a no-op;
  *  threads report liveness with seq_alive_ai() / seq_alive_hk().
  ******************************************************************************
  */
#ifndef APPLICATION_WDG_H
#define APPLICATION_WDG_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void wdg_kick(void);
void wdg_tighten(void);
bool wdg_is_tight(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_WDG_H */
