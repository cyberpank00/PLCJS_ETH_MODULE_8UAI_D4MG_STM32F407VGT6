/**
  ******************************************************************************
  * @file    wdg.c
  * @brief   8UAI watchdog policy (see wdg.h). Register level IWDG.
  ******************************************************************************
  */
#include "wdg.h"

#include "stm32f4xx.h"

#define WDG_TIGHT_PR     0u     /* LSI / 4                     */
#define WDG_TIGHT_RLR    200u   /* 200 / 8 kHz = 25 ms nominal */

static volatile bool s_tight;

void wdg_kick(void)
{
    if (!s_tight) { IWDG->KR = 0xAAAAu; }
}

void wdg_tighten(void)
{
    IWDG->KR = 0xAAAAu;
    IWDG->KR = 0x5555u;
    IWDG->PR  = WDG_TIGHT_PR;
    IWDG->RLR = WDG_TIGHT_RLR;
    while (IWDG->SR != 0u) { }
    IWDG->KR = 0xAAAAu;
    s_tight = true;
}

bool wdg_is_tight(void) { return s_tight; }
