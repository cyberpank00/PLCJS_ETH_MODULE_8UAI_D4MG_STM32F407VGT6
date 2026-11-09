/**
  ******************************************************************************
  * @file    button_module.h
  * @brief   FACT_RES button handling.
  *
  *  The push-button is wired between PC8 and GND with internal pull-up, so a
  *  pressed button reads LOW. Holding the button at power-up triggers a
  *  factory reset; the routine below performs the polling synchronously and
  *  is intended to be invoked early during application bring-up.
  ******************************************************************************
  */
#ifndef APPLICATION_BUTTON_MODULE_H
#define APPLICATION_BUTTON_MODULE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Hold time required at boot to trigger a factory reset, ms. */
#define BUTTON_HOLD_FOR_FACTORY_RESET_MS    2000u

/**
 * Returns true if the FACT_RES button is currently pressed (active level).
 */
bool button_is_pressed(void);

/**
 * Block waiting for the FACT_RES button to be held continuously for
 * `hold_ms` (or be released earlier). Returns true if the full hold time
 * elapsed without release.
 *
 * The function expects the OS scheduler to be already running (uses
 * osDelay()). It periodically refreshes the IWDG so that the watchdog does
 * not bite during the wait.
 *
 * @param hold_ms minimum continuous press duration, ms.
 */
bool button_wait_held(uint32_t hold_ms);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_BUTTON_MODULE_H */
