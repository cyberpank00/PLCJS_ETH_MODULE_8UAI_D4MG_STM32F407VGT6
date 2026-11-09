/**
  ******************************************************************************
  * @file    app.h
  * @brief   Top-level orchestration: factory-reset detection, settings load,
  *          DI/LED tasks and Modbus TCP server start.
  ******************************************************************************
  */
#ifndef APPLICATION_APP_H
#define APPLICATION_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Application entry point. Must be called from an OS task (uses osDelay()).
 * Configures the network interface, spawns DI / LED / Modbus tasks and then
 * never returns - the caller can either be one of those tasks or be reused
 * as the housekeeping task (e.g. the FreeRTOS default task).
 */
void app_run(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_APP_H */
