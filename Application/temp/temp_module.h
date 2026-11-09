/**
  ******************************************************************************
  * @file    temp_module.h
  * @brief   On-chip temperature sensor driver (ADC1_IN16).
  *
  *  The STM32F407 internal temperature sensor is wired to ADC1 channel 16.
  *  This module configures ADC1 for a single, software-triggered conversion of
  *  that channel using direct register access (the HAL ADC driver is not part
  *  of this build) and converts the raw reading to degrees Celsius using the
  *  datasheet characteristics (V25 = 0.76 V, Avg_Slope = 2.5 mV/degC).
  ******************************************************************************
  */
#ifndef APPLICATION_TEMP_MODULE_H
#define APPLICATION_TEMP_MODULE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialise ADC1 for the internal temperature sensor (channel 16).
 * Safe to call once during start-up, before the first read.
 */
void temp_module_init(void);

/**
 * Perform a blocking single conversion and return the raw 12-bit ADC count
 * (0..4095). Returns 0 if the conversion times out.
 */
uint16_t temp_module_read_raw(void);

/**
 * Perform a blocking single conversion and return the temperature in tenths
 * of a degree Celsius as a signed value (e.g. 23.5 degC -> 235, -5.0 degC ->
 * -50). The result is clamped to the int16_t range.
 */
int16_t temp_module_read_decicelsius(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_TEMP_MODULE_H */
