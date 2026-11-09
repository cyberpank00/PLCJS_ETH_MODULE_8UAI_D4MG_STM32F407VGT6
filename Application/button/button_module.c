/**
  ******************************************************************************
  * @file    button_module.c
  * @brief   FACT_RES button handling.
  ******************************************************************************
  */

#include "button_module.h"

#include "cmsis_os.h"
#include "iwdg.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#define BUTTON_POLL_STEP_MS     10u

bool button_is_pressed(void)
{
    return HAL_GPIO_ReadPin(FACT_RES_GPIO_Port, FACT_RES_Pin) == GPIO_PIN_RESET;
}

bool button_wait_held(uint32_t hold_ms)
{
    if (!button_is_pressed()) {
        return false;
    }

    uint32_t elapsed = 0u;
    while (elapsed < hold_ms) {
        if (!button_is_pressed()) {
            return false;
        }
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(BUTTON_POLL_STEP_MS);
        elapsed += BUTTON_POLL_STEP_MS;
    }
    return true;
}
