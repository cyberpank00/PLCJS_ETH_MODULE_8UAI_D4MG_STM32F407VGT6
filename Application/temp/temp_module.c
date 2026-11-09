/**
  ******************************************************************************
  * @file    temp_module.c
  * @brief   On-chip temperature sensor driver (ADC1_IN16), bare-register based.
  ******************************************************************************
  */

#include "temp_module.h"

#include "stm32f4xx_hal.h"

/* ADC1 channel the on-chip temperature sensor is connected to on STM32F40x. */
#define TEMP_ADC_CHANNEL        16u

/* Reference supply and resolution used for the raw->voltage conversion. */
#define TEMP_VREF_MV            3300    /* VDDA, millivolts                    */
#define TEMP_ADC_FULL_SCALE     4096    /* 12-bit conversion                   */

/* STM32F40x datasheet temperature-sensor characteristics. */
#define TEMP_V25_MV             760     /* sensor voltage at 25 degC, mV       */
#define TEMP_AVG_SLOPE_UV       2500    /* average slope, microvolts per degC  */

/* Conversion completion guard (busy-loop iterations). */
#define TEMP_CONV_TIMEOUT       100000u

static volatile uint8_t s_initialised = 0u;

void temp_module_init(void)
{
    /* Enable the ADC1 peripheral clock. */
    __HAL_RCC_ADC1_CLK_ENABLE();

    /* ADC common configuration: prescaler /4 (APB2 is 84 MHz -> 21 MHz ADCCLK,
     * within the 36 MHz maximum) and enable the temperature-sensor/VREFINT
     * path. ADC->CCR is the ADC123 common register block. */
    ADC->CCR = (ADC->CCR & ~ADC_CCR_ADCPRE) | ADC_CCR_ADCPRE_0; /* /4 */
    ADC->CCR |= ADC_CCR_TSVREFE;

    /* 12-bit resolution, single channel (no scan). */
    ADC1->CR1 &= ~ADC_CR1_RES;        /* RES = 00 -> 12-bit */

    /* Right alignment, single (non-continuous), software trigger. */
    ADC1->CR2 &= ~(ADC_CR2_ALIGN | ADC_CR2_CONT);

    /* Longest sample time for channel 16: the sensor needs a long acquisition
     * window. SMPR1 holds SMP10..SMP18; set SMP16 = 111b (480 cycles). */
    ADC1->SMPR1 |= ADC_SMPR1_SMP16;

    /* Single conversion in the regular sequence: length 1, first rank = ch16. */
    ADC1->SQR1 &= ~ADC_SQR1_L;        /* L = 0 -> 1 conversion */
    ADC1->SQR3 = (ADC1->SQR3 & ~ADC_SQR3_SQ1) | (TEMP_ADC_CHANNEL << ADC_SQR3_SQ1_Pos);

    /* Power up the ADC and let it stabilise. */
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile uint32_t i = 0; i < 10000u; i++) {
        __NOP();
    }

    s_initialised = 1u;
}

uint16_t temp_module_read_raw(void)
{
    if (!s_initialised) {
        temp_module_init();
    }

    /* Clear any stale end-of-conversion flag and start a conversion. */
    ADC1->SR &= ~ADC_SR_EOC;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    uint32_t guard = TEMP_CONV_TIMEOUT;
    while (((ADC1->SR & ADC_SR_EOC) == 0u) && (guard-- != 0u)) {
        __NOP();
    }
    if (guard == 0u) {
        return 0u;
    }

    return (uint16_t)(ADC1->DR & 0x0FFFu);
}

int16_t temp_module_read_decicelsius(void)
{
    const uint16_t raw = temp_module_read_raw();

    /* Convert the raw count to the sensor voltage in microvolts to keep the
     * arithmetic in integers:  v_uv = raw * VREF_mV * 1000 / full_scale. */
    const int32_t v_uv = (int32_t)(((int64_t)raw * TEMP_VREF_MV * 1000) / TEMP_ADC_FULL_SCALE);

    /* Temperature in tenths of a degree:
     *   T_degC = (v_sense - V25) / slope + 25
     *   T_dC   = (v_uv - V25_uv) * 10 / slope_uv + 250  */
    const int32_t v25_uv = TEMP_V25_MV * 1000;
    int32_t t_dc = ((v_uv - v25_uv) * 10) / TEMP_AVG_SLOPE_UV + 250;

    if (t_dc > 32767) {
        t_dc = 32767;
    } else if (t_dc < -32768) {
        t_dc = -32768;
    }
    return (int16_t)t_dc;
}
