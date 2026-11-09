/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level: ETHRST high = KSZ8863 not held in reset.
   * The switch must keep forwarding pass-through traffic across MCU restarts;
   * a cold-boot-only reset is issued from ksz8863_boot_init(). Leaving this at
   * RESET held the switch in reset across warm reboots (dead RJ45 link). */
  HAL_GPIO_WritePin(ETHRST_GPIO_Port, ETHRST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STAT_LED_GPIO_Port, STAT_LED_Pin, GPIO_PIN_RESET);

  /* ADS1220 chip-selects idle high (deselected). */
  HAL_GPIO_WritePin(ADC0_CS_GPIO_Port, ADC0_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(ADC1_CS_GPIO_Port, ADC1_CS_Pin, GPIO_PIN_SET);

  /* Status LEDs default low. */
  HAL_GPIO_WritePin(GPIOB, AI0_STAT_Pin|AI1_STAT_Pin|AI2_STAT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, AI3_STAT_Pin|AI4_STAT_Pin|AI5_STAT_Pin|AI6_STAT_Pin|AI7_STAT_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin : ETHINT_Pin */
  GPIO_InitStruct.Pin = ETHINT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ETHRST_Pin */
  GPIO_InitStruct.Pin = ETHRST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ETHRST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STAT_LED_Pin */
  GPIO_InitStruct.Pin = STAT_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STAT_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FACT_RES_Pin */
  GPIO_InitStruct.Pin = FACT_RES_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FACT_RES_GPIO_Port, &GPIO_InitStruct);

  /* ADS1220 chip-selects (production pinout): ADC0_CS = PA9, ADC1_CS = PD8. */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = ADC0_CS_Pin;
  HAL_GPIO_Init(ADC0_CS_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = ADC1_CS_Pin;
  HAL_GPIO_Init(ADC1_CS_GPIO_Port, &GPIO_InitStruct);

  /* ADS1220 DRDY inputs (active-low, driven through the isolators): PD10, PD9. */
  GPIO_InitStruct.Pin = ADC0_DRDY_Pin|ADC1_DRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* Channel status LEDs: AI0..AI2 on port B (PB15, PB14, PB10), AI3..AI7 on
   * port E (PE15..PE11). */
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  GPIO_InitStruct.Pin = AI0_STAT_Pin|AI1_STAT_Pin|AI2_STAT_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = AI3_STAT_Pin|AI4_STAT_Pin|AI5_STAT_Pin|AI6_STAT_Pin|AI7_STAT_Pin;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
