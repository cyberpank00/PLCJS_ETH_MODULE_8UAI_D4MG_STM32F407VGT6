/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ETHINT_Pin GPIO_PIN_1
#define ETHINT_GPIO_Port GPIOB
#define ETHRST_Pin GPIO_PIN_11
#define ETHRST_GPIO_Port GPIOD
#define STAT_LED_Pin GPIO_PIN_9
#define STAT_LED_GPIO_Port GPIOE
#define FACT_RES_Pin GPIO_PIN_10
#define FACT_RES_GPIO_Port GPIOE

/* ---- 8AIC analog board (ADS1220 x2, 4 single-ended inputs each) --------- */
/* SPI1: SCLK = PA5, MISO = PA6, MOSI = PB5 (all AF5). Configured in spi_bus.c. */

/* Per-converter chip-select (active-low, idle high) and DRDY (active-low),
 * both through the digital isolators. ADC0 serves AI0..AI3, ADC1 AI4..AI7. */
#define ADC0_CS_Pin         GPIO_PIN_9
#define ADC0_CS_GPIO_Port   GPIOA
#define ADC1_CS_Pin         GPIO_PIN_8
#define ADC1_CS_GPIO_Port   GPIOD
#define ADC0_DRDY_Pin       GPIO_PIN_10
#define ADC0_DRDY_GPIO_Port GPIOD
#define ADC1_DRDY_Pin       GPIO_PIN_9
#define ADC1_DRDY_GPIO_Port GPIOD

/* Per-channel status LED (active-high). */
#define AI0_STAT_Pin        GPIO_PIN_15
#define AI0_STAT_GPIO_Port  GPIOB
#define AI1_STAT_Pin        GPIO_PIN_14
#define AI1_STAT_GPIO_Port  GPIOB
#define AI2_STAT_Pin        GPIO_PIN_10
#define AI2_STAT_GPIO_Port  GPIOB
#define AI3_STAT_Pin        GPIO_PIN_15
#define AI3_STAT_GPIO_Port  GPIOE
#define AI4_STAT_Pin        GPIO_PIN_14
#define AI4_STAT_GPIO_Port  GPIOE
#define AI5_STAT_Pin        GPIO_PIN_13
#define AI5_STAT_GPIO_Port  GPIOE
#define AI6_STAT_Pin        GPIO_PIN_12
#define AI6_STAT_GPIO_Port  GPIOE
#define AI7_STAT_Pin        GPIO_PIN_11
#define AI7_STAT_GPIO_Port  GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
