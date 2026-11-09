/**
  ******************************************************************************
  * @file    board_pins.c
  * @brief   8UAI board pin set-up (register level, see board_pins.h).
  ******************************************************************************
  */
#include "board_pins.h"
#include "board_io.h"

#include "stm32f4xx.h"

static void pin_mode(GPIO_TypeDef* g, uint8_t pin, uint32_t mode)
{
    g->MODER = (g->MODER & ~(3u << (pin * 2u))) | (mode << (pin * 2u));
}

static void pin_out_pp(GPIO_TypeDef* g, uint8_t pin, uint32_t speed)
{
    g->OTYPER &= ~(1u << pin);
    g->PUPDR  &= ~(3u << (pin * 2u));
    g->OSPEEDR = (g->OSPEEDR & ~(3u << (pin * 2u))) | (speed << (pin * 2u));
    pin_mode(g, pin, 1u);   /* general purpose output */
}

void board_early_pins(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    /* Levels first (BSRR), then the output mode — nOE must never see a 0. */
    BOARD_GPIO(BOARD_NOE_PORT)->BSRR  = 1u << BOARD_NOE_PIN;
    BOARD_GPIO(BOARD_CS_PORT)->BSRR   = 1u << BOARD_CS_PIN;
    BOARD_GPIO(BOARD_RCLK_PORT)->BSRR = 1u << (BOARD_RCLK_PIN + 16u);
    BOARD_GPIO(BOARD_SCLK_PORT)->BSRR = 1u << (BOARD_SCLK_PIN + 16u);

    pin_out_pp(BOARD_GPIO(BOARD_NOE_PORT),  BOARD_NOE_PIN,  2u);
    pin_out_pp(BOARD_GPIO(BOARD_CS_PORT),   BOARD_CS_PIN,   2u);
    pin_out_pp(BOARD_GPIO(BOARD_RCLK_PORT), BOARD_RCLK_PIN, 2u);
    pin_out_pp(BOARD_GPIO(BOARD_SCLK_PORT), BOARD_SCLK_PIN, 2u);
}

static void pin_af(GPIO_TypeDef* g, uint8_t pin, uint8_t af)
{
    volatile uint32_t* afr = &g->AFR[pin >> 3];
    const uint32_t sh = (uint32_t)(pin & 7u) * 4u;
    *afr = (*afr & ~(0xFu << sh)) | ((uint32_t)af << sh);
    g->OTYPER &= ~(1u << pin);
    g->PUPDR  &= ~(3u << (pin * 2u));
    g->OSPEEDR = (g->OSPEEDR & ~(3u << (pin * 2u))) | (2u << (pin * 2u));   /* high speed */
    pin_mode(g, pin, 2u);
}

void board_spi_pins_af(void)
{
    pin_af(BOARD_GPIO(BOARD_SCLK_PORT), BOARD_SCLK_PIN, BOARD_SPI_AF);
    /* Weak pull-down keeps SCLK at 0 while SPE is cleared for a CPHA change. */
    BOARD_GPIO(BOARD_SCLK_PORT)->PUPDR |= 2u << (BOARD_SCLK_PIN * 2u);
    pin_af(BOARD_GPIO(BOARD_MISO_PORT), BOARD_MISO_PIN, BOARD_SPI_AF);
    pin_af(BOARD_GPIO(BOARD_MOSI_PORT), BOARD_MOSI_PIN, BOARD_SPI_AF);
}

void board_led_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOEEN;
    (void)RCC->AHB1ENR;
    for (uint8_t ch = 0u; ch < 8u; ch++) {
        GPIO_TypeDef* g = BOARD_GPIO(BOARD_LED_PORT[ch]);
        g->BSRR = 1u << (BOARD_LED_PIN[ch] + 16u);
        pin_out_pp(g, BOARD_LED_PIN[ch], 0u);
    }
}

void board_led_set(uint8_t ch, uint8_t on)
{
    if (ch >= 8u) { return; }
    BOARD_GPIO(BOARD_LED_PORT[ch])->BSRR =
        1u << (BOARD_LED_PIN[ch] + (on ? 0u : 16u));
}
