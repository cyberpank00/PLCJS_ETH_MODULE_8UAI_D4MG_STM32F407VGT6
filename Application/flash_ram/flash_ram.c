/**
  ******************************************************************************
  * @file    flash_ram.c
  * @brief   Sector erase + word program executed from RAM (see flash_ram.h).
  ******************************************************************************
  */
#include "flash_ram.h"

#include <string.h>

#include "FreeRTOS.h"
#include "board_io.h"
#include "seq.h"
#include "stm32f4xx.h"
#include "task.h"

#define FLASH_ERR_MASK  (FLASH_SR_OPERR | FLASH_SR_WRPERR | FLASH_SR_PGAERR | \
                         FLASH_SR_PGPERR | FLASH_SR_PGSERR)

BOARD_INLINE void flash_wait(void)
{
    while ((FLASH->SR & FLASH_SR_BSY) != 0u) { }
}

/* Runs entirely from RAM: no HAL (HAL_FLASHEx_Erase polls HAL_GetTick, whose
 * tick interrupt is masked here) and no Flash constants. */
BOARD_RAMFUNC static bool ram_erase_program(uint32_t sector, uint32_t addr,
                                            const uint32_t* src, uint32_t words)
{
    flash_wait();
    FLASH->SR = FLASH_SR_EOP | FLASH_ERR_MASK;

    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_SER | (sector << FLASH_CR_SNB_Pos);
    FLASH->CR |= FLASH_CR_STRT;
    flash_wait();
    if ((FLASH->SR & FLASH_ERR_MASK) != 0u) { FLASH->CR = 0u; return false; }

    FLASH->CR = FLASH_CR_PSIZE_1 | FLASH_CR_PG;
    for (uint32_t i = 0u; i < words; i++) {
        *(volatile uint32_t*)(addr + 4u * i) = src[i];
        __DSB();
        flash_wait();
        if ((FLASH->SR & FLASH_ERR_MASK) != 0u) { FLASH->CR = 0u; return false; }
    }
    FLASH->CR = 0u;
    return true;
}

bool flash_ram_write_sector(uint32_t sector, uint32_t addr, const uint32_t* src, uint32_t words)
{
    if ((FLASH->CR & FLASH_CR_LOCK) != 0u) {
        FLASH->KEYR = 0x45670123u;
        FLASH->KEYR = 0xCDEF89ABu;
    }

    seq_flash_window(true);
    taskENTER_CRITICAL();
    const bool ok = ram_erase_program(sector, addr, src, words);
    taskEXIT_CRITICAL();
    seq_flash_window(false);

    FLASH->CR |= FLASH_CR_LOCK;

    /* Drop stale ART data-cache lines of the rewritten sector. */
    FLASH->ACR &= ~FLASH_ACR_DCEN;
    FLASH->ACR |= FLASH_ACR_DCRST;
    FLASH->ACR &= ~FLASH_ACR_DCRST;
    FLASH->ACR |= FLASH_ACR_DCEN;

    return ok && memcmp((const void*)addr, src, words * 4u) == 0;
}
