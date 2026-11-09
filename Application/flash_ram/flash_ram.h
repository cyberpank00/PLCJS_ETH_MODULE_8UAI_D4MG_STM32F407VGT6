/**
  ******************************************************************************
  * @file    flash_ram.h
  * @brief   Sector erase + word program executed from RAM.
  *
  *  An erase stalls every instruction/data fetch from Flash for 1..4 s. The
  *  8UAI protection ISR must keep running meanwhile (it is the only shunt
  *  protection), so the whole erase/program loop runs from RAM inside a
  *  FreeRTOS critical section: interrupts at or below
  *  configMAX_SYSCALL_INTERRUPT_PRIORITY (ETH, tick, PendSV — all executing
  *  from Flash) are masked, the priority-0 sequencer ISR (in RAM) is not.
  *  The sequencer is told about the window so it keeps feeding the watchdog
  *  while the threads are frozen (seq_flash_window, max 5 s).
  ******************************************************************************
  */
#ifndef APPLICATION_FLASH_RAM_H
#define APPLICATION_FLASH_RAM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Erase @p sector, program @p words 32-bit words from @p src at @p addr,
 *  then verify. Blocks the calling thread (and every other task) for the
 *  duration of the erase. */
bool flash_ram_write_sector(uint32_t sector, uint32_t addr, const uint32_t* src, uint32_t words);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_FLASH_RAM_H */
