/**
  * @file   timestamp.h
  * @brief  Core timestamp library
  *
  * This core library component is used to initialize the hardware timestamp
  * counters. The timestamp counter consists of a 16 bit counter (TIM3) which
  * increments every microsecond, and a 32 bit counter which increments
  * whenever the lower 16 bits overflow.
  */

#ifndef CORE_TIMESTAMP_H
#define CORE_TIMESTAMP_H
#include <stdint.h>

/**
  * @brief  Initialize the timestamp timers
  */
void core_timestamp_init();

/**
  * @brief  Read the LSB and the MSB into two separate variables
  */
void core_timestamp_read_separate(uint16_t *lsb, uint32_t *msb);

/**
  * @brief  Read the LSB and MSB and return the full timestamp as a 64 bit
  *         integer
  */
uint64_t core_timestamp_get_tick();

#endif
