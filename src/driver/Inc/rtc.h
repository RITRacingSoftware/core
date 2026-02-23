/**
  * @file   rtc.h
  * @brief  Core RTC library
  *
  * The core library component is used to set and get the current time using
  * the internal RTC module.
  */

#ifndef CORE_RTC_H
#define CORE_RTC_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

/**
  * @brief  Initialize the RTC module
  * @retval 1 if the initialization is successful
  * @retval 0 otherwise
  */
bool core_RTC_init();

/**
  * @brief  Get the current RTC time as a `tm` struct (as defined in time.h)
  *
  * When this function is called, the microsecond value is calculated but
  * is not stored in the struct. This value can be retrieved with a 
  * subsequent call to core_RTC_get_usec().
  * @param  tm Pointer to a `tm` struct
  */
void core_RTC_get_time(struct tm *tm);

/**
  * @brief  Set the current RTC time from a `tm` struct (as defined in time.h)
  * @param  tm Pointer to a `tm` struct
  */
void core_RTC_set_time(struct tm *tm);

/**
  * @brief  Get the microsecond value associated with the most recent
  *         core_RTC_get_time() call
  * @return The microsecond value
  */
uint32_t core_RTC_get_usec();

#endif
