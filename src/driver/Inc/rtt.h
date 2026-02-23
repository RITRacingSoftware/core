/**
  * @file   rtt.h
  * @brief  Core RTT wrapper library
  * 
  * This core library component defines a few wrappers for the functions
  * provided by SEGGER for RTT
  */
#include <SEGGER_RTT.h>

#ifndef CORE_RTT_H
#define CORE_RTT_H

/**
  * @brief  Initialize RTT
  */
#define core_RTT_init() SEGGER_RTT_Init()

/**
  * @brief  Print to the RTT console. The argument structure is identical to
  *         the standard `printf` funcion
  */
#define rprintf(...) SEGGER_RTT_printf(0, __VA_ARGS__)

#endif
