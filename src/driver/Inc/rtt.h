#include <SEGGER_RTT.h>

#ifndef CORE_RTT_H
#define CORE_RTT_H

#define core_RTT_init SEGGER_RTT_Init
#define rprintf(...) SEGGER_RTT_printf(0, __VA_ARGS__)

#endif
