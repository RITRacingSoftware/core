// Included hard-coded into the mock library because it's acting as the Segger RTT files as well
#include "rtt.h"

#pragma weak SEGGER_RTT_Init
#pragma weak SEGGER_RTT_printf 

void SEGGER_RTT_Init() {}

int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...) {}
