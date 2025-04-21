// Included hard-coded into the mock library because it's acting as the Segger RTT files as well
#pragma once

#define core_RTT_init SEGGER_RTT_Init
#define rprintf(...) SEGGER_RTT_printf(0, __VA_ARGS__)

void SEGGER_RTT_Init();
int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);
