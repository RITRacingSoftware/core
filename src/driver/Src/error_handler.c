#include "error_handler.h"
#include "core_config.h"

#include "gpio.h"

#ifndef CORE_ERROR_HANDLER_BLINK_DELAY
#define CORE_ERROR_HANDLER_BLINK_DELAY 200000
#endif

void error_handler()
{
    while(1)
    {
        core_GPIO_toggle_heartbeat();
        for (unsigned long long  i = 0; i < CORE_ERROR_HANDLER_BLINK_DELAY; i++);
    }
}
