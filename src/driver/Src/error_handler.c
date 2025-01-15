#include "error_handler.h"
#include "core_config.h"

#include "gpio.h"

#ifndef ERROR_HANDLER_BLINK_DELAY
#define ERROR_HANDLER_BLINK_DELAY 200000
#endif

void error_handler()
{
    while(1)
    {
        core_GPIO_toggle_heartbeat();
        for (unsigned long long  i = 0; i < ERROR_HANDLER_BLINK_DELAY; i++);
    }
}
