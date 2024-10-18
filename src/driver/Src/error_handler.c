#include "error_handler.h"
#include "gpio.h"

void error_handler()
{
    while(1)
    {
        core_GPIO_toggle_heartbeat();
        for (unsigned long long  i = 0; i < 200000; i++);
    }
}