#include "error_handler.h"
#include "core_config.h"

#include "gpio.h"

void error_handler()
{
    while(1)
    {
        core_GPIO_set_heartbeat(true);
        for (unsigned long long  i = 0; i < 2000000; i++);
        core_GPIO_set_heartbeat(false);
        for (unsigned long long  i = 0; i < 500000; i++);
    }
}