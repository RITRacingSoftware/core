#include "error_handler.h"
#include "interrupts.h"
#include "gpio.h"

void error_handler()
{
    Interrupts_disable();
    while(1)
    {
        GPIO_toggle_heartbeat();
        for (unsigned long long  i = 0; i < 200000; i++);
    }
}