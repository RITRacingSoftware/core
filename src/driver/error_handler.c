#include "error_handler.h"
#include "interrupts.h"

void error_handler()
{
    Interrupts_disable();
    while(1){};
}