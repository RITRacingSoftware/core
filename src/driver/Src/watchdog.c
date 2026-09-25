#include "watchdog.h"

#include <stdbool.h>

#include <stm32g4xx_hal.h>
#include "core_config.h"

#define MASK(x) ((1<<x)-1)

bool core_watchdog_init(bool window, void (*callback)()) {
    // if (CORE_WATCHDOG_RELOAD_VAL > MASK(6)) return false;
    // WWDG->CR = (0b11 << 6) & (CORE_WATCHDOG_RELOAD_VAL);        // Set activation bit, MSB of counter, and counter value
    // if (CORE_WATCHDOG_PRESCALER > MASK(3)) return false;
    // if (CORE_WATCHDOG_WINDOW_VAL > MASK(6)) return false;
    // WWDG->CFR = (CORE_WATCHDOG_PRESCALER << 11) & ((callback == NULL ? 0 : 1) << 9) & (window ? ((1 << 6) & CORE_WATCHDOG_WINDOW_VAL) : 0);
    
    IWDG->KR = 0xCCCC;                                          // Enable watchdog
    IWDG->KR = 0x5555;                                          // Enable register access
    if (CORE_WATCHDOG_PRESCALER > MASK(3)) return false;        // Maximum 3 bit value
    IWDG->PR = CORE_WATCHDOG_PRESCALER;                         // Set prescaler
    if (CORE_WATCHDOG_RELOAD_VAL > MASK(11)) return false;      // Maximum 11 bit value
    IWDG->RLR = CORE_WATCHDOG_RELOAD_VAL;                       // Load the reload value
    while (IWDG->SR) {}
    if (window) {
        if (CORE_WATCHDOG_WINDOW_VAL > MASK(11)) return false;  // Maximum 11 bit value
        IWDG->WINR = CORE_WATCHDOG_WINDOW_VAL;                  // Set window value, also automatically refreshes counter
    }
    else IWDG->KR = 0xAAAA;                                     // Refresh counter value
    return true;
}

void core_watchdog_refresh() {
    // WWDG->CR = (WWDG->CR & (~MASK(7))) & CORE_WATCHDOG_RELOAD_VAL;
    IWDG->KR = 0xAAAA;
}
