#pragma once

#include <stdbool.h>

bool core_watchdog_init(bool window, void (*callback)());
void core_watchdog_refresh();
