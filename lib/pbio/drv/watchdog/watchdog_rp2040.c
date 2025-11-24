// SPDX-License-Identifier: MIT
// Watchdog driver for Pico W / Pico 2 W

#include "pico/stdlib.h"
#include "hardware/watchdog.h"

#define WATCHDOG_TIMEOUT_MS 8000  // 8 second timeout

static bool watchdog_initialized = false;

void pbdrv_watchdog_init(void) {
    if (watchdog_initialized) return;
    
    // Enable watchdog with 8 second timeout
    // pause_on_debug = true so debugging doesn't trigger reset
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    watchdog_initialized = true;
}

void pbdrv_watchdog_update(void) {
    if (watchdog_initialized) {
        watchdog_update();
    }
}

bool pbdrv_watchdog_caused_reboot(void) {
    return watchdog_caused_reboot();
}

void pbdrv_watchdog_disable(void) {
    // Note: RP2040 watchdog cannot be disabled once enabled
    // This is a no-op but provided for API compatibility
}
