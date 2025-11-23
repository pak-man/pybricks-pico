// SPDX-License-Identifier: MIT
// Platform initialization for Pico W / Pico 2 W

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"

void pbdrv_init(void) {
    // Platform initialization
}

void pbdrv_reset(int action) {
    if (action == 1) {
        // BOOTSEL mode
        reset_usb_boot(0, 0);
    } else {
        // Normal reset
        watchdog_reboot(0, 0, 0);
    }
    
    while (1) {
        tight_loop_contents();
    }
}
