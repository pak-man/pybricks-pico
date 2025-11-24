// SPDX-License-Identifier: MIT
// Watchdog driver header for Pico W / Pico 2 W

#ifndef _PBDRV_WATCHDOG_RP2040_H_
#define _PBDRV_WATCHDOG_RP2040_H_

#include <stdbool.h>

void pbdrv_watchdog_init(void);
void pbdrv_watchdog_update(void);
bool pbdrv_watchdog_caused_reboot(void);
void pbdrv_watchdog_disable(void);

#endif // _PBDRV_WATCHDOG_RP2040_H_
