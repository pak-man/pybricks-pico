#include <pbdrv/config.h>

#if PBDRV_CONFIG_CLOCK_RP2350

#include <stdint.h>
#include "pico/stdlib.h"

uint32_t pbdrv_clock_get_us(void) {
    return to_us_since_boot(get_absolute_time());
}

uint32_t pbdrv_clock_get_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

uint32_t pbdrv_clock_get_100us(void) {
    return pbdrv_clock_get_us() / 100;
}

#endif // PBDRV_CONFIG_CLOCK_RP2350