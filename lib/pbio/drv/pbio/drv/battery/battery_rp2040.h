// SPDX-License-Identifier: MIT
// Battery/ADC driver header for Pico W / Pico 2 W

#ifndef _PBDRV_BATTERY_RP2040_H_
#define _PBDRV_BATTERY_RP2040_H_

#include <stdint.h>
#include <stdbool.h>

// Initialization
void pbdrv_battery_init(void);

// Voltage (millivolts)
uint16_t pbdrv_battery_get_voltage_mv(void);
uint16_t pbdrv_battery_get_voltage_now(void);  // Pybricks-compatible alias
uint16_t pbdrv_battery_get_raw_adc(void);

// Current (milliamps) - requires external hardware
void pbdrv_battery_configure_current_sense(uint8_t channel, uint16_t sense_mohm);
uint16_t pbdrv_battery_get_current_ma(void);
uint16_t pbdrv_battery_get_current_now(void);  // Pybricks-compatible alias

// Temperature (deci-celsius, using internal sensor)
int16_t pbdrv_battery_get_temperature_dc(void);

// Battery type detection
typedef enum {
    BATTERY_TYPE_UNKNOWN = 0,
    BATTERY_TYPE_USB,
    BATTERY_TYPE_LIPO_1S,
    BATTERY_TYPE_LIPO_2S,
    BATTERY_TYPE_NIMH_4,
    BATTERY_TYPE_NIMH_6,
    BATTERY_TYPE_ALKALINE_4,
    BATTERY_TYPE_ALKALINE_6,
} pbdrv_battery_type_t;

pbdrv_battery_type_t pbdrv_battery_detect_type(void);
uint8_t pbdrv_battery_get_percentage(void);

#endif // _PBDRV_BATTERY_RP2040_H_
