// SPDX-License-Identifier: MIT
// Battery/ADC driver for Pico W / Pico 2 W
// Supports voltage monitoring via ADC pins

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

// ============================================================================
// Configuration
// ============================================================================

// ADC pins available on Pico W:
// GPIO26 = ADC0, GPIO27 = ADC1, GPIO28 = ADC2
// GPIO29 = ADC3 (VSYS/3 on Pico W - internal voltage monitor)

// Default: Use ADC3 (GPIO29) for VSYS voltage monitoring
// VSYS is connected through a 3:1 voltage divider on Pico W
#ifndef PBDRV_BATTERY_ADC_CHANNEL
#define PBDRV_BATTERY_ADC_CHANNEL   3       // ADC3 = VSYS/3
#endif

#ifndef PBDRV_BATTERY_ADC_GPIO
#define PBDRV_BATTERY_ADC_GPIO      29      // GPIO29 for VSYS
#endif

// Voltage divider ratio (VSYS has 3:1 divider on board)
#ifndef PBDRV_BATTERY_DIVIDER_RATIO
#define PBDRV_BATTERY_DIVIDER_RATIO 3
#endif

// ADC reference voltage (3.3V)
#define ADC_VREF_MV         3300

// ADC resolution (12-bit = 4096)
#define ADC_MAX_VALUE       4095

// Number of samples for averaging
#define ADC_NUM_SAMPLES     8

// ============================================================================
// State
// ============================================================================

static bool adc_initialized = false;
static uint16_t last_voltage_mv = 0;
static uint16_t last_raw_adc = 0;

// Optional: external current sense resistor
// Set to 0 if not available
static uint16_t current_sense_mohm = 0;
static uint8_t current_adc_channel = 0xFF;  // Invalid = not configured

// ============================================================================
// Initialization
// ============================================================================

void pbdrv_battery_init(void) {
    if (adc_initialized) return;
    
    adc_init();
    
    // Configure battery voltage ADC pin
    adc_gpio_init(PBDRV_BATTERY_ADC_GPIO);
    
    adc_initialized = true;
}

// ============================================================================
// Voltage Reading
// ============================================================================

static uint16_t read_adc_averaged(uint8_t channel) {
    adc_select_input(channel);
    
    uint32_t sum = 0;
    for (int i = 0; i < ADC_NUM_SAMPLES; i++) {
        sum += adc_read();
    }
    return (uint16_t)(sum / ADC_NUM_SAMPLES);
}

// Get battery/supply voltage in millivolts
uint16_t pbdrv_battery_get_voltage_mv(void) {
    if (!adc_initialized) {
        pbdrv_battery_init();
    }
    
    uint16_t raw = read_adc_averaged(PBDRV_BATTERY_ADC_CHANNEL);
    last_raw_adc = raw;
    
    // Convert to millivolts
    // voltage = (raw / 4095) * 3300 * divider_ratio
    uint32_t voltage = ((uint32_t)raw * ADC_VREF_MV * PBDRV_BATTERY_DIVIDER_RATIO) / ADC_MAX_VALUE;
    
    last_voltage_mv = (uint16_t)voltage;
    return last_voltage_mv;
}

// Pybricks-compatible API (returns voltage in mV)
uint16_t pbdrv_battery_get_voltage_now(void) {
    return pbdrv_battery_get_voltage_mv();
}

// Get raw ADC value (for debugging)
uint16_t pbdrv_battery_get_raw_adc(void) {
    return last_raw_adc;
}

// ============================================================================
// Current Sensing (optional - requires external hardware)
// ============================================================================

// Configure current sensing
// channel: ADC channel connected to current sense amplifier
// sense_mohm: sense resistor value in milliohms
void pbdrv_battery_configure_current_sense(uint8_t channel, uint16_t sense_mohm) {
    if (channel > 3) return;
    
    current_adc_channel = channel;
    current_sense_mohm = sense_mohm;
    
    // Initialize ADC pin if different from voltage pin
    uint8_t gpio = 26 + channel;
    if (gpio != PBDRV_BATTERY_ADC_GPIO) {
        adc_gpio_init(gpio);
    }
}

// Get current in milliamps (returns 0 if not configured)
uint16_t pbdrv_battery_get_current_ma(void) {
    if (current_adc_channel > 3 || current_sense_mohm == 0) {
        return 0;  // Not configured
    }
    
    uint16_t raw = read_adc_averaged(current_adc_channel);
    
    // Assuming current sense amplifier with known gain
    // This is hardware-dependent - adjust for your circuit
    // Example: INA219 or simple op-amp current sense
    // V = I * R, so I = V / R
    // voltage_mv = (raw / 4095) * 3300
    // current_ma = voltage_mv / sense_mohm * 1000
    
    uint32_t voltage_uv = ((uint32_t)raw * ADC_VREF_MV * 1000) / ADC_MAX_VALUE;
    uint32_t current_ma = voltage_uv / current_sense_mohm;
    
    return (uint16_t)current_ma;
}

// Pybricks-compatible API
uint16_t pbdrv_battery_get_current_now(void) {
    return pbdrv_battery_get_current_ma();
}

// ============================================================================
// Temperature (using internal temp sensor)
// ============================================================================

// RP2040 has internal temperature sensor on ADC4
#define TEMP_ADC_CHANNEL 4

int16_t pbdrv_battery_get_temperature_dc(void) {
    // Select internal temp sensor
    adc_set_temp_sensor_enabled(true);
    adc_select_input(TEMP_ADC_CHANNEL);
    
    uint16_t raw = read_adc_averaged(TEMP_ADC_CHANNEL);
    
    adc_set_temp_sensor_enabled(false);
    
    // Convert to temperature
    // T = 27 - (ADC_voltage - 0.706) / 0.001721
    // ADC_voltage = raw * 3.3 / 4095
    
    float voltage = (float)raw * 3.3f / 4095.0f;
    float temp_c = 27.0f - (voltage - 0.706f) / 0.001721f;
    
    // Return in deci-celsius (0.1°C units)
    return (int16_t)(temp_c * 10.0f);
}

// ============================================================================
// Battery Type Detection (for different power sources)
// ============================================================================

typedef enum {
    BATTERY_TYPE_UNKNOWN = 0,
    BATTERY_TYPE_USB,           // ~5V from USB
    BATTERY_TYPE_LIPO_1S,       // 3.7V nominal (3.0-4.2V)
    BATTERY_TYPE_LIPO_2S,       // 7.4V nominal (6.0-8.4V)
    BATTERY_TYPE_NIMH_4,        // 4x NiMH = 4.8V nominal
    BATTERY_TYPE_NIMH_6,        // 6x NiMH = 7.2V nominal
    BATTERY_TYPE_ALKALINE_4,    // 4x AA = 6V nominal
    BATTERY_TYPE_ALKALINE_6,    // 6x AA = 9V nominal
} pbdrv_battery_type_t;

pbdrv_battery_type_t pbdrv_battery_detect_type(void) {
    uint16_t voltage = pbdrv_battery_get_voltage_mv();
    
    if (voltage < 3500) {
        return BATTERY_TYPE_UNKNOWN;  // Too low
    } else if (voltage < 4500) {
        return BATTERY_TYPE_LIPO_1S;
    } else if (voltage < 5500) {
        return BATTERY_TYPE_USB;
    } else if (voltage < 6500) {
        return BATTERY_TYPE_ALKALINE_4;
    } else if (voltage < 8000) {
        return BATTERY_TYPE_LIPO_2S;
    } else if (voltage < 10000) {
        return BATTERY_TYPE_ALKALINE_6;
    }
    
    return BATTERY_TYPE_UNKNOWN;
}

// Get battery percentage (rough estimate based on voltage)
uint8_t pbdrv_battery_get_percentage(void) {
    uint16_t voltage = pbdrv_battery_get_voltage_mv();
    pbdrv_battery_type_t type = pbdrv_battery_detect_type();
    
    uint16_t min_mv, max_mv;
    
    switch (type) {
        case BATTERY_TYPE_USB:
            return 100;  // USB always "full"
        case BATTERY_TYPE_LIPO_1S:
            min_mv = 3000; max_mv = 4200;
            break;
        case BATTERY_TYPE_LIPO_2S:
            min_mv = 6000; max_mv = 8400;
            break;
        default:
            return 50;  // Unknown, assume 50%
    }
    
    if (voltage <= min_mv) return 0;
    if (voltage >= max_mv) return 100;
    
    return (uint8_t)(((uint32_t)(voltage - min_mv) * 100) / (max_mv - min_mv));
}
