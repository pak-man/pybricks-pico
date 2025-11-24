// SPDX-License-Identifier: MIT
// I/O Port driver header for Pico W / Pico 2 W

#ifndef _PBDRV_IOPORT_RP2040_H_
#define _PBDRV_IOPORT_RP2040_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ============================================================================
// Port Configuration
// ============================================================================

#ifdef PICO_2W
    #define IOPORT_NUM_PORTS    6
#else
    #define IOPORT_NUM_PORTS    2
#endif

// ============================================================================
// Types
// ============================================================================

typedef enum {
    IOPORT_MODE_NONE = 0,
    IOPORT_MODE_I2C,
    IOPORT_MODE_UART,
    IOPORT_MODE_GPIO,
} ioport_mode_t;

typedef enum {
    IOPORT_DEVICE_NONE = 0,
    IOPORT_DEVICE_UNKNOWN,
    IOPORT_DEVICE_MOTOR,
    IOPORT_DEVICE_COLOR_SENSOR,
    IOPORT_DEVICE_DISTANCE_SENSOR,
    IOPORT_DEVICE_FORCE_SENSOR,
    IOPORT_DEVICE_LIGHT,
    IOPORT_DEVICE_CUSTOM_I2C,
    IOPORT_DEVICE_CUSTOM_UART,
} ioport_device_t;

// ============================================================================
// Initialization
// ============================================================================

void pbdrv_ioport_init(void);
void pbdrv_ioport_deinit(uint8_t port);

// ============================================================================
// I2C Functions
// ============================================================================

int pbdrv_ioport_i2c_init(uint8_t port, uint32_t baudrate);
int pbdrv_ioport_i2c_write(uint8_t port, uint8_t addr, const uint8_t *data, size_t len);
int pbdrv_ioport_i2c_read(uint8_t port, uint8_t addr, uint8_t *data, size_t len);
int pbdrv_ioport_i2c_write_reg(uint8_t port, uint8_t addr, uint8_t reg, uint8_t value);
int pbdrv_ioport_i2c_read_reg(uint8_t port, uint8_t addr, uint8_t reg, uint8_t *value);
int pbdrv_ioport_i2c_read_reg16(uint8_t port, uint8_t addr, uint8_t reg, uint16_t *value);
int pbdrv_ioport_i2c_scan(uint8_t port, uint8_t *addresses, uint8_t max_devices);

// ============================================================================
// UART Functions
// ============================================================================

int pbdrv_ioport_uart_init(uint8_t port, uint32_t baudrate);
int pbdrv_ioport_uart_write(uint8_t port, const uint8_t *data, size_t len);
int pbdrv_ioport_uart_read(uint8_t port, uint8_t *data, size_t len, uint32_t timeout_us);
bool pbdrv_ioport_uart_readable(uint8_t port);
int pbdrv_ioport_uart_flush(uint8_t port);

// ============================================================================
// Device Detection
// ============================================================================

ioport_device_t pbdrv_ioport_detect_device(uint8_t port);
ioport_mode_t pbdrv_ioport_get_mode(uint8_t port);
ioport_device_t pbdrv_ioport_get_device(uint8_t port);
uint8_t pbdrv_ioport_get_i2c_addr(uint8_t port);

// ============================================================================
// Debug
// ============================================================================

void pbdrv_ioport_print_status(void);

#endif // _PBDRV_IOPORT_RP2040_H_
