// SPDX-License-Identifier: MIT
// I/O Port driver for Pico W / Pico 2 W
// Supports I2C sensors, UART communication, and port detection

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"

// ============================================================================
// Configuration
// ============================================================================

#ifdef PICO_2W
    #define IOPORT_NUM_PORTS    6   // More ports on Pico 2 W
#else
    #define IOPORT_NUM_PORTS    2   // 2 I/O ports on Pico W
#endif

// Port pin assignments (I2C + UART per port)
// Each port has: SDA, SCL (I2C) and TX, RX (UART)
typedef struct {
    uint8_t i2c_sda;
    uint8_t i2c_scl;
    uint8_t uart_tx;
    uint8_t uart_rx;
    i2c_inst_t *i2c;
    uart_inst_t *uart;
    uint8_t id_pin;     // Optional: for device ID detection
} ioport_hw_config_t;

static const ioport_hw_config_t port_hw_config[IOPORT_NUM_PORTS] = {
#ifdef PICO_2W
    // Pico 2 W: 6 ports using extended GPIO
    { .i2c_sda = 0,  .i2c_scl = 1,  .uart_tx = 0,  .uart_rx = 1,  .i2c = i2c0, .uart = uart0, .id_pin = 26 },
    { .i2c_sda = 2,  .i2c_scl = 3,  .uart_tx = 4,  .uart_rx = 5,  .i2c = i2c1, .uart = uart1, .id_pin = 27 },
    { .i2c_sda = 6,  .i2c_scl = 7,  .uart_tx = 8,  .uart_rx = 9,  .i2c = i2c0, .uart = uart0, .id_pin = 28 },
    { .i2c_sda = 10, .i2c_scl = 11, .uart_tx = 12, .uart_rx = 13, .i2c = i2c1, .uart = uart1, .id_pin = 29 },
    { .i2c_sda = 14, .i2c_scl = 15, .uart_tx = 16, .uart_rx = 17, .i2c = i2c0, .uart = uart0, .id_pin = 26 },
    { .i2c_sda = 18, .i2c_scl = 19, .uart_tx = 20, .uart_rx = 21, .i2c = i2c1, .uart = uart1, .id_pin = 27 },
#else
    // Pico W: 2 I/O ports
    { .i2c_sda = 20, .i2c_scl = 21, .uart_tx = 0,  .uart_rx = 1,  .i2c = i2c0, .uart = uart0, .id_pin = 26 },
    { .i2c_sda = 18, .i2c_scl = 19, .uart_tx = 4,  .uart_rx = 5,  .i2c = i2c1, .uart = uart1, .id_pin = 27 },
#endif
};

// ============================================================================
// Port State
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

typedef struct {
    ioport_mode_t mode;
    ioport_device_t device;
    uint8_t i2c_addr;
    uint32_t uart_baud;
    bool initialized;
} ioport_state_t;

static ioport_state_t port_state[IOPORT_NUM_PORTS];

// Forward declaration
void pbdrv_ioport_deinit(uint8_t port);

// ============================================================================
// Initialization
// ============================================================================

void pbdrv_ioport_init(void) {
    memset(port_state, 0, sizeof(port_state));
}

// ============================================================================
// I2C Functions
// ============================================================================

int pbdrv_ioport_i2c_init(uint8_t port, uint32_t baudrate) {
    if (port >= IOPORT_NUM_PORTS) return -1;
    
    const ioport_hw_config_t *hw = &port_hw_config[port];
    ioport_state_t *state = &port_state[port];
    
    // Deinit if already in different mode
    if (state->mode != IOPORT_MODE_NONE && state->mode != IOPORT_MODE_I2C) {
        pbdrv_ioport_deinit(port);
    }
    
    // Initialize I2C
    i2c_init(hw->i2c, baudrate);
    gpio_set_function(hw->i2c_sda, GPIO_FUNC_I2C);
    gpio_set_function(hw->i2c_scl, GPIO_FUNC_I2C);
    gpio_pull_up(hw->i2c_sda);
    gpio_pull_up(hw->i2c_scl);
    
    state->mode = IOPORT_MODE_I2C;
    state->initialized = true;
    
    return 0;
}

int pbdrv_ioport_i2c_write(uint8_t port, uint8_t addr, const uint8_t *data, size_t len) {
    if (port >= IOPORT_NUM_PORTS) return -1;
    if (port_state[port].mode != IOPORT_MODE_I2C) return -2;
    
    const ioport_hw_config_t *hw = &port_hw_config[port];
    int result = i2c_write_blocking(hw->i2c, addr, data, len, false);
    return (result == PICO_ERROR_GENERIC) ? -3 : result;
}

int pbdrv_ioport_i2c_read(uint8_t port, uint8_t addr, uint8_t *data, size_t len) {
    if (port >= IOPORT_NUM_PORTS) return -1;
    if (port_state[port].mode != IOPORT_MODE_I2C) return -2;
    
    const ioport_hw_config_t *hw = &port_hw_config[port];
    int result = i2c_read_blocking(hw->i2c, addr, data, len, false);
    return (result == PICO_ERROR_GENERIC) ? -3 : result;
}

int pbdrv_ioport_i2c_write_reg(uint8_t port, uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return pbdrv_ioport_i2c_write(port, addr, buf, 2);
}

int pbdrv_ioport_i2c_read_reg(uint8_t port, uint8_t addr, uint8_t reg, uint8_t *value) {
    int result = pbdrv_ioport_i2c_write(port, addr, &reg, 1);
    if (result < 0) return result;
    return pbdrv_ioport_i2c_read(port, addr, value, 1);
}

int pbdrv_ioport_i2c_read_reg16(uint8_t port, uint8_t addr, uint8_t reg, uint16_t *value) {
    uint8_t buf[2];
    int result = pbdrv_ioport_i2c_write(port, addr, &reg, 1);
    if (result < 0) return result;
    result = pbdrv_ioport_i2c_read(port, addr, buf, 2);
    if (result < 0) return result;
    *value = (buf[0] << 8) | buf[1];  // Big endian
    return 0;
}

// Scan I2C bus for devices
int pbdrv_ioport_i2c_scan(uint8_t port, uint8_t *addresses, uint8_t max_devices) {
    if (port >= IOPORT_NUM_PORTS) return -1;
    if (port_state[port].mode != IOPORT_MODE_I2C) return -2;
    
    const ioport_hw_config_t *hw = &port_hw_config[port];
    int found = 0;
    uint8_t dummy;
    
    for (uint8_t addr = 0x08; addr < 0x78 && found < max_devices; addr++) {
        int result = i2c_read_blocking(hw->i2c, addr, &dummy, 1, false);
        if (result >= 0) {
            addresses[found++] = addr;
        }
    }
    
    return found;
}

// ============================================================================
// UART Functions
// ============================================================================

int pbdrv_ioport_uart_init(uint8_t port, uint32_t baudrate) {
    if (port >= IOPORT_NUM_PORTS) return -1;
    
    const ioport_hw_config_t *hw = &port_hw_config[port];
    ioport_state_t *state = &port_state[port];
    
    // Deinit if already in different mode
    if (state->mode != IOPORT_MODE_NONE && state->mode != IOPORT_MODE_UART) {
        pbdrv_ioport_deinit(port);
    }
    
    // Initialize UART
    uart_init(hw->uart, baudrate);
    gpio_set_function(hw->uart_tx, GPIO_FUNC_UART);
    gpio_set_function(hw->uart_rx, GPIO_FUNC_UART);
    
    // Default: 8N1
    uart_set_format(hw->uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(hw->uart, true);
    
    state->mode = IOPORT_MODE_UART;
    state->uart_baud = baudrate;
    state->initialized = true;
    
    return 0;
}

int pbdrv_ioport_uart_write(uint8_t port, const uint8_t *data, size_t len) {
    if (port >= IOPORT_NUM_PORTS) return -1;
    if (port_state[port].mode != IOPORT_MODE_UART) return -2;
    
    const ioport_hw_config_t *hw = &port_hw_config[port];
    uart_write_blocking(hw->uart, data, len);
    return len;
}

int pbdrv_ioport_uart_read(uint8_t port, uint8_t *data, size_t len, uint32_t timeout_us) {
    if (port >= IOPORT_NUM_PORTS) return -1;
    if (port_state[port].mode != IOPORT_MODE_UART) return -2;
    
    const ioport_hw_config_t *hw = &port_hw_config[port];
    size_t received = 0;
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    
    while (received < len) {
        if (uart_is_readable(hw->uart)) {
            data[received++] = uart_getc(hw->uart);
        } else if (time_reached(deadline)) {
            break;  // Timeout
        }
    }
    
    return received;
}

bool pbdrv_ioport_uart_readable(uint8_t port) {
    if (port >= IOPORT_NUM_PORTS) return false;
    if (port_state[port].mode != IOPORT_MODE_UART) return false;
    return uart_is_readable(port_hw_config[port].uart);
}

int pbdrv_ioport_uart_flush(uint8_t port) {
    if (port >= IOPORT_NUM_PORTS) return -1;
    if (port_state[port].mode != IOPORT_MODE_UART) return -2;
    
    const ioport_hw_config_t *hw = &port_hw_config[port];
    while (uart_is_readable(hw->uart)) {
        uart_getc(hw->uart);
    }
    return 0;
}

// ============================================================================
// Device Detection
// ============================================================================

// Known I2C addresses for common sensors
#define I2C_ADDR_VL53L0X        0x29    // Distance sensor
#define I2C_ADDR_TCS34725       0x29    // Color sensor (same as VL53L0X!)
#define I2C_ADDR_BNO055         0x28    // IMU
#define I2C_ADDR_MPU6050        0x68    // IMU
#define I2C_ADDR_INA219         0x40    // Current sensor
#define I2C_ADDR_PCA9685        0x40    // PWM driver

ioport_device_t pbdrv_ioport_detect_device(uint8_t port) {
    if (port >= IOPORT_NUM_PORTS) return IOPORT_DEVICE_NONE;
    
    // First try I2C detection
    if (pbdrv_ioport_i2c_init(port, 100000) == 0) {
        uint8_t addresses[8];
        int found = pbdrv_ioport_i2c_scan(port, addresses, 8);
        
        if (found > 0) {
            // Check known addresses
            for (int i = 0; i < found; i++) {
                switch (addresses[i]) {
                    case I2C_ADDR_VL53L0X:
                        // Could be VL53L0X or TCS34725 - need to read ID register
                        port_state[port].device = IOPORT_DEVICE_DISTANCE_SENSOR;
                        port_state[port].i2c_addr = addresses[i];
                        return IOPORT_DEVICE_DISTANCE_SENSOR;
                    
                    case I2C_ADDR_BNO055:
                    case I2C_ADDR_MPU6050:
                        port_state[port].device = IOPORT_DEVICE_CUSTOM_I2C;
                        port_state[port].i2c_addr = addresses[i];
                        return IOPORT_DEVICE_CUSTOM_I2C;
                    
                    default:
                        port_state[port].device = IOPORT_DEVICE_CUSTOM_I2C;
                        port_state[port].i2c_addr = addresses[i];
                        return IOPORT_DEVICE_CUSTOM_I2C;
                }
            }
        }
    }
    
    // Try UART detection (send probe, check response)
    if (pbdrv_ioport_uart_init(port, 115200) == 0) {
        // Send probe byte and check for response
        uint8_t probe = 0x00;
        uint8_t response;
        
        pbdrv_ioport_uart_flush(port);
        pbdrv_ioport_uart_write(port, &probe, 1);
        
        if (pbdrv_ioport_uart_read(port, &response, 1, 10000) > 0) {
            port_state[port].device = IOPORT_DEVICE_CUSTOM_UART;
            return IOPORT_DEVICE_CUSTOM_UART;
        }
    }
    
    return IOPORT_DEVICE_NONE;
}

// ============================================================================
// Port Management
// ============================================================================

void pbdrv_ioport_deinit(uint8_t port) {
    if (port >= IOPORT_NUM_PORTS) return;
    
    const ioport_hw_config_t *hw = &port_hw_config[port];
    ioport_state_t *state = &port_state[port];
    
    switch (state->mode) {
        case IOPORT_MODE_I2C:
            i2c_deinit(hw->i2c);
            gpio_set_function(hw->i2c_sda, GPIO_FUNC_NULL);
            gpio_set_function(hw->i2c_scl, GPIO_FUNC_NULL);
            break;
        
        case IOPORT_MODE_UART:
            uart_deinit(hw->uart);
            gpio_set_function(hw->uart_tx, GPIO_FUNC_NULL);
            gpio_set_function(hw->uart_rx, GPIO_FUNC_NULL);
            break;
        
        default:
            break;
    }
    
    state->mode = IOPORT_MODE_NONE;
    state->device = IOPORT_DEVICE_NONE;
    state->initialized = false;
}

ioport_mode_t pbdrv_ioport_get_mode(uint8_t port) {
    if (port >= IOPORT_NUM_PORTS) return IOPORT_MODE_NONE;
    return port_state[port].mode;
}

ioport_device_t pbdrv_ioport_get_device(uint8_t port) {
    if (port >= IOPORT_NUM_PORTS) return IOPORT_DEVICE_NONE;
    return port_state[port].device;
}

uint8_t pbdrv_ioport_get_i2c_addr(uint8_t port) {
    if (port >= IOPORT_NUM_PORTS) return 0;
    return port_state[port].i2c_addr;
}

// ============================================================================
// Debug
// ============================================================================

void pbdrv_ioport_print_status(void) {
    printf("I/O Port Status (%d ports):\n", IOPORT_NUM_PORTS);
    
    for (int i = 0; i < IOPORT_NUM_PORTS; i++) {
        const char *mode_str = "NONE";
        switch (port_state[i].mode) {
            case IOPORT_MODE_I2C:  mode_str = "I2C";  break;
            case IOPORT_MODE_UART: mode_str = "UART"; break;
            case IOPORT_MODE_GPIO: mode_str = "GPIO"; break;
            default: break;
        }
        
        printf("  Port %d: %s", i, mode_str);
        if (port_state[i].device != IOPORT_DEVICE_NONE) {
            printf(" (device=%d, addr=0x%02x)", port_state[i].device, port_state[i].i2c_addr);
        }
        printf("\n");
    }
}
