// SPDX-License-Identifier: MIT
// pbio driver configuration for Pico W / Pico 2 W (unified)

#ifndef _PBDRVCONFIG_H_
#define _PBDRVCONFIG_H_

// Include DMA PID config
#include "pbdrvconfig_pid_dma.h"

// ============================================================================
// Platform identification
// ============================================================================

#ifdef PICO_2W
    #define PBDRV_CONFIG_PICO2W                        (1)
    #define PBDRV_CONFIG_PICOW                         (0)
    #define PBDRV_CONFIG_RP2350B                       (1)
    #define PBDRV_CONFIG_RP2040B                       (0)
    #define NUM_MOTORS                                 (12)
#else
    #define PBDRV_CONFIG_PICO2W                        (0)
    #define PBDRV_CONFIG_PICOW                         (1)
    #define PBDRV_CONFIG_RP2350B                       (0)
    #define PBDRV_CONFIG_RP2040B                       (1)
    #define NUM_MOTORS                                 (4)
#endif

// ============================================================================
// Motor / Counter / PWM configuration
// ============================================================================

#define PBDRV_CONFIG_MOTOR_DRIVER_NUM_DEV              (NUM_MOTORS)
#define PBDRV_CONFIG_COUNTER_NUM_DEV                   (NUM_MOTORS)
#define PBDRV_CONFIG_PWM_NUM_DEV                       (NUM_MOTORS)

// System clock rate (Hz)
#define PBDRV_CONFIG_SYS_CLOCK_RATE                    (1000)

// ============================================================================
// Watchdog driver
// ============================================================================

#define PBDRV_CONFIG_WATCHDOG                          (1)
#define PBDRV_CONFIG_WATCHDOG_RP2040                   (1)
#define PBDRV_CONFIG_WATCHDOG_TIMEOUT_MS               (8000)

// ============================================================================
// Battery driver
// ============================================================================

#define PBDRV_CONFIG_BATTERY                           (1)
#define PBDRV_CONFIG_BATTERY_RP2040_ADC                (1)
#define PBDRV_CONFIG_BATTERY_ADC_CHANNEL               (3)     // ADC3 = VSYS
#define PBDRV_CONFIG_BATTERY_DIVIDER_RATIO             (3)     // 3:1 divider

// ============================================================================
// I/O Port driver
// ============================================================================

#define PBDRV_CONFIG_IOPORT                            (1)
#define PBDRV_CONFIG_IOPORT_RP2040                     (1)

#ifdef PICO_2W
    #define PBDRV_CONFIG_IOPORT_NUM_PORTS              (6)
#else
    #define PBDRV_CONFIG_IOPORT_NUM_PORTS              (2)
#endif

// ============================================================================
// Internal Flash Storage driver
// ============================================================================

#define PBDRV_CONFIG_STORAGE                           (1)
#define PBDRV_CONFIG_STORAGE_FLASH_RP2040              (1)

// Flash layout (see storage_flash_rp2040.c for details)
#ifdef PICO_2W
    #define PBDRV_CONFIG_STORAGE_FLASH_OFFSET          (0x3C0000)  // 3840KB
    #define PBDRV_CONFIG_STORAGE_FLASH_SIZE            (128 * 1024)
    #define PBDRV_CONFIG_SETTINGS_FLASH_OFFSET         (0x3E0000)  // 3968KB
    #define PBDRV_CONFIG_SETTINGS_FLASH_SIZE           (128 * 1024)
#else
    #define PBDRV_CONFIG_STORAGE_FLASH_OFFSET          (0x1E0000)  // 1920KB
    #define PBDRV_CONFIG_STORAGE_FLASH_SIZE            (64 * 1024)
    #define PBDRV_CONFIG_SETTINGS_FLASH_OFFSET         (0x1F0000)  // 1984KB
    #define PBDRV_CONFIG_SETTINGS_FLASH_SIZE           (64 * 1024)
#endif

// ============================================================================
// SD Card Storage driver (optional)
// ============================================================================

#define PBDRV_CONFIG_STORAGE_SD_RP2040                 (1)

// SPI instance for SD card
#define PBDRV_CONFIG_STORAGE_SD_SPI_INSTANCE           spi0

// Default SPI pins (directly on Pico board)
// These can be overridden by defining before including this header
#ifndef PBDRV_CONFIG_STORAGE_SD_PIN_MISO
    #define PBDRV_CONFIG_STORAGE_SD_PIN_MISO           (16)
#endif

#ifndef PBDRV_CONFIG_STORAGE_SD_PIN_CS
    #define PBDRV_CONFIG_STORAGE_SD_PIN_CS             (17)
#endif

#ifndef PBDRV_CONFIG_STORAGE_SD_PIN_SCK
    #define PBDRV_CONFIG_STORAGE_SD_PIN_SCK            (18)
#endif

#ifndef PBDRV_CONFIG_STORAGE_SD_PIN_MOSI
    #define PBDRV_CONFIG_STORAGE_SD_PIN_MOSI           (19)
#endif

// Card detect pin (0xFF = not used)
#ifndef PBDRV_CONFIG_STORAGE_SD_PIN_CD
    #define PBDRV_CONFIG_STORAGE_SD_PIN_CD             (0xFF)
#endif

// SPI speeds
#define PBDRV_CONFIG_STORAGE_SD_SPI_INIT_FREQ          (400000)     // 400 kHz
#define PBDRV_CONFIG_STORAGE_SD_SPI_FREQ               (25000000)   // 25 MHz

// Storage layout (sector = 512 bytes)
#define PBDRV_CONFIG_STORAGE_SETTINGS_SECTOR           (32)         // 16KB offset
#define PBDRV_CONFIG_STORAGE_PROGRAM_START_SECTOR      (64)         // 32KB offset
#define PBDRV_CONFIG_STORAGE_PROGRAM_MAX_SIZE          (128 * 1024) // 128KB

// ============================================================================
// IMU driver (BNO085)
// ============================================================================

#define PBDRV_CONFIG_IMU                               (1)
#define PBDRV_CONFIG_IMU_BNO085                        (1)

// BNO085 I2C configuration
#ifdef PICO_2W
    #define PBDRV_CONFIG_IMU_I2C_INSTANCE              i2c1
    #define PBDRV_CONFIG_IMU_PIN_SDA                   (14)
    #define PBDRV_CONFIG_IMU_PIN_SCL                   (15)
#else
    #define PBDRV_CONFIG_IMU_I2C_INSTANCE              i2c0
    #define PBDRV_CONFIG_IMU_PIN_SDA                   (4)
    #define PBDRV_CONFIG_IMU_PIN_SCL                   (5)
#endif

#define PBDRV_CONFIG_IMU_I2C_ADDR                      (0x4A)  // Default address
#define PBDRV_CONFIG_IMU_UPDATE_RATE_HZ                (100)   // 100 Hz

// ============================================================================
// Clock driver (uses Pico SDK directly)
// ============================================================================

#define PBDRV_CONFIG_CLOCK                             (1)
#define PBDRV_CONFIG_CLOCK_PICO_SDK                    (1)

// ============================================================================
// GPIO driver (uses Pico SDK directly)
// ============================================================================

#define PBDRV_CONFIG_GPIO                              (1)
#define PBDRV_CONFIG_GPIO_PICO_SDK                     (1)

// ============================================================================
// Reset driver
// ============================================================================

#define PBDRV_CONFIG_RESET                             (1)
#define PBDRV_CONFIG_RESET_RP2040                      (1)

// ============================================================================
// Bluetooth
// ============================================================================

#define PBDRV_CONFIG_BLUETOOTH                         (1)
#define PBDRV_CONFIG_BLUETOOTH_BTSTACK                 (1)

#endif // _PBDRVCONFIG_H_
