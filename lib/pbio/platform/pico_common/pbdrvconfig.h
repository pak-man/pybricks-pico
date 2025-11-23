// SPDX-License-Identifier: MIT
// Driver configuration for Pico W / Pico 2 W

#ifndef _PBDRVCONFIG_H_
#define _PBDRVCONFIG_H_

#include "pbdrvconfig_pid_dma.h"

// ============================================================================
// Platform identification
// ============================================================================
#ifdef PICO_2W
    #define PBDRV_CONFIG_PICO2W     (1)
    #define PBDRV_CONFIG_PICOW      (0)
    #define PBDRV_CONFIG_RP2350B    (1)
    #define PBDRV_CONFIG_RP2040B    (0)
    #define NUM_MOTORS              (12)
#else
    #define PBDRV_CONFIG_PICOW      (1)
    #define PBDRV_CONFIG_PICO2W     (0)
    #define PBDRV_CONFIG_RP2040B    (1)
    #define PBDRV_CONFIG_RP2350B    (0)
    #define NUM_MOTORS              (4)
#endif

// ============================================================================
// Hardware Peripherals
// ============================================================================

// Clock driver
#define PBDRV_CONFIG_CLOCK                  (1)
#ifdef PICO_2W
    #define PBDRV_CONFIG_CLOCK_RP2350       (1)
    #define PBDRV_CONFIG_CLOCK_RP2040       (0)
#else
    #define PBDRV_CONFIG_CLOCK_RP2040       (1)
    #define PBDRV_CONFIG_CLOCK_RP2350       (0)
#endif

// Counter driver (encoders)
#define PBDRV_CONFIG_COUNTER                (1)
#define PBDRV_CONFIG_COUNTER_NUM_DEV        (NUM_MOTORS)
#ifdef PICO_2W
    #define PBDRV_CONFIG_COUNTER_RP2350_PIO (1)
    #define PBDRV_CONFIG_COUNTER_RP2040_PIO (0)
#else
    #define PBDRV_CONFIG_COUNTER_RP2040_PIO (1)
    #define PBDRV_CONFIG_COUNTER_RP2350_PIO (0)
#endif

// PWM driver
#define PBDRV_CONFIG_PWM                    (1)
#define PBDRV_CONFIG_PWM_NUM_DEV            (NUM_MOTORS)
#ifdef PICO_2W
    #define PBDRV_CONFIG_PWM_RP2350_PIO     (1)
    #define PBDRV_CONFIG_PWM_RP2040_PIO     (0)
#else
    #define PBDRV_CONFIG_PWM_RP2040_PIO     (1)
    #define PBDRV_CONFIG_PWM_RP2350_PIO     (0)
#endif

// GPIO driver
#define PBDRV_CONFIG_GPIO                   (1)
#ifdef PICO_2W
    #define PBDRV_CONFIG_GPIO_RP2350        (1)
    #define PBDRV_CONFIG_GPIO_RP2040        (0)
#else
    #define PBDRV_CONFIG_GPIO_RP2040        (1)
    #define PBDRV_CONFIG_GPIO_RP2350        (0)
#endif

// Reset driver
#define PBDRV_CONFIG_RESET                  (1)
#ifdef PICO_2W
    #define PBDRV_CONFIG_RESET_RP2350       (1)
    #define PBDRV_CONFIG_RESET_RP2040       (0)
#else
    #define PBDRV_CONFIG_RESET_RP2040       (1)
    #define PBDRV_CONFIG_RESET_RP2350       (0)
#endif

// ============================================================================
// Motor Control
// ============================================================================

#define PBDRV_CONFIG_MOTOR_DRIVER           (1)
#define PBDRV_CONFIG_MOTOR_DRIVER_NUM_DEV   (NUM_MOTORS)

#define PBDRV_CONFIG_DCMOTOR                (1)
#define PBDRV_CONFIG_DCMOTOR_NUM_DEV        (NUM_MOTORS)

#define PBDRV_CONFIG_TACHO                  (1)
#define PBDRV_CONFIG_TACHO_NUM_DEV          (NUM_MOTORS)

#define PBDRV_CONFIG_SERVO                  (1)
#define PBDRV_CONFIG_SERVO_NUM_DEV          (NUM_MOTORS)

// ============================================================================
// Communication
// ============================================================================

#define PBDRV_CONFIG_BLUETOOTH                              (1)
#define PBDRV_CONFIG_BLUETOOTH_BTSTACK                      (1)
#define PBDRV_CONFIG_BLUETOOTH_BTSTACK_HCI_TRANSPORT_CYW43  (1)

// ============================================================================
// System
// ============================================================================

#define PBDRV_CONFIG_WATCHDOG               (1)
#ifdef PICO_2W
    #define PBDRV_CONFIG_WATCHDOG_RP2350    (1)
    #define PBDRV_CONFIG_WATCHDOG_RP2040    (0)
#else
    #define PBDRV_CONFIG_WATCHDOG_RP2040    (1)
    #define PBDRV_CONFIG_WATCHDOG_RP2350    (0)
#endif

#define PBDRV_CONFIG_RANDOM                 (1)
#ifdef PICO_2W
    #define PBDRV_CONFIG_RANDOM_RP2350      (1)
    #define PBDRV_CONFIG_RANDOM_RP2040      (0)
#else
    #define PBDRV_CONFIG_RANDOM_RP2040      (1)
    #define PBDRV_CONFIG_RANDOM_RP2350      (0)
#endif

#define PBDRV_CONFIG_SYS_CLOCK_RATE         (1000)
#define PBDRV_CONFIG_CONTROL_LOOP_TIME_MS   (1)

// ============================================================================
// Features
// ============================================================================

#define PBDRV_CONFIG_DMA                    (1)
#ifdef PICO_2W
    #define PBDRV_CONFIG_DMA_RP2350         (1)
    #define PBDRV_CONFIG_DMA_RP2040         (0)
    #define PBDRV_CONFIG_DMA_NUM_CHANNELS   (12)
#else
    #define PBDRV_CONFIG_DMA_RP2040         (1)
    #define PBDRV_CONFIG_DMA_RP2350         (0)
    #define PBDRV_CONFIG_DMA_NUM_CHANNELS   (4)
#endif

#define PBDRV_CONFIG_MULTICORE              (1)
#define PBDRV_CONFIG_NUM_CORES              (2)

#define PBDRV_CONFIG_PID                    (1)
#define PBDRV_CONFIG_PID_POSITION           (1)
#define PBDRV_CONFIG_PID_VELOCITY           (1)

#endif // _PBDRVCONFIG_H_
