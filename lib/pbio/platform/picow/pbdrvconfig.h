// SPDX-License-Identifier: MIT
// pbio driver configuration for RP2040B (Pico W)

#ifndef _PBDRVCONFIG_H_
#define _PBDRVCONFIG_H_

#include "pbdrvconfig_pid_dma.h"

// Platform
#define PBDRV_CONFIG_PICOW                             (1)
#define PBDRV_CONFIG_RP2040B                            (1)

// ========== Hardware Peripherals ==========

// Clock driver
#define PBDRV_CONFIG_CLOCK                              (1)
#define PBDRV_CONFIG_CLOCK_RP2040                       (1)

// Counter driver - 4 encoders with PIO
#define PBDRV_CONFIG_COUNTER                            (1)
#define PBDRV_CONFIG_COUNTER_RP2040_PIO                 (1)
#define PBDRV_CONFIG_COUNTER_NUM_DEV                    (4)

// PWM driver - 4 motors
#define PBDRV_CONFIG_PWM                                (1)
#define PBDRV_CONFIG_PWM_RP2040_PIO                     (1)
#define PBDRV_CONFIG_PWM_NUM_DEV                        (4)

// GPIO driver
#define PBDRV_CONFIG_GPIO                               (1)
#define PBDRV_CONFIG_GPIO_RP2040                        (1)

// Reset driver
#define PBDRV_CONFIG_RESET                              (1)
#define PBDRV_CONFIG_RESET_RP2040                       (1)

// ========== Motor Control ==========

// Motor driver
#define PBDRV_CONFIG_MOTOR_DRIVER                       (1)
#define PBDRV_CONFIG_MOTOR_DRIVER_NUM_DEV               (4)

// DC Motor
#define PBDRV_CONFIG_DCMOTOR                            (1)
#define PBDRV_CONFIG_DCMOTOR_NUM_DEV                    (4)

// Tacho motor
#define PBDRV_CONFIG_TACHO                              (1)
#define PBDRV_CONFIG_TACHO_NUM_DEV                      (4)

// Servo
#define PBDRV_CONFIG_SERVO                              (1)
#define PBDRV_CONFIG_SERVO_NUM_DEV                      (4)

// ========== Communication ==========

// Bluetooth
#define PBDRV_CONFIG_BLUETOOTH                          (1)
#define PBDRV_CONFIG_BLUETOOTH_BTSTACK                  (1)
#define PBDRV_CONFIG_BLUETOOTH_BTSTACK_HCI_TRANSPORT_CYW43 (1)

// ========== System ==========

// Watchdog
#define PBDRV_CONFIG_WATCHDOG                           (1)
#define PBDRV_CONFIG_WATCHDOG_RP2040                    (1)

// Random
#define PBDRV_CONFIG_RANDOM                             (1)
#define PBDRV_CONFIG_RANDOM_RP2040                      (1)

// System clock rate (1 kHz = 1ms tick)
#define PBDRV_CONFIG_SYS_CLOCK_RATE                     (1000)

// Control loop time (1ms)
#define PBDRV_CONFIG_CONTROL_LOOP_TIME_MS               (1)

// ========== Features ==========

// DMA support
#define PBDRV_CONFIG_DMA                                (1)
#define PBDRV_CONFIG_DMA_RP2040                         (1)
#define PBDRV_CONFIG_DMA_NUM_CHANNELS                   (4)

// Multi-core support
#define PBDRV_CONFIG_MULTICORE                          (1)
#define PBDRV_CONFIG_NUM_CORES                          (2)

// PID Control
#define PBDRV_CONFIG_PID                                (1)
#define PBDRV_CONFIG_PID_POSITION                       (1)
#define PBDRV_CONFIG_PID_VELOCITY                       (1)

#endif // _PBDRVCONFIG_H_
