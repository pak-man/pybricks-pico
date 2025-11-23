// SPDX-License-Identifier: MIT
// Copyright (c) 2024 pybricks-pico contributors
// Platform-specific DMA PID configuration for Pico W
//
// FILE LOCATION: lib/pbio/platform/picow/pbdrvconfig_pid_dma.h
//

#ifndef _PBDRVCONFIG_PID_DMA_PICOW_H_
#define _PBDRVCONFIG_PID_DMA_PICOW_H_

#include <stdint.h>
#include <stdbool.h>

// Enable DMA-accelerated PID motor control
#define PBDRV_CONFIG_MOTOR_DRIVER_RP2040_DMA        (1)

// Number of motor ports with DMA PID support
#define PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS  (4)

// PID update rate (Hz) - higher = more responsive, more DMA bandwidth
#define PBDRV_CONFIG_PID_DMA_UPDATE_RATE_HZ         (1000)

// Default encoder counts per revolution (adjust per motor type)
// LEGO Technic motors typically have 360 counts/rev
#define PBDRV_CONFIG_PID_DMA_DEFAULT_CPR            (360)

// ============================================================================
// Pin assignments for Pico W motor ports
// Adjust these to match your hardware connections
// ============================================================================

// Port A - GPIO 2-5
#define PBDRV_CONFIG_PID_DMA_PORT_A_ENC_A   (2)
#define PBDRV_CONFIG_PID_DMA_PORT_A_ENC_B   (3)
#define PBDRV_CONFIG_PID_DMA_PORT_A_PWM_A   (4)
#define PBDRV_CONFIG_PID_DMA_PORT_A_PWM_B   (5)

// Port B - GPIO 6-9
#define PBDRV_CONFIG_PID_DMA_PORT_B_ENC_A   (6)
#define PBDRV_CONFIG_PID_DMA_PORT_B_ENC_B   (7)
#define PBDRV_CONFIG_PID_DMA_PORT_B_PWM_A   (8)
#define PBDRV_CONFIG_PID_DMA_PORT_B_PWM_B   (9)

// Port C - GPIO 10-13
#define PBDRV_CONFIG_PID_DMA_PORT_C_ENC_A   (10)
#define PBDRV_CONFIG_PID_DMA_PORT_C_ENC_B   (11)
#define PBDRV_CONFIG_PID_DMA_PORT_C_PWM_A   (12)
#define PBDRV_CONFIG_PID_DMA_PORT_C_PWM_B   (13)

// Port D - GPIO 14-17
#define PBDRV_CONFIG_PID_DMA_PORT_D_ENC_A   (14)
#define PBDRV_CONFIG_PID_DMA_PORT_D_ENC_B   (15)
#define PBDRV_CONFIG_PID_DMA_PORT_D_PWM_A   (16)
#define PBDRV_CONFIG_PID_DMA_PORT_D_PWM_B   (17)

// ============================================================================
// PIO allocation
// ============================================================================

// Which PIO block to use for encoders (0 or 1)
#define PBDRV_CONFIG_PID_DMA_ENCODER_PIO    (0)

// Which PIO block to use for PWM (0 or 1)
#define PBDRV_CONFIG_PID_DMA_PWM_PIO        (1)

// State machine allocations:
// Encoder PIO (pio0): SM0-SM3 for 4 encoders
// PWM PIO (pio1): SM0-SM3 for 4 H-bridges
// Timer: pio0 SM3 (shared, one timer for all motors)

// ============================================================================
// Default PID gains (can be overridden at runtime)
// Values in fixed-point Q16.16 format
// ============================================================================

// Position control gains
#define PBDRV_CONFIG_PID_DMA_POS_KP_DEFAULT  (32768)   // 0.5 in Q16.16
#define PBDRV_CONFIG_PID_DMA_POS_KI_DEFAULT  (3277)    // 0.05 in Q16.16
#define PBDRV_CONFIG_PID_DMA_POS_KD_DEFAULT  (6554)    // 0.1 in Q16.16

// Speed control gains  
#define PBDRV_CONFIG_PID_DMA_SPD_KP_DEFAULT  (16384)   // 0.25 in Q16.16
#define PBDRV_CONFIG_PID_DMA_SPD_KI_DEFAULT  (6554)    // 0.1 in Q16.16
#define PBDRV_CONFIG_PID_DMA_SPD_KD_DEFAULT  (0)       // 0 (speed control usually needs less D)

// Anti-windup integral limit (in millidegree-seconds)
#define PBDRV_CONFIG_PID_DMA_INTEGRAL_LIMIT  (360000000)

// ============================================================================
// Motor-specific configurations
// Define profiles for different LEGO motor types
// ============================================================================

// LEGO Technic Large Motor
#define PID_DMA_PROFILE_TECHNIC_L_CPR       (360)
#define PID_DMA_PROFILE_TECHNIC_L_MAX_SPEED (1050000)  // ~1050 deg/s in mdeg/s

// LEGO Technic Medium Motor
#define PID_DMA_PROFILE_TECHNIC_M_CPR       (360)
#define PID_DMA_PROFILE_TECHNIC_M_MAX_SPEED (1560000)  // ~1560 deg/s

// LEGO Technic Small Motor
#define PID_DMA_PROFILE_TECHNIC_S_CPR       (360)
#define PID_DMA_PROFILE_TECHNIC_S_MAX_SPEED (1110000)  // ~1110 deg/s

// Generic DC motor with N20 encoder (adjust for your hardware)
#define PID_DMA_PROFILE_N20_CPR             (600)      // Typical N20 with encoder
#define PID_DMA_PROFILE_N20_MAX_SPEED       (6000000)  // ~6000 deg/s

// ============================================================================
// Advanced configuration
// ============================================================================

// Enable stall detection
#define PBDRV_CONFIG_PID_DMA_STALL_DETECT   (1)

// Stall detection thresholds
#define PBDRV_CONFIG_PID_DMA_STALL_PWM_THRESH    (8000)   // PWM > 80%
#define PBDRV_CONFIG_PID_DMA_STALL_SPEED_THRESH  (1000)   // Speed < 1 deg/s
#define PBDRV_CONFIG_PID_DMA_STALL_ERROR_THRESH  (5000)   // Error > 5 deg

// DMA ring buffer size (must be power of 2)
#define PBDRV_CONFIG_PID_DMA_BUFFER_SIZE    (16)

// Enable debug output via UART
#define PBDRV_CONFIG_PID_DMA_DEBUG          (0)

#endif // _PBDRVCONFIG_PID_DMA_PICOW_H_
