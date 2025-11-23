// SPDX-License-Identifier: MIT
// DMA PID configuration for Pico W / Pico 2 W

#ifndef _PBDRVCONFIG_PID_DMA_H_
#define _PBDRVCONFIG_PID_DMA_H_

#include <stdint.h>
#include <stdbool.h>

// Enable DMA-accelerated PID motor control
#define PBDRV_CONFIG_MOTOR_DRIVER_RP2040_DMA        (1)

// ============================================================================
// Motor count - platform dependent
// ============================================================================
#ifdef PICO_2W
    #define PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS  (12)
    #ifndef PID_DMA_MAX_MOTORS
    #define PID_DMA_MAX_MOTORS                          (12)
    #endif
#else
    #define PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS  (4)
    #ifndef PID_DMA_MAX_MOTORS
    #define PID_DMA_MAX_MOTORS                          (4)
    #endif
#endif

// ============================================================================
// PID update rate
// ============================================================================
#define PBDRV_CONFIG_PID_DMA_UPDATE_RATE_HZ         (1000)

// Default encoder counts per revolution
#define PBDRV_CONFIG_PID_DMA_DEFAULT_CPR            (360)

// ============================================================================
// Pin assignments - Pico W (4 motors) uses GPIO 2-17
//                   Pico 2 W (12 motors) uses GPIO 0-47
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

#ifdef PICO_2W
// Port E - GPIO 18-21
#define PBDRV_CONFIG_PID_DMA_PORT_E_ENC_A   (18)
#define PBDRV_CONFIG_PID_DMA_PORT_E_ENC_B   (19)
#define PBDRV_CONFIG_PID_DMA_PORT_E_PWM_A   (20)
#define PBDRV_CONFIG_PID_DMA_PORT_E_PWM_B   (21)

// Port F - GPIO 22-25
#define PBDRV_CONFIG_PID_DMA_PORT_F_ENC_A   (22)
#define PBDRV_CONFIG_PID_DMA_PORT_F_ENC_B   (23)
#define PBDRV_CONFIG_PID_DMA_PORT_F_PWM_A   (24)
#define PBDRV_CONFIG_PID_DMA_PORT_F_PWM_B   (25)

// Port G - GPIO 26-29
#define PBDRV_CONFIG_PID_DMA_PORT_G_ENC_A   (26)
#define PBDRV_CONFIG_PID_DMA_PORT_G_ENC_B   (27)
#define PBDRV_CONFIG_PID_DMA_PORT_G_PWM_A   (28)
#define PBDRV_CONFIG_PID_DMA_PORT_G_PWM_B   (29)

// Port H - GPIO 30-33
#define PBDRV_CONFIG_PID_DMA_PORT_H_ENC_A   (30)
#define PBDRV_CONFIG_PID_DMA_PORT_H_ENC_B   (31)
#define PBDRV_CONFIG_PID_DMA_PORT_H_PWM_A   (32)
#define PBDRV_CONFIG_PID_DMA_PORT_H_PWM_B   (33)

// Port I - GPIO 34-37
#define PBDRV_CONFIG_PID_DMA_PORT_I_ENC_A   (34)
#define PBDRV_CONFIG_PID_DMA_PORT_I_ENC_B   (35)
#define PBDRV_CONFIG_PID_DMA_PORT_I_PWM_A   (36)
#define PBDRV_CONFIG_PID_DMA_PORT_I_PWM_B   (37)

// Port J - GPIO 38-41
#define PBDRV_CONFIG_PID_DMA_PORT_J_ENC_A   (38)
#define PBDRV_CONFIG_PID_DMA_PORT_J_ENC_B   (39)
#define PBDRV_CONFIG_PID_DMA_PORT_J_PWM_A   (40)
#define PBDRV_CONFIG_PID_DMA_PORT_J_PWM_B   (41)

// Port K - GPIO 42-45
#define PBDRV_CONFIG_PID_DMA_PORT_K_ENC_A   (42)
#define PBDRV_CONFIG_PID_DMA_PORT_K_ENC_B   (43)
#define PBDRV_CONFIG_PID_DMA_PORT_K_PWM_A   (44)
#define PBDRV_CONFIG_PID_DMA_PORT_K_PWM_B   (45)

// Port L - GPIO 46-47, 0-1
#define PBDRV_CONFIG_PID_DMA_PORT_L_ENC_A   (46)
#define PBDRV_CONFIG_PID_DMA_PORT_L_ENC_B   (47)
#define PBDRV_CONFIG_PID_DMA_PORT_L_PWM_A   (0)
#define PBDRV_CONFIG_PID_DMA_PORT_L_PWM_B   (1)
#endif // PICO_2W

// ============================================================================
// PIO allocation
// ============================================================================
#define PBDRV_CONFIG_PID_DMA_ENCODER_PIO    (0)
#define PBDRV_CONFIG_PID_DMA_PWM_PIO        (1)

// ============================================================================
// Default PID gains (Q16.16 fixed-point)
// ============================================================================

// Position control gains
#define PBDRV_CONFIG_PID_DMA_POS_KP_DEFAULT  (32768)   // 0.5
#define PBDRV_CONFIG_PID_DMA_POS_KI_DEFAULT  (3277)    // 0.05
#define PBDRV_CONFIG_PID_DMA_POS_KD_DEFAULT  (6554)    // 0.1

// Speed control gains
#define PBDRV_CONFIG_PID_DMA_SPD_KP_DEFAULT  (16384)   // 0.25
#define PBDRV_CONFIG_PID_DMA_SPD_KI_DEFAULT  (6554)    // 0.1
#define PBDRV_CONFIG_PID_DMA_SPD_KD_DEFAULT  (0)

// Anti-windup integral limit
#define PBDRV_CONFIG_PID_DMA_INTEGRAL_LIMIT  (360000000)

// ============================================================================
// Motor profiles
// ============================================================================

// LEGO Technic Large Motor
#define PID_DMA_PROFILE_TECHNIC_L_CPR       (360)
#define PID_DMA_PROFILE_TECHNIC_L_MAX_SPEED (1050000)

// LEGO Technic Medium Motor
#define PID_DMA_PROFILE_TECHNIC_M_CPR       (360)
#define PID_DMA_PROFILE_TECHNIC_M_MAX_SPEED (1560000)

// LEGO Technic Small Motor
#define PID_DMA_PROFILE_TECHNIC_S_CPR       (360)
#define PID_DMA_PROFILE_TECHNIC_S_MAX_SPEED (1110000)

// Generic N20 encoder motor
#define PID_DMA_PROFILE_N20_CPR             (600)
#define PID_DMA_PROFILE_N20_MAX_SPEED       (6000000)

// ============================================================================
// Advanced configuration
// ============================================================================

#define PBDRV_CONFIG_PID_DMA_STALL_DETECT       (1)
#define PBDRV_CONFIG_PID_DMA_STALL_PWM_THRESH   (8000)
#define PBDRV_CONFIG_PID_DMA_STALL_SPEED_THRESH (1000)
#define PBDRV_CONFIG_PID_DMA_STALL_ERROR_THRESH (5000)
#define PBDRV_CONFIG_PID_DMA_BUFFER_SIZE        (16)
#define PBDRV_CONFIG_PID_DMA_DEBUG              (0)

#endif // _PBDRVCONFIG_PID_DMA_H_
