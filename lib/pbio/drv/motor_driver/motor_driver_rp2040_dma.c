// SPDX-License-Identifier: MIT
// Copyright (c) 2024 pybricks-pico contributors
// Integration of DMA PID with pybricks servo/motor system

#include <pbdrv/config.h>

#if PBDRV_CONFIG_MOTOR_DRIVER_RP2040_DMA

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <pbdrv/motor_driver.h>
#include <pbio/error.h>
#include <pbio/port.h>
#include <pbio/dcmotor.h>
#include <pbio/servo.h>
#include <pbio/control.h>

#include "../pid_dma/pid_dma_rp2040.h"

// Port to motor ID mapping (configurable per board)
#ifndef PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS
#define PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS 4
#endif

// Pin configurations for each port (adjust for your hardware)
typedef struct {
    uint8_t encoder_pin_a;
    uint8_t encoder_pin_b;
    uint8_t pwm_pin_a;
    uint8_t pwm_pin_b;
    uint32_t counts_per_rev;
} port_hw_config_t;

// Hardware configuration table - customize for your board
static const port_hw_config_t port_configs[PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS] = {
    // Port A (motor_id 0)
    { .encoder_pin_a = 2,  .encoder_pin_b = 3,  .pwm_pin_a = 4,  .pwm_pin_b = 5,  .counts_per_rev = 360 },
    // Port B (motor_id 1)  
    { .encoder_pin_a = 6,  .encoder_pin_b = 7,  .pwm_pin_a = 8,  .pwm_pin_b = 9,  .counts_per_rev = 360 },
    // Port C (motor_id 2)
    { .encoder_pin_a = 10, .encoder_pin_b = 11, .pwm_pin_a = 12, .pwm_pin_b = 13, .counts_per_rev = 360 },
    // Port D (motor_id 3)
    { .encoder_pin_a = 14, .encoder_pin_b = 15, .pwm_pin_a = 16, .pwm_pin_b = 17, .counts_per_rev = 360 },
};

// Track which ports are initialized
static bool port_initialized[PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS];
static bool dma_initialized = false;

// Direction multipliers for each port
static int8_t port_direction[PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS];

/**
 * Map pybricks port ID to internal motor ID.
 */
static int port_to_motor_id(pbio_port_id_t port) {
    // Adjust mapping based on your port enumeration
    switch (port) {
        case PBIO_PORT_ID_A: return 0;
        case PBIO_PORT_ID_B: return 1;
        case PBIO_PORT_ID_C: return 2;
        case PBIO_PORT_ID_D: return 3;
        default: return -1;
    }
}

// ============================================================================
// pbdrv_motor_driver implementation
// ============================================================================

void pbdrv_motor_driver_init(void) {
    if (!dma_initialized) {
        pid_dma_init();
        dma_initialized = true;
    }
    
    memset(port_initialized, 0, sizeof(port_initialized));
    memset(port_direction, 1, sizeof(port_direction));
}

pbio_error_t pbdrv_motor_driver_get_dev(pbio_port_id_t port, pbdrv_motor_driver_dev_t **dev) {
    int motor_id = port_to_motor_id(port);
    if (motor_id < 0) {
        return PBIO_ERROR_INVALID_PORT;
    }
    
    // Return opaque pointer (motor_id encoded)
    *dev = (pbdrv_motor_driver_dev_t *)(uintptr_t)(motor_id + 1);
    return PBIO_SUCCESS;
}

pbio_error_t pbdrv_motor_driver_coast(pbdrv_motor_driver_dev_t *dev) {
    int motor_id = (int)(uintptr_t)dev - 1;
    if (motor_id < 0 || motor_id >= PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS) {
        return PBIO_ERROR_INVALID_ARG;
    }
    
    pid_dma_enable(motor_id, false);
    pid_dma_set_pwm(motor_id, 0);
    
    return PBIO_SUCCESS;
}

pbio_error_t pbdrv_motor_driver_set_duty_cycle(pbdrv_motor_driver_dev_t *dev, int16_t duty_cycle) {
    int motor_id = (int)(uintptr_t)dev - 1;
    if (motor_id < 0 || motor_id >= PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS) {
        return PBIO_ERROR_INVALID_ARG;
    }
    
    // duty_cycle is -10000 to +10000
    int32_t pwm = (int32_t)duty_cycle * port_direction[motor_id];
    pid_dma_set_pwm(motor_id, pwm);
    
    return PBIO_SUCCESS;
}

// ============================================================================
// Counter driver integration (encoder)
// ============================================================================

pbio_error_t pbdrv_counter_get_count(pbdrv_counter_dev_t *dev, int32_t *count) {
    int motor_id = (int)(uintptr_t)dev - 1;
    if (motor_id < 0 || motor_id >= PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS) {
        return PBIO_ERROR_INVALID_ARG;
    }
    
    int32_t pos, speed, pwm;
    pid_dma_get_state(motor_id, &pos, &speed, &pwm);
    
    // Convert millidegrees to degrees for pybricks
    *count = pos * port_direction[motor_id] / 1000;
    
    return PBIO_SUCCESS;
}

pbio_error_t pbdrv_counter_get_rate(pbdrv_counter_dev_t *dev, int32_t *rate) {
    int motor_id = (int)(uintptr_t)dev - 1;
    if (motor_id < 0 || motor_id >= PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS) {
        return PBIO_ERROR_INVALID_ARG;
    }
    
    int32_t pos, speed, pwm;
    pid_dma_get_state(motor_id, &pos, &speed, &pwm);
    
    // Speed in deg/s
    *rate = speed * port_direction[motor_id] / 1000;
    
    return PBIO_SUCCESS;
}

pbio_error_t pbdrv_counter_reset_count(pbdrv_counter_dev_t *dev) {
    int motor_id = (int)(uintptr_t)dev - 1;
    if (motor_id < 0 || motor_id >= PBDRV_CONFIG_MOTOR_DRIVER_RP2040_NUM_PORTS) {
        return PBIO_ERROR_INVALID_ARG;
    }
    
    pid_dma_reset_position(motor_id);
    return PBIO_SUCCESS;
}

// ============================================================================
// Extended servo interface for DMA PID
// ============================================================================

/**
 * Setup a port for DMA-accelerated motor control.
 * Call this after pbio_servo_setup() for enhanced performance.
 */
pbio_error_t pbdrv_motor_dma_setup(pbio_port_id_t port, pbio_direction_t direction) {
    int motor_id = port_to_motor_id(port);
    if (motor_id < 0) {
        return PBIO_ERROR_INVALID_PORT;
    }
    
    if (port_initialized[motor_id]) {
        // Already initialized, just update direction
        port_direction[motor_id] = (direction == PBIO_DIRECTION_CLOCKWISE) ? 1 : -1;
        return PBIO_SUCCESS;
    }
    
    const port_hw_config_t *hw = &port_configs[motor_id];
    
    pid_dma_hw_config_t config = {
        .encoder_pio = NULL,  // Use default
        .encoder_pin_a = hw->encoder_pin_a,
        .encoder_pin_b = hw->encoder_pin_b,
        .pwm_pio = NULL,      // Use default
        .pwm_pin_a = hw->pwm_pin_a,
        .pwm_pin_b = hw->pwm_pin_b,
        .counts_per_rev = hw->counts_per_rev,
    };
    
    int err = pid_dma_motor_setup(motor_id, &config);
    if (err != 0) {
        return PBIO_ERROR_FAILED;
    }
    
    port_direction[motor_id] = (direction == PBIO_DIRECTION_CLOCKWISE) ? 1 : -1;
    port_initialized[motor_id] = true;
    
    return PBIO_SUCCESS;
}

/**
 * Enable DMA-accelerated PID for a servo.
 * When enabled, the PID loop runs entirely in hardware.
 */
pbio_error_t pbdrv_motor_dma_enable_pid(pbio_port_id_t port, bool enable) {
    int motor_id = port_to_motor_id(port);
    if (motor_id < 0 || !port_initialized[motor_id]) {
        return PBIO_ERROR_INVALID_PORT;
    }
    
    pid_dma_enable(motor_id, enable);
    return PBIO_SUCCESS;
}

/**
 * Set PID gains for DMA-accelerated control.
 * Gains are in milli-units for compatibility with pybricks.
 */
pbio_error_t pbdrv_motor_dma_set_pid(pbio_port_id_t port, 
                                      int32_t kp, int32_t ki, int32_t kd) {
    int motor_id = port_to_motor_id(port);
    if (motor_id < 0 || !port_initialized[motor_id]) {
        return PBIO_ERROR_INVALID_PORT;
    }
    
    // Convert from pybricks units (µNm/deg) to fixed-point
    // Pybricks uses different scaling, adjust as needed
    int32_t fp_kp = kp << 6;  // Scale factor depends on motor
    int32_t fp_ki = ki << 2;
    int32_t fp_kd = kd << 8;
    
    pid_dma_set_gains(motor_id, fp_kp, fp_ki, fp_kd);
    return PBIO_SUCCESS;
}

/**
 * Run to target position using DMA PID.
 */
pbio_error_t pbdrv_motor_dma_run_target(pbio_port_id_t port, int32_t target_deg) {
    int motor_id = port_to_motor_id(port);
    if (motor_id < 0 || !port_initialized[motor_id]) {
        return PBIO_ERROR_INVALID_PORT;
    }
    
    // Convert to millidegrees with direction
    int32_t target_mdeg = target_deg * 1000 * port_direction[motor_id];
    
    pid_dma_set_position_target(motor_id, target_mdeg);
    pid_dma_enable(motor_id, true);
    
    return PBIO_SUCCESS;
}

/**
 * Run at target speed using DMA PID.
 */
pbio_error_t pbdrv_motor_dma_run_speed(pbio_port_id_t port, int32_t speed_deg_s) {
    int motor_id = port_to_motor_id(port);
    if (motor_id < 0 || !port_initialized[motor_id]) {
        return PBIO_ERROR_INVALID_PORT;
    }
    
    int32_t speed_mdeg = speed_deg_s * 1000 * port_direction[motor_id];
    
    pid_dma_set_speed_target(motor_id, speed_mdeg);
    pid_dma_enable(motor_id, true);
    
    return PBIO_SUCCESS;
}

/**
 * Check if motor is done with target (for position mode).
 */
bool pbdrv_motor_dma_is_done(pbio_port_id_t port, int32_t tolerance_deg) {
    int motor_id = port_to_motor_id(port);
    if (motor_id < 0 || !port_initialized[motor_id]) {
        return true;
    }
    
    volatile pid_dma_state_t *state = pid_dma_get_state_ptr(motor_id);
    if (!state || state->mode != 0) {
        return true;  // Not in position mode
    }
    
    int32_t error = state->target_position - state->current_position;
    if (error < 0) error = -error;
    
    return error < (tolerance_deg * 1000);
}

/**
 * Check if motor is stalled.
 */
bool pbdrv_motor_dma_is_stalled(pbio_port_id_t port) {
    int motor_id = port_to_motor_id(port);
    if (motor_id < 0) return false;
    
    return pid_dma_is_stalled(motor_id);
}

// ============================================================================
// Pybricks observer integration
// ============================================================================

/**
 * Get estimated state for pybricks observer.
 * The DMA system provides higher-frequency estimates than the normal
 * pybricks control loop.
 */
pbio_error_t pbdrv_motor_dma_get_observer_state(pbio_port_id_t port,
                                                  int32_t *angle_mdeg,
                                                  int32_t *speed_mdeg_s,
                                                  int32_t *current_ma) {
    int motor_id = port_to_motor_id(port);
    if (motor_id < 0 || !port_initialized[motor_id]) {
        return PBIO_ERROR_INVALID_PORT;
    }
    
    int32_t pos, speed, pwm;
    pid_dma_get_state(motor_id, &pos, &speed, &pwm);
    
    if (angle_mdeg) *angle_mdeg = pos * port_direction[motor_id];
    if (speed_mdeg_s) *speed_mdeg_s = speed * port_direction[motor_id];
    
    // Estimate current from PWM (rough approximation)
    // Real implementation would use ADC current sensing
    if (current_ma) {
        // Assume ~1A at full PWM for small motors
        *current_ma = (pwm * 100) / 10000;
    }
    
    return PBIO_SUCCESS;
}

#endif // PBDRV_CONFIG_MOTOR_DRIVER_RP2040_DMA
