// SPDX-License-Identifier: MIT
// RP2350B 12-motor PWM driver with DMA profiles

#include <pbdrv/config.h>

#if PBDRV_CONFIG_PWM_RP2350_PIO

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "pico/multicore.h"

#include <pbio/error.h>

#define NUM_MOTORS 12
#define PWM_FREQUENCY_HZ 20000
#define PWM_WRAP_VALUE 7500  // 150 MHz / 20 kHz

// Motor control structure
typedef struct {
    uint8_t pin_a;
    uint8_t pin_b;
    uint pwm_slice;
    uint pwm_chan_a;
    uint pwm_chan_b;
    
    // DMA profile support
    int dma_chan;
    int16_t *profile;
    uint32_t profile_len;
    uint32_t profile_pos;
    bool profile_active;
} motor_pwm_t;

static motor_pwm_t motors[NUM_MOTORS];

// Profile execution on Core 1
static volatile bool core1_running = false;

void pbdrv_pwm_init(void) {
    printf("\n=== Initializing 12-Motor PWM System ===\n");
    printf("Target: RP2350B (48 GPIO)\n");
    printf("PWM Frequency: %d Hz\n", PWM_FREQUENCY_HZ);
    
    // Pin assignments for 12 motors
    const uint8_t pin_map[NUM_MOTORS][2] = {
        {0, 1},    {2, 3},    {4, 5},    {6, 7},
        {8, 9},    {10, 11},  {12, 13},  {14, 15},
        {16, 17},  {18, 19},  {20, 21},  {22, 23}
    };
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_pwm_t *m = &motors[i];
        
        m->pin_a = pin_map[i][0];
        m->pin_b = pin_map[i][1];
        m->profile_active = false;
        m->dma_chan = -1;
        
        // Configure PWM
        gpio_set_function(m->pin_a, GPIO_FUNC_PWM);
        gpio_set_function(m->pin_b, GPIO_FUNC_PWM);
        
        m->pwm_slice = pwm_gpio_to_slice_num(m->pin_a);
        m->pwm_chan_a = pwm_gpio_to_channel(m->pin_a);
        m->pwm_chan_b = pwm_gpio_to_channel(m->pin_b);
        
        // Set PWM frequency
        pwm_set_wrap(m->pwm_slice, PWM_WRAP_VALUE);
        pwm_set_chan_level(m->pwm_slice, m->pwm_chan_a, 0);
        pwm_set_chan_level(m->pwm_slice, m->pwm_chan_b, 0);
        pwm_set_enabled(m->pwm_slice, true);
        
        printf("  Motor %2d: GP%d/GP%d, Slice %d\n", 
               i, m->pin_a, m->pin_b, m->pwm_slice);
    }
    
    printf("12-Motor PWM initialized\n");
}

// Set motor duty cycle
void pbdrv_pwm_set_duty_simple(uint8_t id, int16_t duty) {
    if (id >= NUM_MOTORS) return;
    
    motor_pwm_t *m = &motors[id];
    
    // Stop active profile
    if (m->profile_active) {
        m->profile_active = false;
    }
    
    // Calculate PWM level
    uint16_t level;
    
    if (duty > 0) {
        // Forward
        level = (duty * PWM_WRAP_VALUE) / 10000;
        if (level > PWM_WRAP_VALUE) level = PWM_WRAP_VALUE;
        pwm_set_chan_level(m->pwm_slice, m->pwm_chan_a, level);
        pwm_set_chan_level(m->pwm_slice, m->pwm_chan_b, 0);
    } else if (duty < 0) {
        // Reverse
        level = (-duty * PWM_WRAP_VALUE) / 10000;
        if (level > PWM_WRAP_VALUE) level = PWM_WRAP_VALUE;
        pwm_set_chan_level(m->pwm_slice, m->pwm_chan_a, 0);
        pwm_set_chan_level(m->pwm_slice, m->pwm_chan_b, level);
    } else {
        // Coast
        pwm_set_chan_level(m->pwm_slice, m->pwm_chan_a, 0);
        pwm_set_chan_level(m->pwm_slice, m->pwm_chan_b, 0);
    }
}

// Generate trapezoidal profile
void pbdrv_pwm_generate_trapezoid(int16_t *profile, uint32_t length,
                                   int16_t max_speed, uint32_t accel_steps,
                                   uint32_t decel_steps) {
    if (!profile || length == 0) return;
    
    uint32_t cruise_steps = (length > accel_steps + decel_steps) ? 
                             length - accel_steps - decel_steps : 0;
    
    // Acceleration
    for (uint32_t i = 0; i < accel_steps && i < length; i++) {
        profile[i] = (max_speed * i) / accel_steps;
    }
    
    // Cruise
    for (uint32_t i = accel_steps; i < accel_steps + cruise_steps && i < length; i++) {
        profile[i] = max_speed;
    }
    
    // Deceleration
    uint32_t decel_start = accel_steps + cruise_steps;
    for (uint32_t i = decel_start; i < length; i++) {
        uint32_t idx = i - decel_start;
        profile[i] = max_speed - (max_speed * idx) / decel_steps;
    }
    
    printf("Generated trapezoid: accel=%ld, cruise=%ld, decel=%ld\n",
           accel_steps, cruise_steps, decel_steps);
}

// Core 1 task: Execute velocity profiles
void core1_profile_task(void) {
    printf("Core 1: Profile execution task started\n");
    
    while (core1_running) {
        // Update all active profiles at 1 kHz
        for (int i = 0; i < NUM_MOTORS; i++) {
            motor_pwm_t *m = &motors[i];
            
            if (m->profile_active && m->profile) {
                // Apply next profile value
                if (m->profile_pos < m->profile_len) {
                    pbdrv_pwm_set_duty_simple(i, m->profile[m->profile_pos]);
                    m->profile_pos++;
                } else {
                    // Profile complete
                    m->profile_active = false;
                    pbdrv_pwm_set_duty_simple(i, 0);
                    printf("Motor %d profile complete\n", i);
                }
            }
        }
        
        // 1 kHz update rate
        sleep_us(1000);
    }
}

// Start velocity profile
pbio_error_t pbdrv_pwm_run_profile(uint8_t id, int16_t *profile, uint32_t length) {
    if (id >= NUM_MOTORS || !profile || length == 0) {
        return PBIO_ERROR_INVALID_ARG;
    }
    
    motor_pwm_t *m = &motors[id];
    
    m->profile = profile;
    m->profile_len = length;
    m->profile_pos = 0;
    m->profile_active = true;
    
    // Start Core 1 if not running
    if (!core1_running) {
        core1_running = true;
        multicore_launch_core1(core1_profile_task);
    }
    
    printf("Motor %d: Profile started (%ld steps)\n", id, length);
    return PBIO_SUCCESS;
}

// Stop all profiles
void pbdrv_pwm_stop_all_profiles(void) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].profile_active = false;
        pbdrv_pwm_set_duty_simple(i, 0);
    }
}

// Brake
void pbdrv_pwm_brake(uint8_t id) {
    if (id >= NUM_MOTORS) return;
    motor_pwm_t *m = &motors[id];
    pwm_set_chan_level(m->pwm_slice, m->pwm_chan_a, PWM_WRAP_VALUE);
    pwm_set_chan_level(m->pwm_slice, m->pwm_chan_b, PWM_WRAP_VALUE);
}

#endif // PBDRV_CONFIG_PWM_RP2350_PIO
