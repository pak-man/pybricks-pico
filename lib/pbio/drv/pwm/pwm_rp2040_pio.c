// SPDX-License-Identifier: MIT
// RP2040 PWM driver for motor control with dual PWM

#include <pbdrv/config.h>

#if PBDRV_CONFIG_PWM_RP2040_PIO

#include <stdint.h>
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/pwm.h"

#include <pbio/error.h>

// Motor port PWM definitions
typedef struct {
    uint8_t pin_a;         // PWM A pin (White - Forward)
    uint8_t pin_b;         // PWM B pin (Black - Reverse)
    uint pwm_slice_a;      // PWM slice for pin A
    uint pwm_slice_b;      // PWM slice for pin B
    uint pwm_chan_a;       // PWM channel for pin A
    uint pwm_chan_b;       // PWM channel for pin B
} motor_pwm_t;

static motor_pwm_t motors[PBDRV_CONFIG_PWM_NUM_DEV];

void pbdrv_pwm_init(void) {
    printf("Initializing dual-PWM motor control...\n");
    
    // Port A: GPIO 0-1
    motors[0].pin_a = 0;  // White wire (Forward)
    motors[0].pin_b = 1;  // Black wire (Reverse)
    
    // Port B: GPIO 4-5
    motors[1].pin_a = 4;
    motors[1].pin_b = 5;
    
    // Port C: GPIO 8-9
    motors[2].pin_a = 8;
    motors[2].pin_b = 9;
    
    // Port D: GPIO 12-13
    motors[3].pin_a = 12;
    motors[3].pin_b = 13;
    
    // Initialize PWM for all motor ports
    for (int i = 0; i < PBDRV_CONFIG_PWM_NUM_DEV; i++) {
        motor_pwm_t *m = &motors[i];
        
        // Set GPIO function to PWM
        gpio_set_function(m->pin_a, GPIO_FUNC_PWM);
        gpio_set_function(m->pin_b, GPIO_FUNC_PWM);
        
        // Get PWM slice and channel numbers
        m->pwm_slice_a = pwm_gpio_to_slice_num(m->pin_a);
        m->pwm_slice_b = pwm_gpio_to_slice_num(m->pin_b);
        m->pwm_chan_a = pwm_gpio_to_channel(m->pin_a);
        m->pwm_chan_b = pwm_gpio_to_channel(m->pin_b);
        
        // Configure PWM - 20 kHz frequency
        // System clock = 133 MHz, wrap = 6650 for 20 kHz
        pwm_set_wrap(m->pwm_slice_a, 6650);
        pwm_set_wrap(m->pwm_slice_b, 6650);
        
        // Set both channels to 0 initially (coast)
        pwm_set_chan_level(m->pwm_slice_a, m->pwm_chan_a, 0);
        pwm_set_chan_level(m->pwm_slice_b, m->pwm_chan_b, 0);
        
        // Enable PWM slices
        pwm_set_enabled(m->pwm_slice_a, true);
        pwm_set_enabled(m->pwm_slice_b, true);
    }
    
    printf("Dual-PWM initialized (4 ports @ 20 kHz)\n");
}

// Simple wrapper to set motor duty cycle by ID
void pbdrv_pwm_set_duty_simple(uint8_t id, int16_t duty) {
    if (id >= PBDRV_CONFIG_PWM_NUM_DEV) {
        return;
    }
    
    motor_pwm_t *m = &motors[id];
    
    // duty is in range -10000 to +10000 (representing -100.00% to +100.00%)
    
    if (duty > 0) {
        // Clockwise (Forward): PWM on A, B = 0V
        uint16_t level = (duty * 6650) / 10000;
        if (level > 6650) level = 6650;
        
        pwm_set_chan_level(m->pwm_slice_a, m->pwm_chan_a, level);
        pwm_set_chan_level(m->pwm_slice_b, m->pwm_chan_b, 0);
        
    } else if (duty < 0) {
        // Counter-Clockwise (Reverse): A = 0V, PWM on B
        uint16_t level = (-duty * 6650) / 10000;
        if (level > 6650) level = 6650;
        
        pwm_set_chan_level(m->pwm_slice_a, m->pwm_chan_a, 0);
        pwm_set_chan_level(m->pwm_slice_b, m->pwm_chan_b, level);
        
    } else {
        // duty == 0: Coast (both LOW)
        pwm_set_chan_level(m->pwm_slice_a, m->pwm_chan_a, 0);
        pwm_set_chan_level(m->pwm_slice_b, m->pwm_chan_b, 0);
    }
}

// Brake mode
void pbdrv_pwm_brake(uint8_t id) {
    if (id >= PBDRV_CONFIG_PWM_NUM_DEV) {
        return;
    }
    
    motor_pwm_t *m = &motors[id];
    
    // Brake: Both pins at same PWM level
    pwm_set_chan_level(m->pwm_slice_a, m->pwm_chan_a, 6650);
    pwm_set_chan_level(m->pwm_slice_b, m->pwm_chan_b, 6650);
}

#endif // PBDRV_CONFIG_PWM_RP2040_PIO
