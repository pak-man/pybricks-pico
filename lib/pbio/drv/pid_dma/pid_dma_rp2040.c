// SPDX-License-Identifier: MIT
// DMA-accelerated PID motor control implementation

#include "pid_dma_rp2040.h"
#include "pid_dma.pio.h"

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"

static pid_dma_motor_t motors[PID_DMA_MAX_MOTORS];
static bool subsystem_initialized = false;
static uint encoder_prog_offset = 0;

// Fixed-point multiply
static inline int32_t fp_mul(int32_t a, int32_t b) {
    return (int32_t)(((int64_t)a * b) >> PID_FP_SHIFT);
}

static inline int32_t clamp_val(int32_t val, int32_t min, int32_t max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void pid_dma_init(void) {
    if (subsystem_initialized) return;
    memset(motors, 0, sizeof(motors));
    
    // Load encoder PIO program
    encoder_prog_offset = pio_add_program(pio0, &quadrature_encoder_program);
    
    subsystem_initialized = true;
}

int pid_dma_motor_setup(uint8_t motor_id, const pid_dma_hw_config_t *config) {
    if (motor_id >= PID_DMA_MAX_MOTORS) return -1;
    if (!subsystem_initialized) return -2;
    
    pid_dma_motor_t *m = &motors[motor_id];
    memcpy(&m->hw, config, sizeof(pid_dma_hw_config_t));
    memset((void*)&m->state, 0, sizeof(pid_dma_state_t));
    
    // Default gains
    m->state.kp = 32768;  // 0.5 in Q16.16
    m->state.ki = 3277;   // 0.05
    m->state.kd = 6554;   // 0.1
    m->state.integral_limit = 1000000;
    
    // Setup encoder PIO
    PIO enc_pio = pio0;
    uint enc_sm = pio_claim_unused_sm(enc_pio, true);
    m->hw.encoder_pio = enc_pio;
    m->hw.encoder_sm = enc_sm;
    
    // Initialize encoder - only needs pin_a, pin_b must be pin_a + 1
    quadrature_encoder_program_init(enc_pio, enc_sm, encoder_prog_offset,
                                     config->encoder_pin_a);
    
    // Setup PWM using hardware PWM (simpler than PIO for H-bridge)
    // Pin A for forward, Pin B for reverse
    gpio_set_function(config->pwm_pin_a, GPIO_FUNC_PWM);
    gpio_set_function(config->pwm_pin_b, GPIO_FUNC_PWM);
    
    uint slice_a = pwm_gpio_to_slice_num(config->pwm_pin_a);
    uint slice_b = pwm_gpio_to_slice_num(config->pwm_pin_b);
    
    // Configure PWM: 20kHz, 8-bit resolution
    pwm_set_wrap(slice_a, 255);
    pwm_set_wrap(slice_b, 255);
    pwm_set_clkdiv(slice_a, 49.0f);  // 125MHz / 49 / 256 ≈ 10kHz
    pwm_set_clkdiv(slice_b, 49.0f);
    pwm_set_enabled(slice_a, true);
    pwm_set_enabled(slice_b, true);
    
    // Store PWM slice numbers for later use
    m->hw.pwm_sm = slice_a;  // Reuse field for slice_a
    m->hw.pwm_dma_chan = slice_b;  // Reuse field for slice_b
    
    m->initialized = true;
    return 0;
}

// Set motor PWM directly
static void set_motor_pwm(pid_dma_motor_t *m, int32_t pwm) {
    uint pin_a = m->hw.pwm_pin_a;
    uint pin_b = m->hw.pwm_pin_b;
    
    if (pwm > 0) {
        // Forward: pin_a = PWM, pin_b = 0
        pwm_set_gpio_level(pin_a, pwm > 255 ? 255 : pwm);
        pwm_set_gpio_level(pin_b, 0);
    } else if (pwm < 0) {
        // Reverse: pin_a = 0, pin_b = PWM
        pwm_set_gpio_level(pin_a, 0);
        pwm_set_gpio_level(pin_b, (-pwm) > 255 ? 255 : -pwm);
    } else {
        // Stop: both low (coast)
        pwm_set_gpio_level(pin_a, 0);
        pwm_set_gpio_level(pin_b, 0);
    }
}

void pid_dma_set_gains(uint8_t motor_id, int32_t kp, int32_t ki, int32_t kd) {
    if (motor_id >= PID_DMA_MAX_MOTORS) return;
    motors[motor_id].state.kp = kp;
    motors[motor_id].state.ki = ki;
    motors[motor_id].state.kd = kd;
}

void pid_dma_set_position_target(uint8_t motor_id, int32_t position) {
    if (motor_id >= PID_DMA_MAX_MOTORS) return;
    motors[motor_id].state.target_position = position;
    motors[motor_id].state.mode = 0;
    motors[motor_id].state.integral = 0;
}

void pid_dma_set_speed_target(uint8_t motor_id, int32_t speed) {
    if (motor_id >= PID_DMA_MAX_MOTORS) return;
    motors[motor_id].state.target_speed = speed;
    motors[motor_id].state.mode = 1;
}

void pid_dma_set_pwm(uint8_t motor_id, int32_t pwm) {
    if (motor_id >= PID_DMA_MAX_MOTORS) return;
    pid_dma_motor_t *m = &motors[motor_id];
    
    // Scale from -10000..+10000 to -255..+255
    int32_t scaled = (pwm * 255) / 10000;
    m->state.pwm_output = clamp_val(scaled, -255, 255);
    m->state.mode = 2;
    
    set_motor_pwm(m, m->state.pwm_output);
}

void pid_dma_enable(uint8_t motor_id, bool enable) {
    if (motor_id >= PID_DMA_MAX_MOTORS) return;
    pid_dma_motor_t *m = &motors[motor_id];
    
    if (enable && !m->state.enabled) {
        m->state.integral = 0;
        m->state.last_error = 0;
    } else if (!enable && m->state.enabled) {
        set_motor_pwm(m, 0);  // Stop motor
    }
    m->state.enabled = enable;
}

int pid_dma_get_state(uint8_t motor_id, int32_t *position, int32_t *speed, int32_t *pwm) {
    if (motor_id >= PID_DMA_MAX_MOTORS) return -1;
    pid_dma_motor_t *m = &motors[motor_id];
    
    // Read current encoder count
    int32_t count = quadrature_encoder_get_count(m->hw.encoder_pio, m->hw.encoder_sm);
    
    // Convert to millidegrees: count * 360000 / counts_per_rev
    int32_t pos = (count * 360000) / (int32_t)m->hw.counts_per_rev;
    
    if (position) *position = pos;
    if (speed) *speed = m->state.computed_speed;
    if (pwm) *pwm = m->state.pwm_output;
    return 0;
}

bool pid_dma_is_stalled(uint8_t motor_id) {
    if (motor_id >= PID_DMA_MAX_MOTORS) return false;
    return motors[motor_id].state.stalled;
}

void pid_dma_reset_position(uint8_t motor_id) {
    if (motor_id >= PID_DMA_MAX_MOTORS) return;
    pid_dma_motor_t *m = &motors[motor_id];
    
    // Reset encoder count in PIO
    pio_sm_exec(m->hw.encoder_pio, m->hw.encoder_sm, pio_encode_set(pio_x, 0));
    
    m->state.current_position = 0;
    m->state.last_position = 0;
    m->state.target_position = 0;
    m->state.integral = 0;
}

volatile pid_dma_state_t* pid_dma_get_state_ptr(uint8_t motor_id) {
    if (motor_id >= PID_DMA_MAX_MOTORS) return NULL;
    return &motors[motor_id].state;
}

void pid_dma_force_update(uint8_t motor_id) {
    if (motor_id >= PID_DMA_MAX_MOTORS) return;
    pid_dma_motor_t *m = &motors[motor_id];
    
    if (!m->initialized || !m->state.enabled) return;
    
    // Read encoder
    int32_t count = quadrature_encoder_get_count(m->hw.encoder_pio, m->hw.encoder_sm);
    uint32_t now = timer_hw->timerawl;
    
    // Update position (millidegrees)
    m->state.last_position = m->state.current_position;
    m->state.last_timestamp_us = m->state.timestamp_us;
    m->state.current_position = (count * 360000) / (int32_t)m->hw.counts_per_rev;
    m->state.timestamp_us = now;
    
    // Compute speed (millideg/s)
    uint32_t dt = m->state.timestamp_us - m->state.last_timestamp_us;
    if (dt > 0 && dt < 100000) {
        int32_t dpos = m->state.current_position - m->state.last_position;
        m->state.computed_speed = (dpos * 1000000) / (int32_t)dt;
    }
    
    // PID calculation
    int32_t error = 0;
    if (m->state.mode == 0) {
        // Position control
        error = m->state.target_position - m->state.current_position;
    } else if (m->state.mode == 1) {
        // Speed control
        error = m->state.target_speed - m->state.computed_speed;
    } else {
        // Direct PWM mode - no PID
        return;
    }
    
    int32_t derivative = error - m->state.last_error;
    m->state.integral = clamp_val(m->state.integral + error, 
                                   -m->state.integral_limit, m->state.integral_limit);
    
    int32_t p_term = fp_mul(m->state.kp, error);
    int32_t i_term = fp_mul(m->state.ki, m->state.integral);
    int32_t d_term = fp_mul(m->state.kd, derivative);
    
    int32_t output = p_term + i_term + d_term;
    
    // Scale output to PWM range (-255 to +255)
    output = clamp_val(output >> 8, -255, 255);
    
    m->state.last_error = error;
    m->state.pwm_output = output;
    
    // Stall detection
    if (m->state.mode == 0) {
        bool high_effort = (output > 200 || output < -200);
        bool low_speed = (m->state.computed_speed > -10000 && m->state.computed_speed < 10000);
        bool has_error = (error > 5000 || error < -5000);
        m->state.stalled = high_effort && low_speed && has_error;
    }
    
    // Apply to motor
    set_motor_pwm(m, output);
}

void pid_dma_deinit(void) {
    for (uint8_t i = 0; i < PID_DMA_MAX_MOTORS; i++) {
        if (motors[i].initialized) {
            pid_dma_enable(i, false);
            pio_sm_unclaim(motors[i].hw.encoder_pio, motors[i].hw.encoder_sm);
        }
    }
    
    pio_remove_program(pio0, &quadrature_encoder_program, encoder_prog_offset);
    subsystem_initialized = false;
}

// ============================================================================
// Compatibility wrappers for legacy Pybricks counter/PWM API
// These bridge the gap between main.c/system_test.c and the new DMA PID driver
// TODO: Refactor main.c/system_test.c to use pid_dma_* functions directly,
//       then remove these wrappers
// ============================================================================

void pbdrv_counter_init(void) {
    // No-op: Initialization already done in pid_dma_init()
}

void pbdrv_counter_update(void) {
    // No-op: Updates happen per-motor in pid_dma_force_update()
}

void pbdrv_pwm_init(void) {
    // No-op: PWM setup already done in pid_dma_motor_setup()
}

void pbdrv_counter_reset(uint8_t id) {
    pid_dma_reset_position(id);
}

int32_t pbdrv_counter_get_count_simple(uint8_t id) {
    int32_t pos;
    pid_dma_get_state(id, &pos, NULL, NULL);
    return pos / 1000;  // Convert millidegrees to degrees
}

void pbdrv_pwm_set_duty_simple(uint8_t id, int16_t duty) {
    pid_dma_set_pwm(id, duty);
}
