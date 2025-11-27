// SPDX-License-Identifier: MIT
// DSP-accelerated PID controller for RP2350

#include "pid_controller_dsp.h"
#include <math.h>
#include <string.h>

#ifdef PICO_2W
#include "pico/stdlib.h"
#endif

// ============================================================================
// Fixed-Point Helpers
// ============================================================================

#define Q16_16(x) ((int32_t)((x) * 65536.0f))
#define Q16_16_TO_FLOAT(x) ((float)(x) / 65536.0f)
#define Q15(x) ((int16_t)((x) * 32768.0f))

// Saturate to prevent overflow
static inline int32_t saturate_add(int32_t a, int32_t b) {
    int64_t result = (int64_t)a + (int64_t)b;
    if (result > 0x7FFFFFFF) return 0x7FFFFFFF;
    if (result < -0x7FFFFFFF) return -0x7FFFFFFF;
    return (int32_t)result;
}

static inline int32_t saturate(int32_t value, int32_t min, int32_t max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

// ============================================================================
// Basic PID Controller
// ============================================================================

void pid_init(pid_controller_t *pid, float Kp, float Ki, float Kd, float dt) {
    memset(pid, 0, sizeof(pid_controller_t));
    
    // Convert gains to Q16.16 fixed-point
    pid->Kp = Q16_16(Kp);
    pid->Ki = Q16_16(Ki * dt);  // Pre-multiply Ki by dt
    pid->Kd = Q16_16(Kd / dt);  // Pre-divide Kd by dt
    
    pid->dt = dt;
    
    // Default limits (very large)
    pid->output_min = -1000000;
    pid->output_max = 1000000;
    pid->integral_min = -1000000;
    pid->integral_max = 1000000;
    
    // Default derivative filter (100 Hz LPF for 1 kHz sample rate)
    // alpha = dt / (dt + 1/(2*pi*fc)) for fc=100Hz, dt=0.001s
    // alpha ≈ 0.386
    pid->deriv_lpf_alpha = Q15(0.386f);
    
    // Default options
    pid->derivative_on_measurement = true;  // Prevents derivative kick
    pid->reset_on_setpoint_change = false;
    pid->use_back_calc = false;
    
    pid->initialized = true;
}

int32_t pid_update(pid_controller_t *pid, int32_t setpoint, int32_t measurement) {
    if (!pid->initialized) {
        return 0;
    }
    
    // Calculate error
    int32_t error = setpoint - measurement;
    
    // ========== PROPORTIONAL TERM ==========
    // P = Kp * error
    int64_t p_term = ((int64_t)pid->Kp * error) >> 16;
    
    // ========== INTEGRAL TERM ==========
    // I = Ki * sum(error)
    // Note: Ki already includes dt from init
    pid->integral = saturate_add(pid->integral, error);
    pid->integral = saturate(pid->integral, pid->integral_min, pid->integral_max);
    int64_t i_term = ((int64_t)pid->Ki * pid->integral) >> 16;
    
    // ========== DERIVATIVE TERM ==========
    int32_t derivative;
    if (pid->derivative_on_measurement) {
        // Derivative on measurement (prevents derivative kick on setpoint change)
        derivative = measurement - pid->prev_input;
        pid->prev_input = measurement;
    } else {
        // Derivative on error
        derivative = error - pid->prev_error;
    }
    
    // Apply low-pass filter to derivative (reduce noise)
    // filtered = alpha * new + (1-alpha) * old
#ifdef ARM_MATH_DSP
    // Use DSP instruction for multiply-accumulate
    int32_t alpha = pid->deriv_lpf_alpha;
    int32_t one_minus_alpha = 32768 - alpha;
    
    int32_t filtered = 0;
    asm volatile(
        "smlabb %0, %1, %2, %0\n"  // filtered += alpha * derivative
        "smlabb %0, %3, %4, %0\n"  // filtered += (1-alpha) * old
        : "+r"(filtered)
        : "r"(alpha), "r"(derivative), "r"(one_minus_alpha), "r"(pid->filtered_deriv)
    );
    filtered >>= 15;  // Scale from Q15
    pid->filtered_deriv = filtered;
#else
    // Scalar implementation
    int32_t filtered = ((pid->deriv_lpf_alpha * derivative) >> 15) + 
                       (((32768 - pid->deriv_lpf_alpha) * pid->filtered_deriv) >> 15);
    pid->filtered_deriv = filtered;
#endif
    
    int64_t d_term = ((int64_t)pid->Kd * pid->filtered_deriv) >> 16;
    
    pid->prev_error = error;
    
    // ========== TOTAL OUTPUT ==========
    int32_t output = (int32_t)(p_term + i_term + d_term);
    
    // Apply output limits
    int32_t limited_output = saturate(output, pid->output_min, pid->output_max);
    
    // ========== ANTI-WINDUP ==========
    if (pid->use_back_calc && (limited_output != output)) {
        // Back-calculation anti-windup
        // Reduce integral based on saturation amount
        int32_t saturation_error = limited_output - output;
        int32_t integral_correction = ((int64_t)pid->Kb * saturation_error) >> 16;
        pid->integral = saturate_add(pid->integral, integral_correction);
        pid->integral = saturate(pid->integral, pid->integral_min, pid->integral_max);
    }
    
    // Update statistics
    pid->update_count++;
    if (abs(error) > pid->max_error) {
        pid->max_error = abs(error);
    }
    if (abs(limited_output) > pid->max_output) {
        pid->max_output = abs(limited_output);
    }
    
    return limited_output;
}

int32_t pid_update_with_velocity(pid_controller_t *pid, int32_t setpoint,
                                  int32_t measurement, int32_t velocity) {
    // Standard PID update
    int32_t output = pid_update(pid, setpoint, measurement);
    
    // Velocity can be used for feed-forward in cascade controller
    // For basic PID, just return standard output
    return output;
}

void pid_set_limits(pid_controller_t *pid, int32_t min, int32_t max) {
    pid->output_min = min;
    pid->output_max = max;
}

void pid_set_integral_limits(pid_controller_t *pid, int32_t min, int32_t max) {
    pid->integral_min = min;
    pid->integral_max = max;
}

void pid_enable_back_calculation(pid_controller_t *pid, float Kb) {
    pid->use_back_calc = true;
    pid->Kb = Q16_16(Kb);
}

void pid_reset(pid_controller_t *pid) {
    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_input = 0;
    pid->filtered_deriv = 0;
    pid->update_count = 0;
    pid->max_error = 0;
    pid->max_output = 0;
}

// ============================================================================
// Cascade PID Controller
// ============================================================================

void cascade_pid_init(cascade_pid_t *cascade,
                      float pos_Kp, float pos_Ki, float pos_Kd,
                      float vel_Kp, float vel_Ki, float vel_Kd,
                      float dt) {
    memset(cascade, 0, sizeof(cascade_pid_t));
    
    // Initialize position PID (outer loop)
    pid_init(&cascade->position_pid, pos_Kp, pos_Ki, pos_Kd, dt);
    cascade->position_pid.derivative_on_measurement = true;
    
    // Initialize velocity PID (inner loop)
    pid_init(&cascade->velocity_pid, vel_Kp, vel_Ki, vel_Kd, dt);
    cascade->velocity_pid.derivative_on_measurement = true;
    
    // Default feed-forward gains (disabled)
    cascade->velocity_ff = 0;
    cascade->accel_ff = 0;
    
    cascade->initialized = true;
}

int32_t cascade_pid_update(cascade_pid_t *cascade,
                           int32_t position_sp,
                           int32_t position,
                           int32_t velocity) {
    if (!cascade->initialized) {
        return 0;
    }
    
    // ========== OUTER LOOP (POSITION) ==========
    // Position controller outputs desired velocity
    int32_t desired_velocity = pid_update(&cascade->position_pid, position_sp, position);
    
    // ========== INNER LOOP (VELOCITY) ==========
    // Velocity controller outputs motor command
    int32_t motor_command = pid_update(&cascade->velocity_pid, desired_velocity, velocity);
    
    return motor_command;
}

int32_t cascade_pid_update_trajectory(cascade_pid_t *cascade,
                                       int32_t pos_sp, int32_t vel_sp, int32_t accel_sp,
                                       int32_t position, int32_t velocity) {
    if (!cascade->initialized) {
        return 0;
    }
    
    // ========== OUTER LOOP WITH FEED-FORWARD ==========
    // Position error
    int32_t pos_error = pos_sp - position;
    
    // Position PID output
    int32_t pos_correction = pid_update(&cascade->position_pid, pos_sp, position);
    
    // Desired velocity = setpoint velocity + position correction + velocity feed-forward
    int64_t desired_velocity = vel_sp + pos_correction;
    
    // Add velocity feed-forward
    if (cascade->velocity_ff != 0) {
        desired_velocity += ((int64_t)cascade->velocity_ff * vel_sp) >> 16;
    }
    
    // ========== INNER LOOP WITH FEED-FORWARD ==========
    int32_t vel_correction = pid_update(&cascade->velocity_pid, 
                                        (int32_t)desired_velocity, velocity);
    
    // Motor command = velocity correction + acceleration feed-forward
    int32_t motor_command = vel_correction;
    if (cascade->accel_ff != 0) {
        motor_command += ((int64_t)cascade->accel_ff * accel_sp) >> 16;
    }
    
    // Store setpoints for next iteration
    cascade->prev_setpoint_pos = pos_sp;
    cascade->prev_setpoint_vel = vel_sp;
    
    return motor_command;
}

void cascade_pid_set_feedforward(cascade_pid_t *cascade, float vel_ff, float accel_ff) {
    cascade->velocity_ff = Q16_16(vel_ff);
    cascade->accel_ff = Q16_16(accel_ff);
}

void cascade_pid_reset(cascade_pid_t *cascade) {
    pid_reset(&cascade->position_pid);
    pid_reset(&cascade->velocity_pid);
    cascade->prev_setpoint_pos = 0;
    cascade->prev_setpoint_vel = 0;
}

// ============================================================================
// Adaptive Gain Scheduling
// ============================================================================

void adaptive_pid_init(adaptive_pid_t *apid, float Kp, float Ki, float Kd, float dt) {
    memset(apid, 0, sizeof(adaptive_pid_t));
    
    // Initialize base PID
    pid_init(&apid->base_pid, Kp, Ki, Kd, dt);
    
    // Store base gains
    apid->base_Kp = apid->base_pid.Kp;
    apid->base_Ki = apid->base_pid.Ki;
    apid->base_Kd = apid->base_pid.Kd;
    
    // Current gains start as base gains
    apid->current_Kp = apid->base_Kp;
    apid->current_Ki = apid->base_Ki;
    apid->current_Kd = apid->base_Kd;
    
    apid->num_entries = 0;
    apid->initialized = true;
}

void adaptive_pid_add_schedule(adaptive_pid_t *apid, int32_t velocity_threshold,
                                float Kp_scale, float Ki_scale, float Kd_scale) {
    if (apid->num_entries >= 8) {
        return;  // Table full
    }
    
    gain_schedule_entry_t *entry = &apid->schedule[apid->num_entries];
    entry->velocity_threshold = velocity_threshold;
    entry->Kp_scale = Kp_scale;
    entry->Ki_scale = Ki_scale;
    entry->Kd_scale = Kd_scale;
    
    apid->num_entries++;
    
    // Sort by velocity threshold (ascending)
    for (uint8_t i = apid->num_entries - 1; i > 0; i--) {
        if (apid->schedule[i].velocity_threshold < apid->schedule[i-1].velocity_threshold) {
            gain_schedule_entry_t temp = apid->schedule[i];
            apid->schedule[i] = apid->schedule[i-1];
            apid->schedule[i-1] = temp;
        } else {
            break;
        }
    }
}

int32_t adaptive_pid_update(adaptive_pid_t *apid, int32_t setpoint,
                             int32_t measurement, int32_t velocity) {
    if (!apid->initialized) {
        return 0;
    }
    
    // Determine which gain schedule to use based on velocity
    int32_t abs_velocity = abs(velocity);
    
    float Kp_scale = 1.0f;
    float Ki_scale = 1.0f;
    float Kd_scale = 1.0f;
    
    // Find appropriate schedule entry
    for (uint8_t i = 0; i < apid->num_entries; i++) {
        if (abs_velocity >= apid->schedule[i].velocity_threshold) {
            Kp_scale = apid->schedule[i].Kp_scale;
            Ki_scale = apid->schedule[i].Ki_scale;
            Kd_scale = apid->schedule[i].Kd_scale;
        } else {
            break;
        }
    }
    
    // Apply gain scaling
    apid->base_pid.Kp = (int32_t)(((int64_t)apid->base_Kp * Q16_16(Kp_scale)) >> 16);
    apid->base_pid.Ki = (int32_t)(((int64_t)apid->base_Ki * Q16_16(Ki_scale)) >> 16);
    apid->base_pid.Kd = (int32_t)(((int64_t)apid->base_Kd * Q16_16(Kd_scale)) >> 16);
    
    apid->current_Kp = apid->base_pid.Kp;
    apid->current_Ki = apid->base_pid.Ki;
    apid->current_Kd = apid->base_pid.Kd;
    
    // Update PID with scaled gains
    return pid_update(&apid->base_pid, setpoint, measurement);
}

void adaptive_pid_reset(adaptive_pid_t *apid) {
    pid_reset(&apid->base_pid);
    apid->current_Kp = apid->base_Kp;
    apid->current_Ki = apid->base_Ki;
    apid->current_Kd = apid->base_Kd;
}

// ============================================================================
// Performance Monitoring
// ============================================================================

void pid_calculate_performance(pid_controller_t *pid, pid_performance_t *perf) {
    // This would need to be called with a history buffer
    // For now, just return basic stats from PID
    memset(perf, 0, sizeof(pid_performance_t));
    
    perf->max_error = pid->max_error;
    perf->samples = pid->update_count;
    
    // Other metrics require storing error history
    // TODO: Implement error history tracking if needed
}
