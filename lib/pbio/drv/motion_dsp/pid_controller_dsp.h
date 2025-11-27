// SPDX-License-Identifier: MIT
// DSP-accelerated PID controller for RP2350

#ifndef _PID_CONTROLLER_DSP_H_
#define _PID_CONTROLLER_DSP_H_

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Basic PID Controller with DSP Acceleration
// ============================================================================

// PID controller state
typedef struct {
    // Gains (Q16.16 fixed-point for precision)
    int32_t Kp;              // Proportional gain
    int32_t Ki;              // Integral gain
    int32_t Kd;              // Derivative gain
    
    // State variables
    int32_t integral;        // Integral accumulator
    int32_t prev_error;      // Previous error for derivative
    int32_t prev_input;      // Previous input for derivative-on-measurement
    
    // Output limits
    int32_t output_min;      // Minimum output (mdeg or mA)
    int32_t output_max;      // Maximum output (mdeg or mA)
    
    // Integral anti-windup
    int32_t integral_min;    // Minimum integral value
    int32_t integral_max;    // Maximum integral value
    bool use_back_calc;      // Use back-calculation anti-windup
    int32_t Kb;              // Back-calculation gain (Q16.16)
    
    // Derivative filter (for noise reduction)
    int32_t deriv_lpf_alpha; // Low-pass filter coefficient (Q15)
    int32_t filtered_deriv;  // Filtered derivative
    
    // Sample time
    float dt;                // Sample period (seconds)
    
    // Options
    bool derivative_on_measurement;  // Use derivative of input instead of error
    bool reset_on_setpoint_change;   // Reset integral on setpoint change
    
    // Statistics
    uint32_t update_count;
    int32_t max_error;
    int32_t max_output;
    
    bool initialized;
} pid_controller_t;

// Initialize PID controller
// Kp, Ki, Kd in regular floating-point (will be converted to Q16.16)
// dt = sample time in seconds (e.g., 0.001 for 1 kHz)
void pid_init(pid_controller_t *pid, float Kp, float Ki, float Kd, float dt);

// Update PID controller
// setpoint, measurement in millidegrees
// Returns control output in same units
int32_t pid_update(pid_controller_t *pid, int32_t setpoint, int32_t measurement);

// Update PID with velocity feedforward (uses filtered velocity from encoder pipeline)
int32_t pid_update_with_velocity(pid_controller_t *pid, int32_t setpoint, 
                                  int32_t measurement, int32_t velocity);

// Set output limits
void pid_set_limits(pid_controller_t *pid, int32_t min, int32_t max);

// Set integral limits (for anti-windup)
void pid_set_integral_limits(pid_controller_t *pid, int32_t min, int32_t max);

// Enable back-calculation anti-windup
void pid_enable_back_calculation(pid_controller_t *pid, float Kb);

// Reset PID state (clear integral, derivative)
void pid_reset(pid_controller_t *pid);

// ============================================================================
// Cascade PID Controller (Position Outer Loop + Velocity Inner Loop)
// ============================================================================

// Cascade controller with position outer loop and velocity inner loop
typedef struct {
    pid_controller_t position_pid;  // Outer loop (position control)
    pid_controller_t velocity_pid;  // Inner loop (velocity control)
    
    // Feed-forward gains
    int32_t velocity_ff;     // Velocity feed-forward (Q16.16)
    int32_t accel_ff;        // Acceleration feed-forward (Q16.16)
    
    // Trajectory tracking
    int32_t setpoint_position;
    int32_t setpoint_velocity;
    int32_t setpoint_accel;
    
    // Previous setpoint for derivative calculation
    int32_t prev_setpoint_pos;
    int32_t prev_setpoint_vel;
    
    bool initialized;
} cascade_pid_t;

// Initialize cascade controller
// pos_Kp, pos_Ki, pos_Kd = position loop gains
// vel_Kp, vel_Ki, vel_Kd = velocity loop gains
// dt = sample time in seconds
void cascade_pid_init(cascade_pid_t *cascade,
                      float pos_Kp, float pos_Ki, float pos_Kd,
                      float vel_Kp, float vel_Ki, float vel_Kd,
                      float dt);

// Update cascade controller
// position_sp = position setpoint (mdeg)
// position = measured position (mdeg)
// velocity = measured velocity (mdeg/s)
// Returns motor command output
int32_t cascade_pid_update(cascade_pid_t *cascade,
                           int32_t position_sp,
                           int32_t position,
                           int32_t velocity);

// Update cascade with trajectory (position + velocity + acceleration)
int32_t cascade_pid_update_trajectory(cascade_pid_t *cascade,
                                       int32_t pos_sp, int32_t vel_sp, int32_t accel_sp,
                                       int32_t position, int32_t velocity);

// Set feed-forward gains
void cascade_pid_set_feedforward(cascade_pid_t *cascade, float vel_ff, float accel_ff);

// Reset cascade controller
void cascade_pid_reset(cascade_pid_t *cascade);

// ============================================================================
// Adaptive Gain Scheduling
// ============================================================================

// Gain schedule entry (varies gains based on velocity)
typedef struct {
    int32_t velocity_threshold;  // Velocity threshold (mdeg/s)
    float Kp_scale;              // Proportional gain scale
    float Ki_scale;              // Integral gain scale
    float Kd_scale;              // Derivative gain scale
} gain_schedule_entry_t;

// Adaptive PID with gain scheduling
typedef struct {
    pid_controller_t base_pid;   // Base PID controller
    
    // Gain schedule table
    gain_schedule_entry_t schedule[8];  // Up to 8 velocity ranges
    uint8_t num_entries;
    
    // Current scaled gains
    int32_t current_Kp;
    int32_t current_Ki;
    int32_t current_Kd;
    
    // Base gains (unscaled)
    int32_t base_Kp;
    int32_t base_Ki;
    int32_t base_Kd;
    
    bool initialized;
} adaptive_pid_t;

// Initialize adaptive PID
void adaptive_pid_init(adaptive_pid_t *apid, float Kp, float Ki, float Kd, float dt);

// Add gain schedule entry
// velocity_threshold in mdeg/s, scales are multipliers (e.g., 1.0 = no change)
void adaptive_pid_add_schedule(adaptive_pid_t *apid, int32_t velocity_threshold,
                                float Kp_scale, float Ki_scale, float Kd_scale);

// Update adaptive PID (automatically adjusts gains based on velocity)
int32_t adaptive_pid_update(adaptive_pid_t *apid, int32_t setpoint,
                             int32_t measurement, int32_t velocity);

// Reset adaptive PID
void adaptive_pid_reset(adaptive_pid_t *apid);

// ============================================================================
// Performance Monitoring
// ============================================================================

// PID performance metrics
typedef struct {
    int32_t avg_error;           // Average error (mdeg)
    int32_t rms_error;           // RMS error (mdeg)
    int32_t max_error;           // Maximum error (mdeg)
    int32_t settling_time_ms;    // Settling time to ±2% (ms)
    float overshoot_percent;     // Overshoot percentage
    uint32_t samples;            // Number of samples
} pid_performance_t;

// Calculate performance metrics
void pid_calculate_performance(pid_controller_t *pid, pid_performance_t *perf);

#endif // _PID_CONTROLLER_DSP_H_
