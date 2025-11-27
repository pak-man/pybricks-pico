// SPDX-License-Identifier: MIT
// DSP-accelerated encoder filtering for RP2350

#ifndef _ENCODER_FILTER_DSP_H_
#define _ENCODER_FILTER_DSP_H_

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Biquad IIR Filter (2nd-order sections)
// ============================================================================

// Biquad IIR filter for encoder smoothing
// Better phase response than moving average, lower latency than FIR
typedef struct {
    // Filter coefficients (Q15 fixed-point for DSP efficiency)
    int16_t b0, b1, b2;  // Numerator coefficients
    int16_t a1, a2;      // Denominator coefficients (a0 normalized to 1)
    
    // State variables (previous inputs/outputs)
    int32_t x1, x2;      // Previous inputs
    int32_t y1, y2;      // Previous outputs
    
    // Shift for scaling (to prevent overflow)
    int32_t shift;
    
    bool initialized;
} encoder_biquad_filter_t;

// Initialize 2nd-order Butterworth lowpass filter
// fc = cutoff frequency (Hz)
// fs = sample rate (Hz, typically 1000 for motor control)
void encoder_filter_init(encoder_biquad_filter_t *filter, float fc, float fs);

// Filter encoder count (handles overflow/wrap-around)
// Returns filtered position in same units as input
int32_t encoder_filter_update(encoder_biquad_filter_t *filter, int32_t raw_count);

// Reset filter state (use when position is reset)
void encoder_filter_reset(encoder_biquad_filter_t *filter);

// ============================================================================
// Kalman-style State Estimator for Velocity
// ============================================================================

// Kalman filter for position + velocity estimation
// Provides smooth velocity estimate with minimal lag
typedef struct {
    // State vector: [position, velocity] in Q16.16 fixed-point
    int32_t pos_estimate;   // Position (mdeg)
    int32_t vel_estimate;   // Velocity (mdeg/s)
    
    // Covariance matrix (2x2, stored as 4 elements)
    // P = [P00 P01]
    //     [P10 P11]
    int32_t P00, P01, P10, P11;
    
    // Process noise covariance
    int32_t Q_pos;  // Position process noise
    int32_t Q_vel;  // Velocity process noise
    
    // Measurement noise covariance
    int32_t R;      // Position measurement noise
    
    // Timestamp tracking for dt calculation
    uint32_t last_update_us;
    
    bool initialized;
} encoder_kalman_t;

// Initialize Kalman filter with default noise parameters
void encoder_kalman_init(encoder_kalman_t *ek);

// Update Kalman filter with new position measurement
// position_mdeg: measured position in millidegrees
void encoder_kalman_update(encoder_kalman_t *ek, int32_t position_mdeg);

// Get estimated velocity (mdeg/s)
int32_t encoder_kalman_get_velocity(encoder_kalman_t *ek);

// Get estimated position (mdeg)
int32_t encoder_kalman_get_position(encoder_kalman_t *ek);

// Reset Kalman filter state
void encoder_kalman_reset(encoder_kalman_t *ek);

// ============================================================================
// Combined Filter (Biquad + Kalman)
// ============================================================================

// High-performance encoder processing pipeline
typedef struct {
    encoder_biquad_filter_t biquad;
    encoder_kalman_t kalman;
    
    // Raw encoder tracking
    int32_t last_raw_count;
    int32_t cumulative_mdeg;
    
    // Statistics
    uint32_t update_count;
    int32_t max_velocity;
    
} encoder_dsp_pipeline_t;

// Initialize complete encoder processing pipeline
void encoder_pipeline_init(encoder_dsp_pipeline_t *pipeline, float cutoff_hz, float sample_rate_hz);

// Process raw encoder count through pipeline
// Returns filtered position in millidegrees
int32_t encoder_pipeline_update(encoder_dsp_pipeline_t *pipeline, int32_t raw_count);

// Get velocity estimate from pipeline (mdeg/s)
int32_t encoder_pipeline_get_velocity(encoder_dsp_pipeline_t *pipeline);

// Reset pipeline state
void encoder_pipeline_reset(encoder_dsp_pipeline_t *pipeline);

#endif // _ENCODER_FILTER_DSP_H_
