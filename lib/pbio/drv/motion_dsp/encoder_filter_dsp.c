// SPDX-License-Identifier: MIT
// DSP-accelerated encoder filtering for RP2350

#include "encoder_filter_dsp.h"
#include <math.h>
#include <string.h>

#ifdef PICO_2W
#include "pico/stdlib.h"
#include "hardware/timer.h"
#endif

// ============================================================================
// Math Helpers
// ============================================================================

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Q15 fixed-point scaling (for biquad coefficients)
#define Q15_SCALE 32768.0f
#define Q15_TO_INT16(x) ((int16_t)((x) * Q15_SCALE))

// ============================================================================
// Biquad IIR Filter Implementation
// ============================================================================

void encoder_filter_init(encoder_biquad_filter_t *filter, float fc, float fs) {
    memset(filter, 0, sizeof(encoder_biquad_filter_t));
    
    // Design 2nd-order Butterworth lowpass filter
    // Bilinear transform method
    float wc = 2.0f * M_PI * fc;
    float T = 1.0f / fs;
    float wc_prewarped = (2.0f / T) * tanf(wc * T / 2.0f);
    
    // Analog prototype coefficients (normalized)
    float w0 = wc_prewarped;
    float Q = 0.7071f;  // Butterworth: Q = 1/sqrt(2)
    
    // Bilinear transform s -> (2/T) * (1-z^-1)/(1+z^-1)
    float K = w0 * T / 2.0f;
    float K2 = K * K;
    float norm = 1.0f / (1.0f + K / Q + K2);
    
    // Digital filter coefficients (normalized)
    float b0 = K2 * norm;
    float b1 = 2.0f * b0;
    float b2 = b0;
    float a0 = 1.0f;
    float a1 = 2.0f * (K2 - 1.0f) * norm;
    float a2 = (1.0f - K / Q + K2) * norm;
    
    // Convert to Q15 fixed-point for DSP operations
    filter->b0 = Q15_TO_INT16(b0);
    filter->b1 = Q15_TO_INT16(b1);
    filter->b2 = Q15_TO_INT16(b2);
    filter->a1 = Q15_TO_INT16(-a1);  // Note: negate for direct form II
    filter->a2 = Q15_TO_INT16(-a2);
    
    // Shift for scaling (prevent overflow in fixed-point math)
    filter->shift = 15;
    
    filter->initialized = true;
}

int32_t encoder_filter_update(encoder_biquad_filter_t *filter, int32_t raw_count) {
    if (!filter->initialized) {
        return raw_count;
    }
    
    // Direct Form II Transposed implementation
    // More numerically stable than Direct Form I
    
    // Input in Q0 format (raw encoder counts)
    int32_t x0 = raw_count;
    
#ifdef ARM_MATH_DSP
    // Use DSP SMLAD instruction for multiply-accumulate
    // SMLAD performs: result = a*b + c*d + accumulator
    
    // Compute w0 = x0 - a1*y1 - a2*y2
    int32_t w0 = x0;
    
    // Multiply-accumulate: w0 += a1*y1
    asm volatile(
        "smlabb %0, %1, %2, %0"
        : "+r"(w0)
        : "r"(filter->a1), "r"((int32_t)filter->y1)
    );
    
    // Multiply-accumulate: w0 += a2*y2
    asm volatile(
        "smlabb %0, %1, %2, %0"
        : "+r"(w0)
        : "r"(filter->a2), "r"((int32_t)filter->y2)
    );
    
    // Compute y0 = b0*w0 + b1*x1 + b2*x2
    int32_t y0 = 0;
    
    // y0 = b0*w0
    asm volatile(
        "smlabb %0, %1, %2, %0"
        : "+r"(y0)
        : "r"(filter->b0), "r"(w0)
    );
    
    // y0 += b1*x1
    asm volatile(
        "smlabb %0, %1, %2, %0"
        : "+r"(y0)
        : "r"(filter->b1), "r"((int32_t)filter->x1)
    );
    
    // y0 += b2*x2
    asm volatile(
        "smlabb %0, %1, %2, %0"
        : "+r"(y0)
        : "r"(filter->b2), "r"((int32_t)filter->x2)
    );
    
    // Scale back from Q15
    y0 >>= filter->shift;
    
#else
    // Scalar implementation for RP2040 or testing
    int32_t w0 = x0 + ((filter->a1 * filter->y1) >> 15) + ((filter->a2 * filter->y2) >> 15);
    int32_t y0 = ((filter->b0 * w0) >> 15) + ((filter->b1 * filter->x1) >> 15) + ((filter->b2 * filter->x2) >> 15);
#endif
    
    // Update state variables
    filter->x2 = filter->x1;
    filter->x1 = w0;
    filter->y2 = filter->y1;
    filter->y1 = y0;
    
    return y0;
}

void encoder_filter_reset(encoder_biquad_filter_t *filter) {
    filter->x1 = 0;
    filter->x2 = 0;
    filter->y1 = 0;
    filter->y2 = 0;
}

// ============================================================================
// Kalman Filter Implementation
// ============================================================================

void encoder_kalman_init(encoder_kalman_t *ek) {
    memset(ek, 0, sizeof(encoder_kalman_t));
    
    // Initial covariance (high uncertainty)
    ek->P00 = 1000 << 16;  // Position variance
    ek->P11 = 10000 << 16; // Velocity variance
    ek->P01 = 0;
    ek->P10 = 0;
    
    // Process noise (tuned for typical motor encoder)
    ek->Q_pos = 10 << 16;    // Small position drift
    ek->Q_vel = 1000 << 16;  // Velocity can change quickly
    
    // Measurement noise (encoder quantization)
    ek->R = 100 << 16;
    
    ek->initialized = true;
}

void encoder_kalman_update(encoder_kalman_t *ek, int32_t position_mdeg) {
    if (!ek->initialized) {
        return;
    }
    
#ifdef PICO_2W
    uint32_t now_us = time_us_32();
#else
    uint32_t now_us = 1000;  // Default 1ms for testing
#endif
    
    // Calculate dt (seconds in Q16.16 fixed-point)
    float dt_sec = 0.001f;  // Default 1ms
    if (ek->last_update_us > 0) {
        dt_sec = (now_us - ek->last_update_us) * 1e-6f;
    }
    ek->last_update_us = now_us;
    
    int32_t dt = (int32_t)(dt_sec * 65536.0f);  // Q16.16 format
    
    // ========== PREDICTION STEP ==========
    // State prediction: x_k|k-1 = F * x_k-1|k-1
    // F = [1  dt]
    //     [0   1]
    
    // pos = pos + vel * dt
    int32_t pos_pred = ek->pos_estimate + ((int64_t)ek->vel_estimate * dt >> 16);
    int32_t vel_pred = ek->vel_estimate;  // Constant velocity model
    
    // Covariance prediction: P_k|k-1 = F * P_k-1|k-1 * F' + Q
    int32_t P00_pred = ek->P00 + ((int64_t)ek->P01 * dt >> 16) + ((int64_t)ek->P10 * dt >> 16) + 
                       ((int64_t)ek->P11 * dt * dt >> 32) + ek->Q_pos;
    int32_t P01_pred = ek->P01 + ((int64_t)ek->P11 * dt >> 16);
    int32_t P10_pred = ek->P10 + ((int64_t)ek->P11 * dt >> 16);
    int32_t P11_pred = ek->P11 + ek->Q_vel;
    
    // ========== UPDATE STEP ==========
    // Innovation: y = z - H * x_k|k-1  (H = [1 0] for position measurement)
    int32_t innovation = position_mdeg - pos_pred;
    
    // Innovation covariance: S = H * P * H' + R
    int32_t S = P00_pred + ek->R;
    
    // Kalman gain: K = P * H' * S^-1
    // K = [P00/S]
    //     [P10/S]
    int64_t K0 = ((int64_t)P00_pred << 16) / S;  // Q16.16
    int64_t K1 = ((int64_t)P10_pred << 16) / S;
    
    // State update: x_k|k = x_k|k-1 + K * innovation
    ek->pos_estimate = pos_pred + ((K0 * innovation) >> 16);
    ek->vel_estimate = vel_pred + ((K1 * innovation) >> 16);
    
    // Covariance update: P_k|k = (I - K*H) * P_k|k-1
    int32_t I_KH = (1 << 16) - K0;  // (1 - K0) in Q16.16
    
    ek->P00 = ((int64_t)I_KH * P00_pred) >> 16;
    ek->P01 = ((int64_t)I_KH * P01_pred) >> 16;
    ek->P10 = P10_pred - ((K1 * P00_pred) >> 16);
    ek->P11 = P11_pred - ((K1 * P01_pred) >> 16);
}

int32_t encoder_kalman_get_velocity(encoder_kalman_t *ek) {
    return ek->vel_estimate;
}

int32_t encoder_kalman_get_position(encoder_kalman_t *ek) {
    return ek->pos_estimate;
}

void encoder_kalman_reset(encoder_kalman_t *ek) {
    ek->pos_estimate = 0;
    ek->vel_estimate = 0;
    ek->last_update_us = 0;
    
    // Reset covariance to high uncertainty
    ek->P00 = 1000 << 16;
    ek->P11 = 10000 << 16;
    ek->P01 = 0;
    ek->P10 = 0;
}

// ============================================================================
// Combined Pipeline Implementation
// ============================================================================

void encoder_pipeline_init(encoder_dsp_pipeline_t *pipeline, float cutoff_hz, float sample_rate_hz) {
    memset(pipeline, 0, sizeof(encoder_dsp_pipeline_t));
    
    encoder_filter_init(&pipeline->biquad, cutoff_hz, sample_rate_hz);
    encoder_kalman_init(&pipeline->kalman);
}

int32_t encoder_pipeline_update(encoder_dsp_pipeline_t *pipeline, int32_t raw_count) {
    // Handle encoder overflow/wrap (assume 16-bit encoder)
    int32_t delta = raw_count - pipeline->last_raw_count;
    if (delta > 32768) {
        delta -= 65536;
    } else if (delta < -32768) {
        delta += 65536;
    }
    pipeline->last_raw_count = raw_count;
    
    // Accumulate to millidegrees (assuming 360000 mdeg per revolution)
    // TODO: Make counts_per_revolution configurable
    const int32_t MDEG_PER_REV = 360000;
    const int32_t COUNTS_PER_REV = 8192;  // Example: 8192-count encoder
    pipeline->cumulative_mdeg += (delta * MDEG_PER_REV) / COUNTS_PER_REV;
    
    // Apply biquad filter to smoothed position
    int32_t filtered_mdeg = encoder_filter_update(&pipeline->biquad, pipeline->cumulative_mdeg);
    
    // Update Kalman filter for position + velocity estimation
    encoder_kalman_update(&pipeline->kalman, filtered_mdeg);
    
    // Track statistics
    pipeline->update_count++;
    int32_t vel = encoder_kalman_get_velocity(&pipeline->kalman);
    if (abs(vel) > pipeline->max_velocity) {
        pipeline->max_velocity = abs(vel);
    }
    
    return encoder_kalman_get_position(&pipeline->kalman);
}

int32_t encoder_pipeline_get_velocity(encoder_dsp_pipeline_t *pipeline) {
    return encoder_kalman_get_velocity(&pipeline->kalman);
}

void encoder_pipeline_reset(encoder_dsp_pipeline_t *pipeline) {
    encoder_filter_reset(&pipeline->biquad);
    encoder_kalman_reset(&pipeline->kalman);
    pipeline->cumulative_mdeg = 0;
    pipeline->last_raw_count = 0;
    pipeline->update_count = 0;
    pipeline->max_velocity = 0;
}
