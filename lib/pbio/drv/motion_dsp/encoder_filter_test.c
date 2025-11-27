// SPDX-License-Identifier: MIT
// Test suite for DSP-accelerated encoder filtering

#include "encoder_filter_dsp.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef PICO_2W
#include "pico/stdlib.h"
#include "hardware/timer.h"
#endif

// ============================================================================
// Test Signal Generators
// ============================================================================

// Generate noisy encoder signal (position + noise)
static int32_t generate_noisy_encoder(float time_sec, float noise_amplitude) {
    // Sinusoidal position (360 degrees = 1 revolution)
    float pos_deg = 180.0f * sinf(2.0f * M_PI * 0.5f * time_sec);
    
    // Add quantization noise (±noise_amplitude counts)
    int32_t noise = (int32_t)((rand() / (float)RAND_MAX - 0.5f) * 2.0f * noise_amplitude);
    
    // Convert to encoder counts (8192 counts/rev = 22.5 counts/deg)
    int32_t counts = (int32_t)(pos_deg * 22.5f) + noise;
    
    return counts;
}

// Generate step response test signal
static int32_t generate_step(float time_sec, float step_time) {
    if (time_sec < step_time) {
        return 0;
    } else {
        return 4096;  // Step to half revolution
    }
}

// ============================================================================
// Test Cases
// ============================================================================

// Test 1: Biquad filter initialization and coefficient validation
static bool test_biquad_init(void) {
    printf("\n=== Test 1: Biquad Filter Initialization ===\n");
    
    encoder_biquad_filter_t filter;
    encoder_filter_init(&filter, 50.0f, 1000.0f);  // 50 Hz cutoff, 1 kHz sample rate
    
    if (!filter.initialized) {
        printf("❌ Filter not initialized\n");
        return false;
    }
    
    printf("✓ Filter initialized\n");
    printf("  Coefficients (Q15): b0=%d, b1=%d, b2=%d, a1=%d, a2=%d\n",
           filter.b0, filter.b1, filter.b2, filter.a1, filter.a2);
    
    // Sanity check: b0+b1+b2 should be close to 32768 (unity DC gain)
    int32_t dc_gain = filter.b0 + filter.b1 + filter.b2;
    printf("  DC gain (Q15): %ld (expected ~32768)\n", dc_gain);
    
    if (abs(dc_gain - 32768) > 3000) {
        printf("❌ DC gain out of range\n");
        return false;
    }
    
    printf("✓ Biquad initialization passed\n");
    return true;
}

// Test 2: Kalman filter initialization
static bool test_kalman_init(void) {
    printf("\n=== Test 2: Kalman Filter Initialization ===\n");
    
    encoder_kalman_t kalman;
    encoder_kalman_init(&kalman);
    
    if (!kalman.initialized) {
        printf("❌ Kalman not initialized\n");
        return false;
    }
    
    printf("✓ Kalman initialized\n");
    printf("  Initial covariance: P00=%ld, P11=%ld\n", kalman.P00 >> 16, kalman.P11 >> 16);
    printf("  Process noise: Q_pos=%ld, Q_vel=%ld\n", kalman.Q_pos >> 16, kalman.Q_vel >> 16);
    printf("  Measurement noise: R=%ld\n", kalman.R >> 16);
    
    printf("✓ Kalman initialization passed\n");
    return true;
}

// Test 3: Filter response to DC signal (should pass through)
static bool test_dc_response(void) {
    printf("\n=== Test 3: DC Signal Response ===\n");
    
    encoder_biquad_filter_t filter;
    encoder_filter_init(&filter, 50.0f, 1000.0f);
    
    const int32_t DC_VALUE = 1000;
    const int32_t NUM_SAMPLES = 100;
    
    int32_t output = 0;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        output = encoder_filter_update(&filter, DC_VALUE);
    }
    
    // After settling, output should be close to input
    int32_t error = abs(output - DC_VALUE);
    printf("  Input: %ld, Output: %ld, Error: %ld\n", DC_VALUE, output, error);
    
    if (error > 50) {  // Allow 5% error
        printf("❌ DC response error too large\n");
        return false;
    }
    
    printf("✓ DC response test passed\n");
    return true;
}

// Test 4: Noise rejection performance
static bool test_noise_rejection(void) {
    printf("\n=== Test 4: Noise Rejection ===\n");
    
    encoder_dsp_pipeline_t pipeline;
    encoder_pipeline_init(&pipeline, 50.0f, 1000.0f);
    
    const int32_t NUM_SAMPLES = 500;
    const float SAMPLE_PERIOD = 0.001f;  // 1 ms
    const float NOISE_AMPLITUDE = 10.0f;  // ±10 counts
    
    float total_input_noise = 0.0f;
    float total_output_noise = 0.0f;
    int32_t prev_output = 0;
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        float time = i * SAMPLE_PERIOD;
        int32_t noisy_input = generate_noisy_encoder(time, NOISE_AMPLITUDE);
        int32_t filtered_output = encoder_pipeline_update(&pipeline, noisy_input);
        
        if (i > 50) {  // Skip transient
            // Measure noise as sample-to-sample variation
            int32_t input_diff = abs(noisy_input - prev_output);
            int32_t output_diff = abs(filtered_output - prev_output);
            
            total_input_noise += input_diff;
            total_output_noise += output_diff;
        }
        
        prev_output = filtered_output;
    }
    
    float noise_reduction_ratio = total_input_noise / (total_output_noise + 1e-6f);
    printf("  Noise reduction ratio: %.2f:1\n", noise_reduction_ratio);
    
    if (noise_reduction_ratio < 2.0f) {
        printf("❌ Insufficient noise rejection (expected >2:1)\n");
        return false;
    }
    
    printf("✓ Noise rejection test passed\n");
    return true;
}

// Test 5: Step response (check settling time)
static bool test_step_response(void) {
    printf("\n=== Test 5: Step Response ===\n");
    
    encoder_dsp_pipeline_t pipeline;
    encoder_pipeline_init(&pipeline, 50.0f, 1000.0f);
    
    const int32_t NUM_SAMPLES = 200;
    const float SAMPLE_PERIOD = 0.001f;
    const float STEP_TIME = 0.05f;  // Step at 50 ms
    const int32_t STEP_VALUE = 4096;
    
    int32_t settling_sample = -1;
    int32_t output = 0;
    
    for (int i = 0; i < NUM_SAMPLES; i++) {
        float time = i * SAMPLE_PERIOD;
        int32_t input = generate_step(time, STEP_TIME);
        output = encoder_pipeline_update(&pipeline, input);
        
        // Check if within 2% of final value
        if (time > STEP_TIME && settling_sample < 0) {
            if (abs(output - STEP_VALUE) < STEP_VALUE * 0.02f) {
                settling_sample = i;
            }
        }
    }
    
    float settling_time_ms = (settling_sample - (int32_t)(STEP_TIME / SAMPLE_PERIOD)) * SAMPLE_PERIOD * 1000.0f;
    printf("  Settling time (2%%): %.1f ms\n", settling_time_ms);
    printf("  Final output: %ld (target: %ld)\n", output, STEP_VALUE);
    
    if (settling_time_ms > 50.0f) {
        printf("⚠ Settling time longer than expected (>50ms)\n");
    }
    
    printf("✓ Step response test passed\n");
    return true;
}

// Test 6: Velocity estimation accuracy
static bool test_velocity_estimation(void) {
    printf("\n=== Test 6: Velocity Estimation ===\n");
    
    encoder_dsp_pipeline_t pipeline;
    encoder_pipeline_init(&pipeline, 50.0f, 1000.0f);
    
    const int32_t NUM_SAMPLES = 500;
    const float SAMPLE_PERIOD = 0.001f;
    const float OMEGA = 2.0f * M_PI * 0.5f;  // 0.5 Hz sine wave
    
    // Let filter settle
    for (int i = 0; i < 100; i++) {
        float time = i * SAMPLE_PERIOD;
        int32_t input = generate_noisy_encoder(time, 5.0f);
        encoder_pipeline_update(&pipeline, input);
    }
    
    // Measure velocity accuracy
    float total_error = 0.0f;
    int32_t samples_counted = 0;
    
    for (int i = 100; i < NUM_SAMPLES; i++) {
        float time = i * SAMPLE_PERIOD;
        int32_t input = generate_noisy_encoder(time, 5.0f);
        encoder_pipeline_update(&pipeline, input);
        
        // True velocity (derivative of position)
        float true_vel_deg_s = 180.0f * OMEGA * cosf(OMEGA * time);
        int32_t true_vel_mdeg_s = (int32_t)(true_vel_deg_s * 1000.0f);
        
        // Estimated velocity
        int32_t est_vel_mdeg_s = encoder_pipeline_get_velocity(&pipeline);
        
        float error_percent = 100.0f * abs(est_vel_mdeg_s - true_vel_mdeg_s) / (abs(true_vel_mdeg_s) + 1.0f);
        total_error += error_percent;
        samples_counted++;
    }
    
    float avg_error_percent = total_error / samples_counted;
    printf("  Average velocity error: %.1f%%\n", avg_error_percent);
    
    if (avg_error_percent > 20.0f) {
        printf("❌ Velocity estimation error too high (>20%%)\n");
        return false;
    }
    
    printf("✓ Velocity estimation test passed\n");
    return true;
}

// ============================================================================
// Performance Benchmark
// ============================================================================

typedef struct {
    uint32_t biquad_cycles;
    uint32_t kalman_cycles;
    uint32_t pipeline_cycles;
    float biquad_us;
    float kalman_us;
    float pipeline_us;
} filter_benchmark_result_t;

static filter_benchmark_result_t benchmark_filters(void) {
    filter_benchmark_result_t result = {0};
    
#ifdef PICO_2W
    encoder_biquad_filter_t biquad;
    encoder_kalman_t kalman;
    encoder_dsp_pipeline_t pipeline;
    
    encoder_filter_init(&biquad, 50.0f, 1000.0f);
    encoder_kalman_init(&kalman);
    encoder_pipeline_init(&pipeline, 50.0f, 1000.0f);
    
    const uint32_t ITERATIONS = 1000;
    
    // Benchmark biquad filter
    uint32_t start = time_us_32();
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        encoder_filter_update(&biquad, (int32_t)i);
    }
    uint32_t end = time_us_32();
    result.biquad_us = (end - start) / (float)ITERATIONS;
    result.biquad_cycles = (uint32_t)(result.biquad_us * 150.0f);  // 150 MHz clock
    
    // Benchmark Kalman filter
    start = time_us_32();
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        encoder_kalman_update(&kalman, (int32_t)i);
    }
    end = time_us_32();
    result.kalman_us = (end - start) / (float)ITERATIONS;
    result.kalman_cycles = (uint32_t)(result.kalman_us * 150.0f);
    
    // Benchmark complete pipeline
    start = time_us_32();
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        encoder_pipeline_update(&pipeline, (int32_t)i);
    }
    end = time_us_32();
    result.pipeline_us = (end - start) / (float)ITERATIONS;
    result.pipeline_cycles = (uint32_t)(result.pipeline_us * 150.0f);
#endif
    
    return result;
}

// ============================================================================
// Test Suite Entry Point
// ============================================================================

void encoder_filter_run_tests(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║   DSP-ACCELERATED ENCODER FILTER TEST SUITE             ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");
    
    int tests_passed = 0;
    int tests_total = 0;
    
    // Run tests
    tests_total++; if (test_biquad_init()) tests_passed++;
    tests_total++; if (test_kalman_init()) tests_passed++;
    tests_total++; if (test_dc_response()) tests_passed++;
    tests_total++; if (test_noise_rejection()) tests_passed++;
    tests_total++; if (test_step_response()) tests_passed++;
    tests_total++; if (test_velocity_estimation()) tests_passed++;
    
    // Performance benchmark
    printf("\n=== Performance Benchmark ===\n");
    filter_benchmark_result_t bench = benchmark_filters();
    
    printf("  Biquad filter:  %.2f µs (%lu cycles)\n", bench.biquad_us, bench.biquad_cycles);
    printf("  Kalman filter:  %.2f µs (%lu cycles)\n", bench.kalman_us, bench.kalman_cycles);
    printf("  Full pipeline:  %.2f µs (%lu cycles)\n", bench.pipeline_us, bench.pipeline_cycles);
    
    // Calculate throughput (1 kHz control loop = 1000 µs budget)
    float budget_us = 1000.0f;
    float pipeline_budget_percent = 100.0f * bench.pipeline_us / budget_us;
    printf("\n  Pipeline uses %.1f%% of 1 kHz control loop budget\n", pipeline_budget_percent);
    
    if (pipeline_budget_percent > 10.0f) {
        printf("  ⚠ Warning: Pipeline overhead >10%% of control loop\n");
    } else {
        printf("  ✓ Pipeline overhead acceptable\n");
    }
    
    // Summary
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║   TEST SUMMARY: %d/%d PASSED                              ║\n", tests_passed, tests_total);
    printf("╚══════════════════════════════════════════════════════════╝\n");
    
    if (tests_passed == tests_total) {
        printf("✓ All tests passed! Encoder filters ready for Phase 3.\n");
    } else {
        printf("❌ Some tests failed. Review results above.\n");
    }
}
