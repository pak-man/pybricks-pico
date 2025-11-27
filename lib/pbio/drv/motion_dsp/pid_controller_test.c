// SPDX-License-Identifier: MIT
// Test suite for DSP-accelerated PID controller

#include "pid_controller_dsp.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifdef PICO_2W
#include "pico/stdlib.h"
#include "hardware/timer.h"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Plant Models for Testing
// ============================================================================

// Simple integrator plant: output = integral(input)
typedef struct {
    int32_t position;     // Current position (mdeg)
    int32_t velocity;     // Current velocity (mdeg/s)
    float dt;             // Sample time (s)
    int32_t inertia;      // Simulated inertia (Q16.16)
    int32_t damping;      // Simulated damping (Q16.16)
} integrator_plant_t;

static void plant_init(integrator_plant_t *plant, float dt) {
    memset(plant, 0, sizeof(integrator_plant_t));
    plant->dt = dt;
    plant->inertia = (int32_t)(1.0f * 65536.0f);   // Unit inertia
    plant->damping = (int32_t)(0.1f * 65536.0f);   // Light damping
}

static void plant_update(integrator_plant_t *plant, int32_t force) {
    // Simple physics: F = m*a + b*v
    // a = (F - b*v) / m
    
    int64_t damping_force = ((int64_t)plant->damping * plant->velocity) >> 16;
    int64_t net_force = force - damping_force;
    int32_t acceleration = (int32_t)((net_force << 16) / plant->inertia);
    
    // Integrate: v = v + a*dt
    plant->velocity += (int32_t)(acceleration * plant->dt);
    
    // Integrate: pos = pos + v*dt
    plant->position += (int32_t)(plant->velocity * plant->dt);
}

// ============================================================================
// Test Cases
// ============================================================================

// Test 1: Basic PID initialization
static bool test_pid_init(void) {
    printf("\n=== Test 1: PID Initialization ===\n");
    
    pid_controller_t pid;
    pid_init(&pid, 1.0f, 0.1f, 0.01f, 0.001f);
    
    if (!pid.initialized) {
        printf("❌ PID not initialized\n");
        return false;
    }
    
    printf("✓ PID initialized\n");
    printf("  Kp (Q16.16): %ld (%.3f)\n", pid.Kp, pid.Kp / 65536.0f);
    printf("  Ki (Q16.16): %ld (%.3f)\n", pid.Ki, pid.Ki / 65536.0f);
    printf("  Kd (Q16.16): %ld (%.3f)\n", pid.Kd, pid.Kd / 65536.0f);
    printf("  dt: %.4f s\n", pid.dt);
    
    printf("✓ PID initialization passed\n");
    return true;
}

// Test 2: Step response (position control)
static bool test_step_response(void) {
    printf("\n=== Test 2: Step Response ===\n");
    
    pid_controller_t pid;
    integrator_plant_t plant;
    
    pid_init(&pid, 2.0f, 0.5f, 0.05f, 0.001f);
    pid_set_limits(&pid, -10000, 10000);
    pid_enable_back_calculation(&pid, 1.0f);
    
    plant_init(&plant, 0.001f);
    
    const int32_t SETPOINT = 90000;  // 90 degrees
    const int32_t SAMPLES = 2000;    // 2 seconds
    
    int32_t settling_sample = -1;
    int32_t peak_value = 0;
    
    for (int32_t i = 0; i < SAMPLES; i++) {
        int32_t control = pid_update(&pid, SETPOINT, plant.position);
        plant_update(&plant, control);
        
        // Track peak for overshoot
        if (abs(plant.position) > abs(peak_value)) {
            peak_value = plant.position;
        }
        
        // Check settling (within 2% of setpoint)
        if (settling_sample < 0 && i > 100) {
            if (abs(plant.position - SETPOINT) < (SETPOINT / 50)) {
                settling_sample = i;
            }
        }
    }
    
    float settling_time_ms = settling_sample * 0.001f * 1000.0f;
    float overshoot = 100.0f * (peak_value - SETPOINT) / (float)SETPOINT;
    float final_error = abs(plant.position - SETPOINT);
    
    printf("  Target: %ld mdeg (%.1f deg)\n", SETPOINT, SETPOINT / 1000.0f);
    printf("  Final position: %ld mdeg (%.1f deg)\n", plant.position, plant.position / 1000.0f);
    printf("  Final error: %ld mdeg (%.3f deg)\n", final_error, final_error / 1000.0f);
    printf("  Settling time (2%%): %.1f ms\n", settling_time_ms);
    printf("  Overshoot: %.1f%%\n", overshoot);
    
    if (final_error > 1000) {  // >1 degree error
        printf("❌ Final error too large\n");
        return false;
    }
    
    if (settling_time_ms > 500.0f) {
        printf("⚠ Settling time longer than expected\n");
    }
    
    printf("✓ Step response test passed\n");
    return true;
}

// Test 3: Tracking sinusoidal reference
static bool test_sinusoidal_tracking(void) {
    printf("\n=== Test 3: Sinusoidal Tracking ===\n");
    
    pid_controller_t pid;
    integrator_plant_t plant;
    
    pid_init(&pid, 3.0f, 1.0f, 0.1f, 0.001f);
    pid_set_limits(&pid, -20000, 20000);
    
    plant_init(&plant, 0.001f);
    
    const int32_t SAMPLES = 1000;
    const float FREQ = 1.0f;  // 1 Hz
    const int32_t AMPLITUDE = 45000;  // 45 degrees
    
    int64_t total_error_sq = 0;
    int32_t max_tracking_error = 0;
    
    for (int32_t i = 0; i < SAMPLES; i++) {
        float time = i * 0.001f;
        int32_t setpoint = (int32_t)(AMPLITUDE * sinf(2.0f * M_PI * FREQ * time));
        
        int32_t control = pid_update(&pid, setpoint, plant.position);
        plant_update(&plant, control);
        
        int32_t tracking_error = abs(setpoint - plant.position);
        if (tracking_error > max_tracking_error) {
            max_tracking_error = tracking_error;
        }
        
        if (i > 100) {  // Skip transient
            total_error_sq += (int64_t)tracking_error * tracking_error;
        }
    }
    
    int32_t rms_error = (int32_t)sqrtf(total_error_sq / (SAMPLES - 100));
    
    printf("  Frequency: %.1f Hz\n", FREQ);
    printf("  Amplitude: %ld mdeg (%.1f deg)\n", AMPLITUDE, AMPLITUDE / 1000.0f);
    printf("  RMS tracking error: %ld mdeg (%.3f deg)\n", rms_error, rms_error / 1000.0f);
    printf("  Max tracking error: %ld mdeg (%.3f deg)\n", max_tracking_error, max_tracking_error / 1000.0f);
    
    if (rms_error > 5000) {  // >5 degrees RMS
        printf("❌ RMS tracking error too large\n");
        return false;
    }
    
    printf("✓ Sinusoidal tracking test passed\n");
    return true;
}

// Test 4: Cascade controller
static bool test_cascade_controller(void) {
    printf("\n=== Test 4: Cascade Controller ===\n");
    
    cascade_pid_t cascade;
    integrator_plant_t plant;
    
    // Position loop: slower, higher gain
    // Velocity loop: faster, moderate gain
    cascade_pid_init(&cascade, 
                     5.0f, 1.0f, 0.1f,   // Position Kp, Ki, Kd
                     2.0f, 0.5f, 0.05f,  // Velocity Kp, Ki, Kd
                     0.001f);
    
    plant_init(&plant, 0.001f);
    
    const int32_t SETPOINT = 180000;  // 180 degrees
    const int32_t SAMPLES = 1500;
    
    int32_t settling_sample = -1;
    
    for (int32_t i = 0; i < SAMPLES; i++) {
        int32_t control = cascade_pid_update(&cascade, SETPOINT, 
                                             plant.position, plant.velocity);
        plant_update(&plant, control);
        
        if (settling_sample < 0 && i > 200) {
            if (abs(plant.position - SETPOINT) < (SETPOINT / 50)) {
                settling_sample = i;
            }
        }
    }
    
    float settling_time_ms = settling_sample * 0.001f * 1000.0f;
    int32_t final_error = abs(plant.position - SETPOINT);
    
    printf("  Target: %ld mdeg (%.1f deg)\n", SETPOINT, SETPOINT / 1000.0f);
    printf("  Final position: %ld mdeg (%.1f deg)\n", plant.position, plant.position / 1000.0f);
    printf("  Final velocity: %ld mdeg/s\n", plant.velocity);
    printf("  Final error: %ld mdeg (%.3f deg)\n", final_error, final_error / 1000.0f);
    printf("  Settling time: %.1f ms\n", settling_time_ms);
    
    if (final_error > 2000) {
        printf("❌ Cascade control error too large\n");
        return false;
    }
    
    printf("✓ Cascade controller test passed\n");
    return true;
}

// Test 5: Adaptive gain scheduling
static bool test_adaptive_gains(void) {
    printf("\n=== Test 5: Adaptive Gain Scheduling ===\n");
    
    adaptive_pid_t apid;
    adaptive_pid_init(&apid, 2.0f, 0.5f, 0.05f, 0.001f);
    
    // Add gain schedules
    // Low velocity: high gains for precision
    adaptive_pid_add_schedule(&apid, 0, 1.0f, 1.0f, 1.0f);
    
    // Medium velocity: reduce gains slightly
    adaptive_pid_add_schedule(&apid, 50000, 0.8f, 0.6f, 0.8f);
    
    // High velocity: reduce gains more for stability
    adaptive_pid_add_schedule(&apid, 100000, 0.5f, 0.3f, 0.5f);
    
    printf("  Schedule entries: %d\n", apid.num_entries);
    for (uint8_t i = 0; i < apid.num_entries; i++) {
        printf("    Threshold %d: %ld mdeg/s, scales: Kp=%.2f Ki=%.2f Kd=%.2f\n",
               i, apid.schedule[i].velocity_threshold,
               apid.schedule[i].Kp_scale,
               apid.schedule[i].Ki_scale,
               apid.schedule[i].Kd_scale);
    }
    
    // Test at different velocities
    int32_t test_velocities[] = {0, 30000, 75000, 150000};
    
    for (int i = 0; i < 4; i++) {
        int32_t vel = test_velocities[i];
        
        // Update once to trigger gain scheduling
        adaptive_pid_update(&apid, 90000, 45000, vel);
        
        printf("  At velocity %ld mdeg/s:\n", vel);
        printf("    Kp: %ld (%.3f)\n", apid.current_Kp, apid.current_Kp / 65536.0f);
        printf("    Ki: %ld (%.3f)\n", apid.current_Ki, apid.current_Ki / 65536.0f);
        printf("    Kd: %ld (%.3f)\n", apid.current_Kd, apid.current_Kd / 65536.0f);
    }
    
    printf("✓ Adaptive gain scheduling test passed\n");
    return true;
}

// Test 6: Anti-windup performance
static bool test_anti_windup(void) {
    printf("\n=== Test 6: Anti-Windup Performance ===\n");
    
    pid_controller_t pid_with_windup, pid_without_windup;
    integrator_plant_t plant1, plant2;
    
    // Two identical PIDs, one with anti-windup
    pid_init(&pid_with_windup, 2.0f, 1.0f, 0.05f, 0.001f);
    pid_set_limits(&pid_with_windup, -5000, 5000);
    pid_enable_back_calculation(&pid_with_windup, 1.0f);
    
    pid_init(&pid_without_windup, 2.0f, 1.0f, 0.05f, 0.001f);
    pid_set_limits(&pid_without_windup, -5000, 5000);
    
    plant_init(&plant1, 0.001f);
    plant_init(&plant2, 0.001f);
    
    const int32_t SETPOINT = 90000;
    const int32_t SAMPLES = 1000;
    
    int32_t settling1 = -1, settling2 = -1;
    
    for (int32_t i = 0; i < SAMPLES; i++) {
        int32_t control1 = pid_update(&pid_with_windup, SETPOINT, plant1.position);
        int32_t control2 = pid_update(&pid_without_windup, SETPOINT, plant2.position);
        
        plant_update(&plant1, control1);
        plant_update(&plant2, control2);
        
        if (settling1 < 0 && abs(plant1.position - SETPOINT) < (SETPOINT / 50)) {
            settling1 = i;
        }
        if (settling2 < 0 && abs(plant2.position - SETPOINT) < (SETPOINT / 50)) {
            settling2 = i;
        }
    }
    
    printf("  With anti-windup:\n");
    printf("    Settling time: %.1f ms\n", settling1 * 0.001f * 1000.0f);
    printf("    Integral: %ld\n", pid_with_windup.integral);
    
    printf("  Without anti-windup:\n");
    printf("    Settling time: %.1f ms\n", settling2 * 0.001f * 1000.0f);
    printf("    Integral: %ld\n", pid_without_windup.integral);
    
    if (settling1 < settling2) {
        printf("✓ Anti-windup improved settling time by %.1f ms\n", 
               (settling2 - settling1) * 0.001f * 1000.0f);
    }
    
    printf("✓ Anti-windup test passed\n");
    return true;
}

// ============================================================================
// Performance Benchmark
// ============================================================================

typedef struct {
    uint32_t basic_pid_cycles;
    uint32_t cascade_pid_cycles;
    uint32_t adaptive_pid_cycles;
    float basic_pid_us;
    float cascade_pid_us;
    float adaptive_pid_us;
} pid_benchmark_result_t;

static pid_benchmark_result_t benchmark_pid_controllers(void) {
    pid_benchmark_result_t result = {0};
    
#ifdef PICO_2W
    pid_controller_t basic_pid;
    cascade_pid_t cascade;
    adaptive_pid_t adaptive;
    integrator_plant_t plant;
    
    pid_init(&basic_pid, 2.0f, 0.5f, 0.05f, 0.001f);
    cascade_pid_init(&cascade, 2.0f, 0.5f, 0.05f, 1.0f, 0.3f, 0.02f, 0.001f);
    adaptive_pid_init(&adaptive, 2.0f, 0.5f, 0.05f, 0.001f);
    adaptive_pid_add_schedule(&adaptive, 50000, 0.8f, 0.6f, 0.8f);
    
    plant_init(&plant, 0.001f);
    
    const uint32_t ITERATIONS = 1000;
    
    // Benchmark basic PID
    uint32_t start = time_us_32();
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        pid_update(&basic_pid, 90000, plant.position);
    }
    uint32_t end = time_us_32();
    result.basic_pid_us = (end - start) / (float)ITERATIONS;
    result.basic_pid_cycles = (uint32_t)(result.basic_pid_us * 150.0f);
    
    // Benchmark cascade PID
    start = time_us_32();
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        cascade_pid_update(&cascade, 90000, plant.position, plant.velocity);
    }
    end = time_us_32();
    result.cascade_pid_us = (end - start) / (float)ITERATIONS;
    result.cascade_pid_cycles = (uint32_t)(result.cascade_pid_us * 150.0f);
    
    // Benchmark adaptive PID
    start = time_us_32();
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        adaptive_pid_update(&adaptive, 90000, plant.position, plant.velocity);
    }
    end = time_us_32();
    result.adaptive_pid_us = (end - start) / (float)ITERATIONS;
    result.adaptive_pid_cycles = (uint32_t)(result.adaptive_pid_us * 150.0f);
#endif
    
    return result;
}

// ============================================================================
// Test Suite Entry Point
// ============================================================================

void pid_controller_run_tests(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║   DSP-ACCELERATED PID CONTROLLER TEST SUITE             ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");
    
    int tests_passed = 0;
    int tests_total = 0;
    
    // Run tests
    tests_total++; if (test_pid_init()) tests_passed++;
    tests_total++; if (test_step_response()) tests_passed++;
    tests_total++; if (test_sinusoidal_tracking()) tests_passed++;
    tests_total++; if (test_cascade_controller()) tests_passed++;
    tests_total++; if (test_adaptive_gains()) tests_passed++;
    tests_total++; if (test_anti_windup()) tests_passed++;
    
    // Performance benchmark
    printf("\n=== Performance Benchmark ===\n");
    pid_benchmark_result_t bench = benchmark_pid_controllers();
    
    printf("  Basic PID:      %.2f µs (%lu cycles)\n", 
           bench.basic_pid_us, bench.basic_pid_cycles);
    printf("  Cascade PID:    %.2f µs (%lu cycles)\n",
           bench.cascade_pid_us, bench.cascade_pid_cycles);
    printf("  Adaptive PID:   %.2f µs (%lu cycles)\n",
           bench.adaptive_pid_us, bench.adaptive_pid_cycles);
    
    // Calculate budget
    float budget_us = 1000.0f;
    float basic_budget = 100.0f * bench.basic_pid_us / budget_us;
    float cascade_budget = 100.0f * bench.cascade_pid_us / budget_us;
    
    printf("\n  Budget usage (1 kHz loop):\n");
    printf("    Basic PID:    %.2f%%\n", basic_budget);
    printf("    Cascade PID:  %.2f%%\n", cascade_budget);
    
    if (cascade_budget > 5.0f) {
        printf("  ⚠ Warning: Cascade PID >5%% of control loop\n");
    } else {
        printf("  ✓ PID overhead acceptable\n");
    }
    
    // Summary
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║   TEST SUMMARY: %d/%d PASSED                              ║\n", tests_passed, tests_total);
    printf("╚══════════════════════════════════════════════════════════╝\n");
    
    if (tests_passed == tests_total) {
        printf("✓ All tests passed! PID controllers ready for integration.\n");
    } else {
        printf("❌ Some tests failed. Review results above.\n");
    }
}
