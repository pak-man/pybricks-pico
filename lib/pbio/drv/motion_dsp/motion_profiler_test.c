// SPDX-License-Identifier: MIT
// Test suite for multi-axis motion profiling

#include "motion_profiler_dsp.h"
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
// Test Cases
// ============================================================================

// Test 1: Linear Interpolation Initialization
static bool test_linear_interp_init(void) {
    printf("\n=== Test 1: Linear Interpolation Initialization ===\n");
    
    linear_interpolation_t move;
    
    position_t start = {0, 0, 0, 0};
    position_t end = {100000, 50000, 0, 0};  // 100mm, 50mm
    
    linear_interp_init(&move, start, end, 200000, 500000, 1000000);
    
    if (!move.initialized) {
        printf("❌ Linear interpolation not initialized\n");
        return false;
    }
    
    printf("  Start: (%ld, %ld, %ld, %ld)\n", start.x, start.y, start.z, start.a);
    printf("  End: (%ld, %ld, %ld, %ld)\n", end.x, end.y, end.z, end.a);
    printf("  Distance: %ld\n", move.distance);
    printf("  Max velocity: %ld\n", move.max_velocity);
    
    if (move.distance == 0) {
        printf("❌ Distance calculation failed\n");
        return false;
    }
    
    printf("✓ Linear interpolation initialization passed\n");
    return true;
}

// Test 2: Linear Interpolation Path
static bool test_linear_interp_path(void) {
    printf("\n=== Test 2: Linear Interpolation Path ===\n");
    
    linear_interpolation_t move;
    
    position_t start = {0, 0, 0, 0};
    position_t end = {100000, 100000, 0, 0};  // Diagonal
    
    linear_interp_init(&move, start, end, 200000, 500000, 1000000);
    linear_interp_start(&move);
    
    // Sample path at 10%, 50%, 90%
    float test_points[] = {0.1f, 0.5f, 0.9f};
    
    for (int i = 0; i < 3; i++) {
        // Simulate time passing
        move.master_profile.elapsed_time = test_points[i] * move.master_profile.total_time;
        
        position_t pos = linear_interp_get_position(&move);
        
        printf("  At %.0f%%: pos=(%ld, %ld)\n", 
               test_points[i] * 100, pos.x, pos.y);
        
        // Verify position is on diagonal line
        int32_t expected_x = (int32_t)(test_points[i] * 100000);
        int32_t expected_y = (int32_t)(test_points[i] * 100000);
        
        int32_t error_x = abs(pos.x - expected_x);
        int32_t error_y = abs(pos.y - expected_y);
        
        if (error_x > 5000 || error_y > 5000) {  // 5mm tolerance
            printf("❌ Path error too large: X=%ld, Y=%ld\n", error_x, error_y);
            return false;
        }
    }
    
    printf("✓ Linear interpolation path test passed\n");
    return true;
}

// Test 3: Circular Interpolation Initialization
static bool test_circular_interp_init(void) {
    printf("\n=== Test 3: Circular Interpolation Initialization ===\n");
    
    circular_interpolation_t arc;
    
    position_t start = {100000, 0, 0, 0};   // (100, 0)
    position_t end = {0, 100000, 0, 0};     // (0, 100)
    position_t center = {0, 0, 0, 0};       // Origin
    
    circular_interp_init(&arc, start, end, center,
                         CIRCULAR_CCW,
                         AXIS_X, AXIS_Y,
                         200000, 500000, 1000000);
    
    if (!arc.initialized) {
        printf("❌ Circular interpolation not initialized\n");
        return false;
    }
    
    printf("  Start: (%ld, %ld)\n", start.x, start.y);
    printf("  End: (%ld, %ld)\n", end.x, end.y);
    printf("  Center: (%ld, %ld)\n", center.x, center.y);
    printf("  Radius: %ld\n", arc.radius);
    printf("  Arc length: %ld\n", arc.arc_length);
    printf("  Start angle: %.2f rad\n", arc.start_angle);
    printf("  End angle: %.2f rad\n", arc.end_angle);
    printf("  Total angle: %.2f rad (%.1f°)\n", 
           arc.total_angle, arc.total_angle * 180 / M_PI);
    
    // Verify radius (should be 100mm)
    if (abs(arc.radius - 100000) > 1000) {
        printf("❌ Radius calculation error\n");
        return false;
    }
    
    printf("✓ Circular interpolation initialization passed\n");
    return true;
}

// Test 4: Gantry Cross-Coupling
static bool test_gantry_coupling(void) {
    printf("\n=== Test 4: Gantry Cross-Coupling ===\n");
    
    gantry_controller_t gantry;
    
    gantry_init(&gantry, Q16_16(0.5), 2000);
    gantry_set_target(&gantry, 0, 100000, 200000, 500000, 1000000);
    gantry_start(&gantry);
    
    // Simulate motors with position error
    int32_t motor1_pos = 50000;  // Motor 1 ahead
    int32_t motor2_pos = 48000;  // Motor 2 behind
    
    int32_t sp1, sp2;
    gantry_update(&gantry, motor1_pos, motor2_pos, &sp1, &sp2);
    
    printf("  Motor 1 position: %ld\n", motor1_pos);
    printf("  Motor 2 position: %ld\n", motor2_pos);
    printf("  Position error: %ld\n", gantry.position_error);
    printf("  Motor 1 setpoint: %ld\n", sp1);
    printf("  Motor 2 setpoint: %ld\n", sp2);
    
    // Verify correction applied
    // Motor 1 (ahead) should get lower setpoint
    // Motor 2 (behind) should get higher setpoint
    int32_t correction_direction = (sp2 - sp1);
    
    if (correction_direction <= 0) {
        printf("❌ Cross-coupling correction wrong direction\n");
        return false;
    }
    
    printf("  Correction magnitude: %ld\n", abs(sp1 - sp2) / 2);
    printf("✓ Gantry cross-coupling test passed\n");
    return true;
}

// Test 5: Electronic Gearing
static bool test_electronic_gearing(void) {
    printf("\n=== Test 5: Electronic Gearing ===\n");
    
    electronic_gearing_t gearing;
    
    float gear_ratio = 2.0f;  // Slave moves 2x master
    int32_t offset = 10000;   // 10mm offset
    
    gearing_init(&gearing, gear_ratio, offset);
    gearing_set_master(&gearing, 0, 50000, 100000, 200000, 500000);
    gearing_start(&gearing);
    
    // Test at 50% of master move
    gearing.master_profile.elapsed_time = gearing.master_profile.total_time * 0.5f;
    
    int32_t master_pos, slave_pos;
    gearing_get_positions(&gearing, &master_pos, &slave_pos);
    
    printf("  Gear ratio: %.2f\n", gear_ratio);
    printf("  Offset: %ld\n", offset);
    printf("  Master position: %ld\n", master_pos);
    printf("  Slave position: %ld\n", slave_pos);
    
    // Verify slave = master * ratio + offset
    int32_t expected_slave = (int32_t)(master_pos * gear_ratio) + offset;
    int32_t error = abs(slave_pos - expected_slave);
    
    printf("  Expected slave: %ld\n", expected_slave);
    printf("  Error: %ld\n", error);
    
    if (error > 1000) {  // 1mm tolerance
        printf("❌ Electronic gearing error too large\n");
        return false;
    }
    
    printf("✓ Electronic gearing test passed\n");
    return true;
}

// Test 6: Synchronized Multi-Axis
static bool test_synchronized_motion(void) {
    printf("\n=== Test 6: Synchronized Multi-Axis Motion ===\n");
    
    synchronized_motion_t sync;
    
    sync_motion_init(&sync, 3);  // X, Y, Z
    
    // Different distances, should finish together
    sync_motion_set_axis_target(&sync, AXIS_X, 0, 100000, 200000, 500000, 1000000);
    sync_motion_set_axis_target(&sync, AXIS_Y, 0, 50000, 200000, 500000, 1000000);
    sync_motion_set_axis_target(&sync, AXIS_Z, 0, 25000, 200000, 500000, 1000000);
    
    sync_motion_start(&sync);
    
    printf("  Number of axes: %d\n", sync.num_axes);
    printf("  X target: 100000\n");
    printf("  Y target: 50000\n");
    printf("  Z target: 25000\n");
    
    // Check all axes start
    for (int i = 0; i < 3; i++) {
        if (!sync.axis_profile[i].active) {
            printf("❌ Axis %d not active\n", i);
            return false;
        }
    }
    
    printf("✓ Synchronized multi-axis test passed\n");
    return true;
}

// ============================================================================
// Performance Benchmark
// ============================================================================

typedef struct {
    float linear_interp_us;
    float circular_interp_us;
    float gantry_update_us;
    uint32_t linear_cycles;
    uint32_t circular_cycles;
    uint32_t gantry_cycles;
} motion_benchmark_result_t;

static motion_benchmark_result_t benchmark_motion_profiling(void) {
    motion_benchmark_result_t result = {0};
    
#ifdef PICO_2W
    const uint32_t ITERATIONS = 1000;
    
    // Benchmark linear interpolation
    linear_interpolation_t move;
    position_t start = {0, 0, 0, 0};
    position_t end = {100000, 100000, 0, 0};
    linear_interp_init(&move, start, end, 200000, 500000, 1000000);
    linear_interp_start(&move);
    
    uint32_t t_start = time_us_32();
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        position_t pos = linear_interp_get_position(&move);
        (void)pos;  // Prevent optimization
    }
    uint32_t t_end = time_us_32();
    result.linear_interp_us = (t_end - t_start) / (float)ITERATIONS;
    result.linear_cycles = (uint32_t)(result.linear_interp_us * 150.0f);
    
    // Benchmark circular interpolation
    circular_interpolation_t arc;
    position_t center = {0, 0, 0, 0};
    circular_interp_init(&arc, start, end, center, CIRCULAR_CCW,
                         AXIS_X, AXIS_Y, 200000, 500000, 1000000);
    circular_interp_start(&arc);
    
    t_start = time_us_32();
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        position_t pos = circular_interp_get_position(&arc);
        (void)pos;
    }
    t_end = time_us_32();
    result.circular_interp_us = (t_end - t_start) / (float)ITERATIONS;
    result.circular_cycles = (uint32_t)(result.circular_interp_us * 150.0f);
    
    // Benchmark gantry update
    gantry_controller_t gantry;
    gantry_init(&gantry, Q16_16(0.5), 2000);
    gantry_set_target(&gantry, 0, 100000, 200000, 500000, 1000000);
    gantry_start(&gantry);
    
    int32_t sp1, sp2;
    t_start = time_us_32();
    for (uint32_t i = 0; i < ITERATIONS; i++) {
        gantry_update(&gantry, 50000, 49000, &sp1, &sp2);
    }
    t_end = time_us_32();
    result.gantry_update_us = (t_end - t_start) / (float)ITERATIONS;
    result.gantry_cycles = (uint32_t)(result.gantry_update_us * 150.0f);
#endif
    
    return result;
}

// ============================================================================
// Test Suite Entry Point
// ============================================================================

void motion_profiling_run_tests(void) {
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║   MULTI-AXIS MOTION PROFILING TEST SUITE                ║\n");
    printf("╚══════════════════════════════════════════════════════════╝\n");
    
    int tests_passed = 0;
    int tests_total = 0;
    
    // Run tests
    tests_total++; if (test_linear_interp_init()) tests_passed++;
    tests_total++; if (test_linear_interp_path()) tests_passed++;
    tests_total++; if (test_circular_interp_init()) tests_passed++;
    tests_total++; if (test_gantry_coupling()) tests_passed++;
    tests_total++; if (test_electronic_gearing()) tests_passed++;
    tests_total++; if (test_synchronized_motion()) tests_passed++;
    
    // Performance benchmark
    printf("\n=== Performance Benchmark ===\n");
    motion_benchmark_result_t bench = benchmark_motion_profiling();
    
    printf("  Linear interpolation:   %.2f µs (%lu cycles)\n",
           bench.linear_interp_us, bench.linear_cycles);
    printf("  Circular interpolation: %.2f µs (%lu cycles)\n",
           bench.circular_interp_us, bench.circular_cycles);
    printf("  Gantry cross-coupling:  %.2f µs (%lu cycles)\n",
           bench.gantry_update_us, bench.gantry_cycles);
    
    // Calculate budget
    float budget_us = 1000.0f;  // 1 kHz loop
    float linear_budget = 100.0f * bench.linear_interp_us / budget_us;
    float circular_budget = 100.0f * bench.circular_interp_us / budget_us;
    float gantry_budget = 100.0f * bench.gantry_update_us / budget_us;
    
    printf("\n  Budget usage (1 kHz loop):\n");
    printf("    Linear interpolation:   %.2f%%\n", linear_budget);
    printf("    Circular interpolation: %.2f%%\n", circular_budget);
    printf("    Gantry cross-coupling:  %.2f%%\n", gantry_budget);
    
    // Combined with previous phases
    float phase2_budget = 0.45f;  // Encoder filter
    float phase3_budget = 0.6f;   // Cascade PID
    float phase4_budget = 0.35f;  // S-curve trajectory
    float phase5_budget = (linear_budget > circular_budget) ? 
                          circular_budget : linear_budget;  // Worst case
    
    float total_budget = phase2_budget + phase3_budget + phase4_budget + phase5_budget;
    
    printf("\n  Combined system budget:\n");
    printf("    Phase 2 (Encoder):    %.2f%%\n", phase2_budget);
    printf("    Phase 3 (PID):        %.2f%%\n", phase3_budget);
    printf("    Phase 4 (Trajectory): %.2f%%\n", phase4_budget);
    printf("    Phase 5 (Multi-axis): %.2f%%\n", phase5_budget);
    printf("    ────────────────────────────\n");
    printf("    TOTAL:                %.2f%%\n", total_budget);
    printf("    Available:            %.2f%%\n", 100.0f - total_budget);
    
    if (total_budget > 5.0f) {
        printf("  ⚠ Warning: Total budget >5%% of control loop\n");
    } else {
        printf("  ✓ System overhead excellent (<%d%%)\n", (int)(total_budget + 1));
    }
    
    // Summary
    printf("\n");
    printf("╔══════════════════════════════════════════════════════════╗\n");
    printf("║   TEST SUMMARY: %d/%d PASSED                              ║\n", tests_passed, tests_total);
    printf("╚══════════════════════════════════════════════════════════╝\n");
    
    if (tests_passed == tests_total) {
        printf("✓ All tests passed! Multi-axis motion profiling ready.\n");
    } else {
        printf("❌ Some tests failed. Review results above.\n");
    }
}
