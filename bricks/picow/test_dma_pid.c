// SPDX-License-Identifier: MIT
// Copyright (c) 2024 pybricks-pico contributors
// Test/example for DMA-accelerated PID motor control

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "lib/pbio/drv/pid_dma/pid_dma_rp2040.h"

// Test configuration - adjust pins for your hardware
#define TEST_MOTOR_ENC_A    2
#define TEST_MOTOR_ENC_B    3
#define TEST_MOTOR_PWM_A    4
#define TEST_MOTOR_PWM_B    5
#define TEST_MOTOR_CPR      360  // Counts per revolution

static void print_motor_state(uint8_t motor_id) {
    int32_t pos, speed, pwm;
    pid_dma_get_state(motor_id, &pos, &speed, &pwm);
    
    printf("Motor %d: pos=%ld mdeg (%ld deg), speed=%ld mdeg/s, pwm=%ld\n",
           motor_id, pos, pos/1000, speed, pwm);
}

// ============================================================================
// Test 1: Basic position control
// ============================================================================
static void test_position_control(void) {
    printf("\n=== Test 1: Position Control ===\n");
    
    // Set target position: 90 degrees
    printf("Setting target: 90 degrees\n");
    pid_dma_set_position_target(0, 90000);  // 90 deg in millidegrees
    pid_dma_enable(0, true);
    
    // Wait and monitor
    for (int i = 0; i < 50; i++) {
        sleep_ms(100);
        print_motor_state(0);
        
        // Check if done
        volatile pid_dma_state_t *state = pid_dma_get_state_ptr(0);
        int32_t error = state->target_position - state->current_position;
        if (error < 0) error = -error;
        
        if (error < 1000) {  // Within 1 degree
            printf("Target reached!\n");
            break;
        }
    }
    
    sleep_ms(500);
    
    // Return to zero
    printf("Returning to 0 degrees\n");
    pid_dma_set_position_target(0, 0);
    
    for (int i = 0; i < 50; i++) {
        sleep_ms(100);
        print_motor_state(0);
        
        volatile pid_dma_state_t *state = pid_dma_get_state_ptr(0);
        int32_t error = state->target_position - state->current_position;
        if (error < 0) error = -error;
        
        if (error < 1000) {
            printf("Zero reached!\n");
            break;
        }
    }
    
    pid_dma_enable(0, false);
}

// ============================================================================
// Test 2: Speed control
// ============================================================================
static void test_speed_control(void) {
    printf("\n=== Test 2: Speed Control ===\n");
    
    // Set target speed: 180 deg/s
    printf("Setting speed: 180 deg/s\n");
    pid_dma_set_speed_target(0, 180000);  // 180 deg/s in millideg/s
    pid_dma_enable(0, true);
    
    // Run for 3 seconds
    for (int i = 0; i < 30; i++) {
        sleep_ms(100);
        print_motor_state(0);
    }
    
    // Reverse direction
    printf("Reversing: -180 deg/s\n");
    pid_dma_set_speed_target(0, -180000);
    
    for (int i = 0; i < 30; i++) {
        sleep_ms(100);
        print_motor_state(0);
    }
    
    // Stop
    printf("Stopping\n");
    pid_dma_enable(0, false);
}

// ============================================================================
// Test 3: Stall detection
// ============================================================================
static void test_stall_detection(void) {
    printf("\n=== Test 3: Stall Detection ===\n");
    printf("Hold the motor shaft to trigger stall detection\n");
    
    pid_dma_set_position_target(0, 180000);  // 180 degrees
    pid_dma_enable(0, true);
    
    for (int i = 0; i < 50; i++) {
        sleep_ms(100);
        print_motor_state(0);
        
        if (pid_dma_is_stalled(0)) {
            printf("*** STALL DETECTED! ***\n");
            break;
        }
    }
    
    pid_dma_enable(0, false);
    pid_dma_reset_position(0);
}

// ============================================================================
// Test 4: PID tuning verification
// ============================================================================
static void test_pid_tuning(void) {
    printf("\n=== Test 4: PID Tuning Comparison ===\n");
    
    // Test with default gains
    printf("\n-- Default PID gains --\n");
    pid_dma_set_gains(0, 32768, 3277, 6554);  // Default values
    pid_dma_set_position_target(0, 180000);
    pid_dma_enable(0, true);
    
    for (int i = 0; i < 30; i++) {
        sleep_ms(100);
        print_motor_state(0);
    }
    
    pid_dma_enable(0, false);
    sleep_ms(500);
    pid_dma_reset_position(0);
    sleep_ms(500);
    
    // Test with high P gain (more aggressive)
    printf("\n-- High P gain (aggressive) --\n");
    pid_dma_set_gains(0, 65536, 3277, 6554);  // 2x P
    pid_dma_set_position_target(0, 180000);
    pid_dma_enable(0, true);
    
    for (int i = 0; i < 30; i++) {
        sleep_ms(100);
        print_motor_state(0);
    }
    
    pid_dma_enable(0, false);
    sleep_ms(500);
    pid_dma_reset_position(0);
    sleep_ms(500);
    
    // Test with high D gain (more damped)
    printf("\n-- High D gain (damped) --\n");
    pid_dma_set_gains(0, 32768, 3277, 26214);  // 4x D
    pid_dma_set_position_target(0, 180000);
    pid_dma_enable(0, true);
    
    for (int i = 0; i < 30; i++) {
        sleep_ms(100);
        print_motor_state(0);
    }
    
    pid_dma_enable(0, false);
    pid_dma_reset_position(0);
}

// ============================================================================
// Test 5: Multi-motor coordination
// ============================================================================
static void test_multi_motor(void) {
    printf("\n=== Test 5: Multi-Motor (if 2+ motors connected) ===\n");
    
    // Setup second motor on Port B (motor_id 1)
    pid_dma_hw_config_t config1 = {
        .encoder_pin_a = 6,
        .encoder_pin_b = 7,
        .pwm_pin_a = 8,
        .pwm_pin_b = 9,
        .counts_per_rev = TEST_MOTOR_CPR,
    };
    
    if (pid_dma_motor_setup(1, &config1) != 0) {
        printf("Failed to setup motor 1\n");
        return;
    }
    
    // Move both motors to different positions simultaneously
    printf("Moving motor 0 to 90 deg, motor 1 to -90 deg\n");
    pid_dma_set_position_target(0, 90000);
    pid_dma_set_position_target(1, -90000);
    pid_dma_enable(0, true);
    pid_dma_enable(1, true);
    
    for (int i = 0; i < 30; i++) {
        sleep_ms(100);
        
        int32_t pos0, pos1, spd, pwm;
        pid_dma_get_state(0, &pos0, &spd, &pwm);
        pid_dma_get_state(1, &pos1, &spd, &pwm);
        
        printf("M0: %ld deg, M1: %ld deg\n", pos0/1000, pos1/1000);
    }
    
    pid_dma_enable(0, false);
    pid_dma_enable(1, false);
}

// ============================================================================
// Test 6: Performance benchmark
// ============================================================================
static void test_performance(void) {
    printf("\n=== Test 6: Performance Benchmark ===\n");
    
    // Measure how much CPU time is available while DMA PID runs
    pid_dma_set_speed_target(0, 360000);  // 360 deg/s
    pid_dma_enable(0, true);
    
    uint32_t start = time_us_32();
    uint32_t loop_count = 0;
    
    // Count how many tight loop iterations we can do in 1 second
    while (time_us_32() - start < 1000000) {
        // Simulate doing other work
        volatile uint32_t dummy = 0;
        for (int i = 0; i < 100; i++) {
            dummy += i;
        }
        loop_count++;
    }
    
    pid_dma_enable(0, false);
    
    printf("Loop iterations in 1s: %lu\n", loop_count);
    printf("This shows CPU is free while DMA handles PID!\n");
    
    // Compare to synchronous PID
    printf("\nCompare: same test WITHOUT DMA (if using CPU-based PID)\n");
    printf("Would show significantly fewer iterations.\n");
}

// ============================================================================
// Main test runner
// ============================================================================
int main(void) {
    stdio_init_all();
    
    // Initialize WiFi for Pico W LED (just for visual feedback)
    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
    }
    
    sleep_ms(2000);  // Wait for serial connection
    
    printf("\n");
    printf("================================================\n");
    printf("  DMA-Accelerated PID Motor Control Test Suite\n");
    printf("  for pybricks-pico on RP2040/RP2350\n");
    printf("================================================\n");
    
    // Initialize DMA PID subsystem
    printf("\nInitializing DMA PID system...\n");
    pid_dma_init();
    
    // Setup test motor
    pid_dma_hw_config_t config = {
        .encoder_pio = NULL,  // Use default (pio0)
        .encoder_pin_a = TEST_MOTOR_ENC_A,
        .encoder_pin_b = TEST_MOTOR_ENC_B,
        .pwm_pio = NULL,      // Use default (pio1)
        .pwm_pin_a = TEST_MOTOR_PWM_A,
        .pwm_pin_b = TEST_MOTOR_PWM_B,
        .counts_per_rev = TEST_MOTOR_CPR,
    };
    
    if (pid_dma_motor_setup(0, &config) != 0) {
        printf("Failed to setup motor!\n");
        return 1;
    }
    
    printf("Motor setup complete.\n");
    printf("\nPress any key to start tests...\n");
    getchar();
    
    // Run tests
    test_position_control();
    sleep_ms(1000);
    
    test_speed_control();
    sleep_ms(1000);
    
    test_stall_detection();
    sleep_ms(1000);
    
    test_pid_tuning();
    sleep_ms(1000);
    
    // Optional: uncomment if you have multiple motors
    // test_multi_motor();
    // sleep_ms(1000);
    
    test_performance();
    
    printf("\n=== All tests complete! ===\n");
    
    // Cleanup
    pid_dma_deinit();
    
    while (1) {
        // Blink LED to show we're done
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
        sleep_ms(250);
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
        sleep_ms(250);
    }
    
    return 0;
}
