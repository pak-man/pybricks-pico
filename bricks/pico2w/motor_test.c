// SPDX-License-Identifier: MIT
// Advanced motor test functions for 12-motor system

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "pico/stdlib.h"

// External motor control API
extern void motor_stop(uint8_t motor_id);
extern void motor_run_duty(uint8_t motor_id, int16_t duty);
extern void motor_run_target(uint8_t motor_id, int32_t target_position);
extern void motor_run_velocity(uint8_t motor_id, int32_t target_velocity);
extern bool motor_at_target(uint8_t motor_id);
extern int32_t pbdrv_counter_get_count_simple(uint8_t id);
extern int32_t pbdrv_counter_get_rate_simple(uint8_t id);
extern void pbdrv_counter_reset(uint8_t id);

#define NUM_MOTORS 12

// Test: Sequential motor activation (wave pattern)
void motor_test_wave(void) {
    printf("\n=== Wave Test ===\n");
    printf("Motors activate sequentially in a wave\n");
    
    for (int cycle = 0; cycle < 3; cycle++) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            motor_run_duty(i, 5000);  // 50% duty
            sleep_ms(100);
            motor_stop(i);
        }
    }
    
    printf("Wave test complete\n");
}

// Test: All motors synchronized rotation
void motor_test_synchronized(void) {
    printf("\n=== Synchronized Rotation Test ===\n");
    
    // Reset all encoders
    for (int i = 0; i < NUM_MOTORS; i++) {
        pbdrv_counter_reset(i);
    }
    
    // Command all motors to same target
    int32_t target = 720;  // 2 rotations (assuming ~360 ticks/rev)
    printf("All motors → %ld ticks\n", target);
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_run_target(i, target);
    }
    
    // Monitor progress
    bool all_done = false;
    while (!all_done) {
        all_done = true;
        
        printf("Progress: ");
        for (int i = 0; i < NUM_MOTORS; i++) {
            int32_t pos = pbdrv_counter_get_count_simple(i);
            printf("M%d:%ld ", i, pos);
            
            if (!motor_at_target(i)) {
                all_done = false;
            }
        }
        printf("\n");
        
        sleep_ms(200);
    }
    
    printf("All motors reached target!\n");
}

// Test: Velocity control (all motors at different speeds)
void motor_test_velocity_ramp(void) {
    printf("\n=== Velocity Ramp Test ===\n");
    
    // Each motor gets progressively higher velocity
    for (int i = 0; i < NUM_MOTORS; i++) {
        int32_t velocity = 200 * (i + 1);  // 200, 400, 600, ... ticks/s
        motor_run_velocity(i, velocity);
        printf("M%d: %ld ticks/s\n", i, velocity);
    }
    
    // Run for 5 seconds
    printf("Running for 5 seconds...\n");
    for (int t = 0; t < 50; t++) {
        sleep_ms(100);
        
        if (t % 10 == 0) {
            printf("t=%ds: ", t/10);
            for (int i = 0; i < 4; i++) {  // Print first 4 motors
                printf("M%d:%ld ", i, pbdrv_counter_get_count_simple(i));
            }
            printf("...\n");
        }
    }
    
    // Stop all
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_stop(i);
    }
    
    printf("Velocity ramp complete\n");
}

// Test: Hexapod gait simulation (6 legs × 2 motors)
void motor_test_hexapod_gait(void) {
    printf("\n=== Hexapod Gait Simulation ===\n");
    printf("Simulating tripod gait (6 legs, 2 DOF each)\n");
    
    // Tripod gait: legs 0,2,4 move together, then 1,3,5
    const int leg_pairs[2][3] = {
        {0, 2, 4},  // First tripod (hip = M0,4,8, knee = M1,5,9)
        {1, 3, 5}   // Second tripod (hip = M2,6,10, knee = M3,7,11)
    };
    
    for (int cycle = 0; cycle < 5; cycle++) {
        for (int tripod = 0; tripod < 2; tripod++) {
            printf("Cycle %d, Tripod %d\n", cycle, tripod);
            
            // Lift and move forward
            for (int i = 0; i < 3; i++) {
                int leg = leg_pairs[tripod][i];
                int hip_motor = leg * 2;
                int knee_motor = leg * 2 + 1;
                
                // Lift (knee)
                motor_run_target(knee_motor, 90);
                // Move forward (hip)
                motor_run_target(hip_motor, 45);
            }
            
            sleep_ms(300);
            
            // Lower
            for (int i = 0; i < 3; i++) {
                int leg = leg_pairs[tripod][i];
                int knee_motor = leg * 2 + 1;
                motor_run_target(knee_motor, 0);
            }
            
            sleep_ms(300);
        }
    }
    
    // Return to neutral
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_run_target(i, 0);
    }
    
    printf("Hexapod gait complete\n");
}

// Test: Circle pattern (for robot arm or drawing robot)
void motor_test_circle_pattern(void) {
    printf("\n=== Circle Pattern Test ===\n");
    printf("Motors 0-1 draw circle (2-DOF arm)\n");
    
    const int steps = 36;  // 36 points around circle
    const float radius = 100.0f;  // Encoder ticks
    
    for (int step = 0; step < steps; step++) {
        float angle = (2.0f * M_PI * step) / steps;
        
        int32_t x = (int32_t)(radius * cosf(angle));
        int32_t y = (int32_t)(radius * sinf(angle));
        
        motor_run_target(0, x);  // X axis
        motor_run_target(1, y);  // Y axis
        
        printf("Step %d: (%ld, %ld)\n", step, x, y);
        
        // Wait for motors to reach position
        while (!motor_at_target(0) || !motor_at_target(1)) {
            sleep_ms(10);
        }
        
        sleep_ms(100);
    }
    
    // Return to center
    motor_run_target(0, 0);
    motor_run_target(1, 0);
    
    printf("Circle pattern complete\n");
}

// Test: Load test (all motors max duty)
void motor_test_load(void) {
    printf("\n=== Load Test ===\n");
    printf("WARNING: This will run all 12 motors at 80%% duty!\n");
    printf("Ensure adequate power supply and motor cooling.\n");
    printf("Starting in 3 seconds...\n");
    sleep_ms(3000);
    
    // All motors forward at 80%
    printf("All motors → 80%% duty\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_run_duty(i, 8000);
    }
    
    // Monitor for 5 seconds
    for (int t = 0; t < 50; t++) {
        if (t % 10 == 0) {
            printf("t=%ds: ", t/10);
            for (int i = 0; i < 4; i++) {
                int32_t vel = pbdrv_counter_get_rate_simple(i);
                printf("M%d:%ld ", i, vel);
            }
            printf("ticks/s\n");
        }
        sleep_ms(100);
    }
    
    // Stop all
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_stop(i);
    }
    
    printf("Load test complete\n");
}

// Parse and execute test command
void motor_test_parse_command(const char *cmd) {
    if (strcmp(cmd, "WAVE") == 0) {
        motor_test_wave();
    } else if (strcmp(cmd, "SYNC") == 0) {
        motor_test_synchronized();
    } else if (strcmp(cmd, "VRAMP") == 0) {
        motor_test_velocity_ramp();
    } else if (strcmp(cmd, "HEXAPOD") == 0) {
        motor_test_hexapod_gait();
    } else if (strcmp(cmd, "CIRCLE") == 0) {
        motor_test_circle_pattern();
    } else if (strcmp(cmd, "LOAD") == 0) {
        motor_test_load();
    } else {
        printf("Unknown test: %s\n", cmd);
        printf("Available tests:\n");
        printf("  WAVE     - Sequential activation\n");
        printf("  SYNC     - Synchronized rotation\n");
        printf("  VRAMP    - Velocity ramp\n");
        printf("  HEXAPOD  - Hexapod gait simulation\n");
        printf("  CIRCLE   - Circle pattern (2-DOF)\n");
        printf("  LOAD     - Load test (all motors)\n");
    }
}
