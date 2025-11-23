// SPDX-License-Identifier: MIT
// Complete motor test suite for 4-motor system

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"

// External motor functions
extern void pbdrv_pwm_set_duty_simple(uint8_t id, int16_t duty);
extern int32_t pbdrv_counter_get_count_simple(uint8_t id);
extern int32_t pbdrv_counter_get_rate_simple(uint8_t id);
extern void pbdrv_counter_reset(uint8_t id);

#define NUM_MOTORS 4

// ========== Basic Tests ==========

// Test: All motors forward
void motor_test_all_forward(void) {
    printf("\n=== All Motors Forward Test ===\n");
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        pbdrv_counter_reset(i);
    }
    
    printf("Running all motors at 50%% for 2 seconds...\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        pbdrv_pwm_set_duty_simple(i, 5000);
    }
    
    for (int t = 0; t < 20; t++) {
        sleep_ms(100);
        if (t % 5 == 0) {
            printf("t=%d.%ds: ", t/10, t%10);
            for (int i = 0; i < NUM_MOTORS; i++) {
                printf("M%d=%ld ", i, pbdrv_counter_get_count_simple(i));
            }
            printf("\n");
        }
    }
    
    printf("Stopping...\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        pbdrv_pwm_set_duty_simple(i, 0);
    }
    
    printf("Final counts:\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        printf("  Motor %d: %ld ticks\n", i, pbdrv_counter_get_count_simple(i));
    }
}

// Test: Single motor forward/reverse
void motor_test_single(uint8_t motor_id) {
    printf("\n=== Single Motor Test (M%d) ===\n", motor_id);
    
    pbdrv_counter_reset(motor_id);
    
    printf("Forward 30%%...\n");
    pbdrv_pwm_set_duty_simple(motor_id, 3000);
    sleep_ms(1000);
    int32_t fwd = pbdrv_counter_get_count_simple(motor_id);
    printf("  Count after forward: %ld\n", fwd);
    
    pbdrv_pwm_set_duty_simple(motor_id, 0);
    sleep_ms(500);
    
    printf("Reverse 30%%...\n");
    pbdrv_pwm_set_duty_simple(motor_id, -3000);
    sleep_ms(1000);
    int32_t rev = pbdrv_counter_get_count_simple(motor_id);
    printf("  Count after reverse: %ld\n", rev);
    printf("  Net count: %ld (should be near 0)\n", rev);
    
    pbdrv_pwm_set_duty_simple(motor_id, 0);
}

// ========== Advanced Tests ==========

// Test: Speed ramp with acceleration profile
void motor_test_ramp(uint8_t motor_id) {
    printf("\n=== Speed Ramp Test (M%d) ===\n", motor_id);
    
    pbdrv_counter_reset(motor_id);
    
    printf("Ramping up (0%% to 80%%)...\n");
    for (int duty = 0; duty <= 8000; duty += 200) {
        pbdrv_pwm_set_duty_simple(motor_id, duty);
        sleep_ms(50);
    }
    
    sleep_ms(500);
    int32_t count_max = pbdrv_counter_get_count_simple(motor_id);
    printf("Count at max speed: %ld\n", count_max);
    
    printf("Ramping down (80%% to 0%%)...\n");
    for (int duty = 8000; duty >= 0; duty -= 200) {
        pbdrv_pwm_set_duty_simple(motor_id, duty);
        sleep_ms(50);
    }
    
    int32_t count_final = pbdrv_counter_get_count_simple(motor_id);
    printf("Final count: %ld\n", count_final);
}

// Test: Wave pattern (sequential activation)
void motor_test_wave(void) {
    printf("\n=== Wave Pattern Test ===\n");
    printf("Motors activate sequentially...\n");
    
    for (int cycle = 0; cycle < 3; cycle++) {
        printf("Cycle %d:\n", cycle + 1);
        for (int i = 0; i < NUM_MOTORS; i++) {
            printf("  M%d ON\n", i);
            pbdrv_pwm_set_duty_simple(i, 5000);
            sleep_ms(200);
            pbdrv_pwm_set_duty_simple(i, 0);
        }
    }
    printf("Wave complete\n");
}

// Test: Synchronized rotation
void motor_test_synchronized(void) {
    printf("\n=== Synchronized Rotation Test ===\n");
    printf("All motors run to same position\n");
    
    // Reset encoders
    for (int i = 0; i < NUM_MOTORS; i++) {
        pbdrv_counter_reset(i);
    }
    
    // Simple open-loop "synchronized" motion
    // (Note: Real sync needs PID, this is just timed)
    printf("Running all motors at 40%% for 2 seconds...\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        pbdrv_pwm_set_duty_simple(i, 4000);
    }
    
    // Monitor positions
    for (int t = 0; t < 20; t++) {
        sleep_ms(100);
        if (t % 5 == 0) {
            printf("t=%ds: ", t/10);
            for (int i = 0; i < NUM_MOTORS; i++) {
                printf("M%d:%ld ", i, pbdrv_counter_get_count_simple(i));
            }
            printf("\n");
        }
    }
    
    // Stop all
    for (int i = 0; i < NUM_MOTORS; i++) {
        pbdrv_pwm_set_duty_simple(i, 0);
    }
    
    printf("Final positions:\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        printf("  M%d: %ld ticks\n", i, pbdrv_counter_get_count_simple(i));
    }
}

// Test: Velocity measurement
void motor_test_velocity(void) {
    printf("\n=== Velocity Measurement Test ===\n");
    
    // Run motors at different speeds
    printf("M0: 30%%, M1: 50%%, M2: 70%%, M3: 90%%\n");
    pbdrv_pwm_set_duty_simple(0, 3000);
    pbdrv_pwm_set_duty_simple(1, 5000);
    pbdrv_pwm_set_duty_simple(2, 7000);
    pbdrv_pwm_set_duty_simple(3, 9000);
    
    // Measure for 3 seconds
    for (int t = 0; t < 30; t++) {
        sleep_ms(100);
        if (t % 10 == 0) {
            printf("Velocities (ticks/s):\n");
            for (int i = 0; i < NUM_MOTORS; i++) {
                int32_t vel = pbdrv_counter_get_rate_simple(i);
                printf("  M%d: %ld\n", i, vel);
            }
        }
    }
    
    // Stop all
    for (int i = 0; i < NUM_MOTORS; i++) {
        pbdrv_pwm_set_duty_simple(i, 0);
    }
}

// Test: Circle pattern (2-DOF robot arm with motors 0-1)
void motor_test_circle(void) {
    printf("\n=== Circle Pattern Test ===\n");
    printf("Motors 0-1 trace a circle (open-loop)\n");
    
    const int steps = 36;
    const float radius = 200.0f;  // Duty cycle units
    
    for (int step = 0; step < steps; step++) {
        float angle = (2.0f * M_PI * step) / steps;
        
        int16_t duty_x = (int16_t)(radius * cosf(angle));
        int16_t duty_y = (int16_t)(radius * sinf(angle));
        
        pbdrv_pwm_set_duty_simple(0, duty_x);
        pbdrv_pwm_set_duty_simple(1, duty_y);
        
        if (step % 6 == 0) {
            printf("Step %d: M0=%d, M1=%d\n", step, duty_x, duty_y);
        }
        
        sleep_ms(100);
    }
    
    // Return to center
    pbdrv_pwm_set_duty_simple(0, 0);
    pbdrv_pwm_set_duty_simple(1, 0);
    printf("Circle complete\n");
}

// Test: Load test (stress test)
void motor_test_load(void) {
    printf("\n=== Load Test ===\n");
    printf("WARNING: Running all motors at 80%% duty!\n");
    printf("Ensure adequate power supply.\n");
    printf("Starting in 3 seconds...\n");
    sleep_ms(3000);
    
    printf("All motors -> 80%% duty\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        pbdrv_pwm_set_duty_simple(i, 8000);
    }
    
    // Monitor for 5 seconds
    for (int t = 0; t < 50; t++) {
        sleep_ms(100);
        if (t % 10 == 0) {
            printf("t=%ds: ", t/10);
            for (int i = 0; i < NUM_MOTORS; i++) {
                int32_t vel = pbdrv_counter_get_rate_simple(i);
                printf("M%d:%ld ", i, vel);
            }
            printf("ticks/s\n");
        }
    }
    
    // Stop all
    for (int i = 0; i < NUM_MOTORS; i++) {
        pbdrv_pwm_set_duty_simple(i, 0);
    }
    printf("Load test complete\n");
}

// Test: Acceleration profile (trapezoidal)
void motor_test_trapezoid(uint8_t motor_id) {
    printf("\n=== Trapezoidal Profile Test (M%d) ===\n", motor_id);
    
    pbdrv_counter_reset(motor_id);
    
    // Phase 1: Acceleration
    printf("Phase 1: Accelerating...\n");
    for (int duty = 0; duty <= 6000; duty += 100) {
        pbdrv_pwm_set_duty_simple(motor_id, duty);
        sleep_ms(20);
    }
    
    // Phase 2: Constant velocity
    printf("Phase 2: Cruising at 60%%...\n");
    sleep_ms(1000);
    printf("  Position: %ld\n", pbdrv_counter_get_count_simple(motor_id));
    
    // Phase 3: Deceleration
    printf("Phase 3: Decelerating...\n");
    for (int duty = 6000; duty >= 0; duty -= 100) {
        pbdrv_pwm_set_duty_simple(motor_id, duty);
        sleep_ms(20);
    }
    
    int32_t final_pos = pbdrv_counter_get_count_simple(motor_id);
    printf("Final position: %ld ticks\n", final_pos);
}

// ========== Command Parser ==========

void motor_test_parse_command(const char *cmd) {
    if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0) {
        printf("\n=== Available Commands ===\n");
        printf("Basic:\n");
        printf("  T0-T3     - Test single motor\n");
        printf("  A         - All motors forward test\n");
        printf("  R0-R3     - Speed ramp test\n");
        printf("  M0+5000   - Set motor duty (-10000 to +10000)\n");
        printf("  S         - Stop all motors\n");
        printf("  C         - Show encoder counts\n");
        printf("\nAdvanced:\n");
        printf("  WAVE      - Wave pattern\n");
        printf("  SYNC      - Synchronized rotation\n");
        printf("  VEL       - Velocity measurement\n");
        printf("  CIRCLE    - Circle pattern (M0-M1)\n");
        printf("  LOAD      - Load/stress test\n");
        printf("  TRAP0-3   - Trapezoidal profile\n");
        
    } else if (cmd[0] == 'T' && cmd[1] >= '0' && cmd[1] <= '3') {
        motor_test_single(cmd[1] - '0');
        
    } else if (strcmp(cmd, "A") == 0) {
        motor_test_all_forward();
        
    } else if (cmd[0] == 'R' && cmd[1] >= '0' && cmd[1] <= '3') {
        motor_test_ramp(cmd[1] - '0');
        
    } else if (strcmp(cmd, "WAVE") == 0) {
        motor_test_wave();
        
    } else if (strcmp(cmd, "SYNC") == 0) {
        motor_test_synchronized();
        
    } else if (strcmp(cmd, "VEL") == 0) {
        motor_test_velocity();
        
    } else if (strcmp(cmd, "CIRCLE") == 0) {
        motor_test_circle();
        
    } else if (strcmp(cmd, "LOAD") == 0) {
        motor_test_load();
        
    } else if (strncmp(cmd, "TRAP", 4) == 0 && cmd[4] >= '0' && cmd[4] <= '3') {
        motor_test_trapezoid(cmd[4] - '0');
        
    } else if (cmd[0] == 'M' && cmd[1] >= '0' && cmd[1] <= '3') {
        uint8_t motor = cmd[1] - '0';
        if (cmd[2] == '+' || cmd[2] == '-') {
            int16_t duty = atoi(&cmd[2]);
            pbdrv_pwm_set_duty_simple(motor, duty);
            printf("M%d set to %d\n", motor, duty);
        }
        
    } else if (strcmp(cmd, "S") == 0) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            pbdrv_pwm_set_duty_simple(i, 0);
        }
        printf("All motors stopped\n");
        
    } else if (strcmp(cmd, "C") == 0) {
        printf("Encoder counts:\n");
        for (int i = 0; i < NUM_MOTORS; i++) {
            printf("  M%d: %ld\n", i, pbdrv_counter_get_count_simple(i));
        }
        
    } else {
        printf("Unknown command: %s\n", cmd);
        printf("Type HELP for command list\n");
    }
}
