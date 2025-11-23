// SPDX-License-Identifier: MIT
// Unified motor test suite - supports 4-motor (Pico W) and 12-motor (Pico 2 W)

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"

// Platform configuration
#ifdef PICO_2W
    #define NUM_MOTORS 12
#else
    #define NUM_MOTORS 4
#endif

// External motor control API (from main.c)
extern void motor_stop(uint8_t motor_id);
extern void motor_run_duty(uint8_t motor_id, int16_t duty);
extern void motor_run_target(uint8_t motor_id, int32_t target_position);
extern void motor_run_velocity(uint8_t motor_id, int32_t target_velocity);
extern bool motor_at_target(uint8_t motor_id);
extern int32_t motor_get_position(uint8_t motor_id);
extern int32_t motor_get_velocity(uint8_t motor_id);
extern void motor_reset_position(uint8_t motor_id);

// ========== Basic Tests ==========

void motor_test_all_forward(void) {
    printf("\n=== All Motors Forward Test ===\n");
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_reset_position(i);
    }
    
    printf("Running all %d motors at 50%% for 2 seconds...\n", NUM_MOTORS);
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_run_duty(i, 5000);
    }
    
    for (int t = 0; t < 20; t++) {
        sleep_ms(100);
        if (t % 5 == 0) {
            printf("t=%d.%ds: ", t/10, t%10);
            // Print first 4 motors always, indicate more if present
            for (int i = 0; i < (NUM_MOTORS > 4 ? 4 : NUM_MOTORS); i++) {
                printf("M%d=%ld ", i, motor_get_position(i));
            }
            if (NUM_MOTORS > 4) printf("...");
            printf("\n");
        }
    }
    
    printf("Stopping...\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_stop(i);
    }
    
    printf("Final counts:\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        printf("  Motor %d: %ld deg\n", i, motor_get_position(i));
    }
}

void motor_test_single(uint8_t motor_id) {
    if (motor_id >= NUM_MOTORS) {
        printf("Invalid motor ID: %d (max %d)\n", motor_id, NUM_MOTORS - 1);
        return;
    }
    
    printf("\n=== Single Motor Test (M%d) ===\n", motor_id);
    
    motor_reset_position(motor_id);
    
    printf("Forward 30%%...\n");
    motor_run_duty(motor_id, 3000);
    sleep_ms(1000);
    int32_t fwd = motor_get_position(motor_id);
    printf("  Position after forward: %ld deg\n", fwd);
    
    motor_stop(motor_id);
    sleep_ms(500);
    
    printf("Reverse 30%%...\n");
    motor_run_duty(motor_id, -3000);
    sleep_ms(1000);
    int32_t rev = motor_get_position(motor_id);
    printf("  Position after reverse: %ld deg\n", rev);
    printf("  Net position: %ld deg (should be near 0)\n", rev);
    
    motor_stop(motor_id);
}

// ========== Advanced Tests ==========

void motor_test_ramp(uint8_t motor_id) {
    if (motor_id >= NUM_MOTORS) {
        printf("Invalid motor ID: %d (max %d)\n", motor_id, NUM_MOTORS - 1);
        return;
    }
    
    printf("\n=== Speed Ramp Test (M%d) ===\n", motor_id);
    
    motor_reset_position(motor_id);
    
    printf("Ramping up (0%% to 80%%)...\n");
    for (int duty = 0; duty <= 8000; duty += 200) {
        motor_run_duty(motor_id, duty);
        sleep_ms(50);
    }
    
    sleep_ms(500);
    int32_t count_max = motor_get_position(motor_id);
    printf("Position at max speed: %ld deg\n", count_max);
    
    printf("Ramping down (80%% to 0%%)...\n");
    for (int duty = 8000; duty >= 0; duty -= 200) {
        motor_run_duty(motor_id, duty);
        sleep_ms(50);
    }
    
    int32_t count_final = motor_get_position(motor_id);
    printf("Final position: %ld deg\n", count_final);
    motor_stop(motor_id);
}

void motor_test_wave(void) {
    printf("\n=== Wave Pattern Test ===\n");
    printf("Motors activate sequentially (%d motors)...\n", NUM_MOTORS);
    
    for (int cycle = 0; cycle < 3; cycle++) {
        printf("Cycle %d:\n", cycle + 1);
        for (int i = 0; i < NUM_MOTORS; i++) {
            printf("  M%d ON\n", i);
            motor_run_duty(i, 5000);
            sleep_ms(200);
            motor_stop(i);
        }
    }
    printf("Wave complete\n");
}

void motor_test_synchronized(void) {
    printf("\n=== Synchronized Rotation Test ===\n");
    printf("All %d motors run to 720 deg\n", NUM_MOTORS);
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_reset_position(i);
    }
    
    int32_t target = 720;
    printf("All motors -> %ld deg\n", target);
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_run_target(i, target);
    }
    
    // Monitor progress
    for (int t = 0; t < 50; t++) {
        sleep_ms(100);
        if (t % 10 == 0) {
            bool all_done = true;
            printf("Progress: ");
            for (int i = 0; i < (NUM_MOTORS > 4 ? 4 : NUM_MOTORS); i++) {
                int32_t pos = motor_get_position(i);
                printf("M%d:%ld ", i, pos);
                if (!motor_at_target(i)) all_done = false;
            }
            if (NUM_MOTORS > 4) printf("...");
            printf("\n");
            
            if (all_done) {
                printf("All motors reached target!\n");
                break;
            }
        }
    }
    
    // Stop all
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_stop(i);
    }
}

void motor_test_velocity(void) {
    printf("\n=== Velocity Measurement Test ===\n");
    
    // Run motors at different speeds
    printf("Setting velocities (deg/s):\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        int32_t vel = 100 * (i + 1);
        motor_run_velocity(i, vel);
        printf("  M%d: %ld deg/s\n", i, vel);
    }
    
    // Measure for 3 seconds
    for (int t = 0; t < 30; t++) {
        sleep_ms(100);
        if (t % 10 == 0) {
            printf("Measured velocities:\n");
            for (int i = 0; i < (NUM_MOTORS > 6 ? 6 : NUM_MOTORS); i++) {
                int32_t vel = motor_get_velocity(i);
                printf("  M%d: %ld deg/s\n", i, vel);
            }
        }
    }
    
    // Stop all
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_stop(i);
    }
}

void motor_test_circle(void) {
    printf("\n=== Circle Pattern Test ===\n");
    printf("Motors 0-1 trace a circle\n");
    
    const int steps = 36;
    const int32_t radius = 90;  // degrees
    
    motor_reset_position(0);
    motor_reset_position(1);
    
    for (int step = 0; step < steps; step++) {
        float angle = (2.0f * M_PI * step) / steps;
        
        int32_t x = (int32_t)(radius * cosf(angle));
        int32_t y = (int32_t)(radius * sinf(angle));
        
        motor_run_target(0, x);
        motor_run_target(1, y);
        
        if (step % 6 == 0) {
            printf("Step %d: M0=%ld, M1=%ld deg\n", step, x, y);
        }
        
        // Wait for motors
        for (int wait = 0; wait < 20; wait++) {
            if (motor_at_target(0) && motor_at_target(1)) break;
            sleep_ms(10);
        }
        sleep_ms(50);
    }
    
    motor_run_target(0, 0);
    motor_run_target(1, 0);
    printf("Circle complete\n");
}

void motor_test_load(void) {
    printf("\n=== Load Test ===\n");
    printf("WARNING: Running all %d motors at 80%% duty!\n", NUM_MOTORS);
    printf("Ensure adequate power supply.\n");
    printf("Starting in 3 seconds...\n");
    sleep_ms(3000);
    
    printf("All motors -> 80%% duty\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_run_duty(i, 8000);
    }
    
    // Monitor for 5 seconds
    for (int t = 0; t < 50; t++) {
        sleep_ms(100);
        if (t % 10 == 0) {
            printf("t=%ds: ", t/10);
            for (int i = 0; i < (NUM_MOTORS > 4 ? 4 : NUM_MOTORS); i++) {
                int32_t vel = motor_get_velocity(i);
                printf("M%d:%ld ", i, vel);
            }
            printf("deg/s\n");
        }
    }
    
    // Stop all
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_stop(i);
    }
    printf("Load test complete\n");
}

void motor_test_trapezoid(uint8_t motor_id) {
    if (motor_id >= NUM_MOTORS) {
        printf("Invalid motor ID: %d (max %d)\n", motor_id, NUM_MOTORS - 1);
        return;
    }
    
    printf("\n=== Trapezoidal Profile Test (M%d) ===\n", motor_id);
    
    motor_reset_position(motor_id);
    
    printf("Phase 1: Accelerating...\n");
    for (int duty = 0; duty <= 6000; duty += 100) {
        motor_run_duty(motor_id, duty);
        sleep_ms(20);
    }
    
    printf("Phase 2: Cruising at 60%%...\n");
    sleep_ms(1000);
    printf("  Position: %ld deg\n", motor_get_position(motor_id));
    
    printf("Phase 3: Decelerating...\n");
    for (int duty = 6000; duty >= 0; duty -= 100) {
        motor_run_duty(motor_id, duty);
        sleep_ms(20);
    }
    
    int32_t final_pos = motor_get_position(motor_id);
    printf("Final position: %ld deg\n", final_pos);
    motor_stop(motor_id);
}

#ifdef PICO_2W
// Hexapod gait simulation (12 motors: 6 legs × 2 DOF)
void motor_test_hexapod_gait(void) {
    printf("\n=== Hexapod Gait Simulation ===\n");
    printf("Simulating tripod gait (6 legs, 2 DOF each)\n");
    
    const int leg_pairs[2][3] = {
        {0, 2, 4},  // First tripod
        {1, 3, 5}   // Second tripod
    };
    
    for (int cycle = 0; cycle < 5; cycle++) {
        for (int tripod = 0; tripod < 2; tripod++) {
            printf("Cycle %d, Tripod %d\n", cycle, tripod);
            
            for (int i = 0; i < 3; i++) {
                int leg = leg_pairs[tripod][i];
                int hip_motor = leg * 2;
                int knee_motor = leg * 2 + 1;
                
                motor_run_target(knee_motor, 90);  // Lift
                motor_run_target(hip_motor, 45);   // Forward
            }
            sleep_ms(300);
            
            for (int i = 0; i < 3; i++) {
                int leg = leg_pairs[tripod][i];
                int knee_motor = leg * 2 + 1;
                motor_run_target(knee_motor, 0);  // Lower
            }
            sleep_ms(300);
        }
    }
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        motor_run_target(i, 0);
    }
    printf("Hexapod gait complete\n");
}
#endif

// ========== Command Parser ==========

void motor_test_parse_command(const char *cmd) {
    if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0) {
        printf("\n=== Available Commands ===\n");
        printf("Platform: %d motors\n\n", NUM_MOTORS);
        printf("Basic:\n");
        printf("  T<id>     - Test single motor (0-%d)\n", NUM_MOTORS - 1);
        printf("  A         - All motors forward test\n");
        printf("  R<id>     - Speed ramp test\n");
        printf("  S         - Stop all motors\n");
        printf("  C         - Show positions\n");
        printf("\nAdvanced:\n");
        printf("  WAVE      - Wave pattern\n");
        printf("  SYNC      - Synchronized rotation\n");
        printf("  VEL       - Velocity measurement\n");
        printf("  CIRCLE    - Circle pattern (M0-M1)\n");
        printf("  LOAD      - Load/stress test\n");
        printf("  TRAP<id>  - Trapezoidal profile\n");
#ifdef PICO_2W
        printf("  HEXAPOD   - Hexapod gait simulation\n");
#endif
        
    } else if (cmd[0] == 'T' && cmd[1] >= '0' && cmd[1] <= '9') {
        int id = atoi(&cmd[1]);
        motor_test_single(id);
        
    } else if (strcmp(cmd, "A") == 0) {
        motor_test_all_forward();
        
    } else if (cmd[0] == 'R' && cmd[1] >= '0' && cmd[1] <= '9') {
        int id = atoi(&cmd[1]);
        motor_test_ramp(id);
        
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
        
    } else if (strncmp(cmd, "TRAP", 4) == 0 && cmd[4] >= '0' && cmd[4] <= '9') {
        int id = atoi(&cmd[4]);
        motor_test_trapezoid(id);
        
#ifdef PICO_2W
    } else if (strcmp(cmd, "HEXAPOD") == 0) {
        motor_test_hexapod_gait();
#endif
        
    } else if (strcmp(cmd, "S") == 0) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            motor_stop(i);
        }
        printf("All motors stopped\n");
        
    } else if (strcmp(cmd, "C") == 0) {
        printf("Motor positions:\n");
        for (int i = 0; i < NUM_MOTORS; i++) {
            printf("  M%d: %ld deg\n", i, motor_get_position(i));
        }
        
    } else {
        printf("Unknown command: %s\n", cmd);
        printf("Type HELP for command list\n");
    }
}