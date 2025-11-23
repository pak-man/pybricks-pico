// SPDX-License-Identifier: MIT
// Pybricks 12-Motor System for RP2350B Pico 2 W
// Uses DMA PID for efficient closed-loop control

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "hardware/sync.h"
#include "hardware/pwm.h"

// DMA PID motor control
#include "pid_dma_rp2040.h"

// BTstack functions
extern void btstack_init(void);
extern bool btstack_is_connected(void);
extern bool nus_is_ready(void);
extern void nus_send_data(const uint8_t *data, uint16_t length);

// Legacy motor control (for compatibility)
extern void pbdrv_counter_init(void);
extern void pbdrv_counter_update(void);
extern void pbdrv_pwm_init(void);

#define NUM_MOTORS 12

// ========== Motor Pin Configuration for RP2350B ==========
// RP2350B has 48 GPIO pins, allowing 12 motors with 4 pins each
// Each motor needs: ENC_A, ENC_B (consecutive), PWM_A, PWM_B

static const struct {
    uint8_t enc_a;
    uint8_t enc_b;
    uint8_t pwm_a;
    uint8_t pwm_b;
} motor_pins[NUM_MOTORS] = {
    // Motors 0-3: GPIO 0-15
    { .enc_a = 0,  .enc_b = 1,  .pwm_a = 2,  .pwm_b = 3  },
    { .enc_a = 4,  .enc_b = 5,  .pwm_a = 6,  .pwm_b = 7  },
    { .enc_a = 8,  .enc_b = 9,  .pwm_a = 10, .pwm_b = 11 },
    { .enc_a = 12, .enc_b = 13, .pwm_a = 14, .pwm_b = 15 },
    // Motors 4-7: GPIO 16-31
    { .enc_a = 16, .enc_b = 17, .pwm_a = 18, .pwm_b = 19 },
    { .enc_a = 20, .enc_b = 21, .pwm_a = 22, .pwm_b = 23 },
    { .enc_a = 24, .enc_b = 25, .pwm_a = 26, .pwm_b = 27 },
    { .enc_a = 28, .enc_b = 29, .pwm_a = 30, .pwm_b = 31 },
    // Motors 8-11: GPIO 32-47
    { .enc_a = 32, .enc_b = 33, .pwm_a = 34, .pwm_b = 35 },
    { .enc_a = 36, .enc_b = 37, .pwm_a = 38, .pwm_b = 39 },
    { .enc_a = 40, .enc_b = 41, .pwm_a = 42, .pwm_b = 43 },
    { .enc_a = 44, .enc_b = 45, .pwm_a = 46, .pwm_b = 47 },
};

// ========== Motor Control API (wraps DMA PID) ==========

void motor_stop(uint8_t motor_id) {
    if (motor_id >= NUM_MOTORS) return;
    pid_dma_enable(motor_id, false);
    pid_dma_set_pwm(motor_id, 0);
}

void motor_run_duty(uint8_t motor_id, int16_t duty) {
    if (motor_id >= NUM_MOTORS) return;
    pid_dma_set_pwm(motor_id, duty);
}

void motor_run_target(uint8_t motor_id, int32_t target_position) {
    if (motor_id >= NUM_MOTORS) return;
    // Convert degrees to millidegrees
    pid_dma_set_position_target(motor_id, target_position * 1000);
    pid_dma_enable(motor_id, true);
}

void motor_run_velocity(uint8_t motor_id, int32_t target_velocity) {
    if (motor_id >= NUM_MOTORS) return;
    // Convert deg/s to millideg/s
    pid_dma_set_speed_target(motor_id, target_velocity * 1000);
    pid_dma_enable(motor_id, true);
}

bool motor_at_target(uint8_t motor_id) {
    if (motor_id >= NUM_MOTORS) return true;
    
    volatile pid_dma_state_t *state = pid_dma_get_state_ptr(motor_id);
    if (!state || state->mode != 0) return true;  // Not in position mode
    
    int32_t error = state->target_position - state->current_position;
    if (error < 0) error = -error;
    return error < 5000;  // Within 5 degrees
}

int32_t motor_get_position(uint8_t motor_id) {
    int32_t pos, speed, pwm;
    pid_dma_get_state(motor_id, &pos, &speed, &pwm);
    return pos / 1000;  // Return degrees
}

int32_t motor_get_velocity(uint8_t motor_id) {
    int32_t pos, speed, pwm;
    pid_dma_get_state(motor_id, &pos, &speed, &pwm);
    return speed / 1000;  // Return deg/s
}

void motor_reset_position(uint8_t motor_id) {
    pid_dma_reset_position(motor_id);
}

// ========== Core 1: Real-Time Control Loop ==========

static volatile bool core1_running = false;
static volatile uint32_t control_loop_counter = 0;
static volatile uint32_t control_loop_overruns = 0;

void core1_control_task(void) {
    printf("Core 1: Starting real-time control loop @ 1 kHz\n");
    
    uint64_t next_tick = time_us_64() + 1000;
    
    while (core1_running) {
        uint64_t now = time_us_64();
        
        if (now > next_tick + 100) {
            control_loop_overruns++;
        }
        
        while (time_us_64() < next_tick) {
            tight_loop_contents();
        }
        
        // === 1 kHz Control Loop ===
        
        // Run PID update for all motors
        for (int i = 0; i < NUM_MOTORS; i++) {
            pid_dma_force_update(i);
        }
        
        control_loop_counter++;
        next_tick += 1000;
    }
}

// ========== Command Processing ==========

void process_command(const char *cmd) {
    if (cmd[0] == 'M') {
        if (cmd[1] == 'A') {
            // Status all motors
            char status[512];
            int len = snprintf(status, sizeof(status), "Motors:\n");
            for (int i = 0; i < NUM_MOTORS; i++) {
                int32_t pos = motor_get_position(i);
                int32_t vel = motor_get_velocity(i);
                len += snprintf(status + len, sizeof(status) - len,
                    " %2d: pos=%6ld vel=%6ld\n", i, pos, vel);
            }
            printf("%s", status);
            if (nus_is_ready()) {
                nus_send_data((uint8_t*)status, len);
            }
            
        } else if (cmd[1] == 'Z') {
            // Zero all encoders
            for (int i = 0; i < NUM_MOTORS; i++) {
                motor_reset_position(i);
            }
            printf("All encoders zeroed\n");
            
        } else if (cmd[1] == 'T') {
            // Coordinated test
            printf("Test: All motors to 360 degrees\n");
            for (int i = 0; i < NUM_MOTORS; i++) {
                motor_run_target(i, 360);
            }
            
        } else if (cmd[1] == 'S') {
            // Stop all
            printf("Stopping all motors\n");
            for (int i = 0; i < NUM_MOTORS; i++) {
                motor_stop(i);
            }
            
        } else if (cmd[1] >= '0' && cmd[1] <= '9') {
            // Parse motor ID (M0-M11)
            int motor_id = cmd[1] - '0';
            int cmd_offset = 2;
            if (cmd[2] >= '0' && cmd[2] <= '9') {
                motor_id = motor_id * 10 + (cmd[2] - '0');
                cmd_offset = 3;
            }
            
            if (motor_id >= NUM_MOTORS) {
                printf("Invalid motor ID: %d (max %d)\n", motor_id, NUM_MOTORS - 1);
                return;
            }
            
            char op = cmd[cmd_offset];
            const char *value_str = &cmd[cmd_offset + 1];
            
            if (op == 'D' || op == 'd') {
                int duty = atoi(value_str);
                motor_run_duty(motor_id, duty);
                printf("M%d: duty %d\n", motor_id, duty);
                
            } else if (op == 'P' || op == 'p') {
                int pos = atoi(value_str);
                motor_run_target(motor_id, pos);
                printf("M%d: target %d deg\n", motor_id, pos);
                
            } else if (op == 'V' || op == 'v') {
                int vel = atoi(value_str);
                motor_run_velocity(motor_id, vel);
                printf("M%d: velocity %d deg/s\n", motor_id, vel);
                
            } else if (op == 'S' || op == 's') {
                motor_stop(motor_id);
                printf("M%d: stopped\n", motor_id);
                
            } else if (op == 'R' || op == 'r') {
                motor_reset_position(motor_id);
                printf("M%d: position reset\n", motor_id);
                
            } else if (op == 'G' || op == 'g') {
                // Set PID gains: M0G100,10,50 = Kp=100, Ki=10, Kd=50
                int kp = 0, ki = 0, kd = 0;
                sscanf(value_str, "%d,%d,%d", &kp, &ki, &kd);
                pid_dma_set_gains(motor_id, kp << 8, ki << 8, kd << 8);
                printf("M%d: gains Kp=%d Ki=%d Kd=%d\n", motor_id, kp, ki, kd);
            }
        }
    } else if (cmd[0] == 'H' || cmd[0] == 'h' || cmd[0] == '?') {
        // Help
        printf("\nCommands:\n");
        printf("  M<id>D<duty>    - Duty cycle (-10000 to +10000)\n");
        printf("  M<id>P<pos>     - Position target (degrees)\n");
        printf("  M<id>V<vel>     - Velocity target (deg/s)\n");
        printf("  M<id>S          - Stop motor\n");
        printf("  M<id>R          - Reset encoder\n");
        printf("  M<id>G<p,i,d>   - Set PID gains\n");
        printf("  MA              - Status all motors\n");
        printf("  MZ              - Zero all encoders\n");
        printf("  MT              - Test all (360 deg)\n");
        printf("  MS              - Stop all\n");
        printf("Examples: M0D5000  M5P360  M11V1000\n\n");
    }
}

static char cmd_buffer[128];
static uint8_t cmd_len = 0;

void process_ble_command(const uint8_t *data, uint16_t len) {
    for (int i = 0; i < len && cmd_len < sizeof(cmd_buffer) - 1; i++) {
        if (data[i] == '\n' || data[i] == '\r') {
            if (cmd_len > 0) {
                cmd_buffer[cmd_len] = '\0';
                process_command(cmd_buffer);
                cmd_len = 0;
            }
        } else {
            cmd_buffer[cmd_len++] = data[i];
        }
    }
}

// ========== Main Application ==========

int main() {
    stdio_init_all();
    sleep_ms(1000);  // Wait for USB
    
    printf("\n");
    printf("================================================\n");
    printf("  Pybricks 12-Motor System - Pico 2 W\n");
    printf("================================================\n");
    printf("Platform: RP2350B (48 GPIO, 520KB RAM)\n");
    printf("Motors: 12 with DMA PID control\n");
    printf("Control: Dual-core @ 1 kHz\n\n");
    
    // Initialize DMA PID system
    printf("Initializing DMA PID system...\n");
    pid_dma_init();
    
    // Setup all 12 motors
    printf("Configuring motors:\n");
    int motors_ok = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        pid_dma_hw_config_t config = {
            .encoder_pio = NULL,
            .encoder_pin_a = motor_pins[i].enc_a,
            .encoder_pin_b = motor_pins[i].enc_b,
            .pwm_pio = NULL,
            .pwm_pin_a = motor_pins[i].pwm_a,
            .pwm_pin_b = motor_pins[i].pwm_b,
            .counts_per_rev = 360,
        };
        
        if (pid_dma_motor_setup(i, &config) == 0) {
            motors_ok++;
            printf("  M%d: GPIO %d-%d (enc), %d-%d (pwm) OK\n",
                   i, motor_pins[i].enc_a, motor_pins[i].enc_b,
                   motor_pins[i].pwm_a, motor_pins[i].pwm_b);
        } else {
            printf("  M%d: FAILED\n", i);
        }
    }
    printf("Motors initialized: %d/%d\n\n", motors_ok, NUM_MOTORS);
    
    // Start Core 1 control loop
    printf("Starting Core 1 control loop...\n");
    core1_running = true;
    multicore_launch_core1(core1_control_task);
    sleep_ms(100);
    
    // Initialize Bluetooth
    printf("Initializing Bluetooth...\n");
    if (cyw43_arch_init()) {
        printf("ERROR: CYW43 init failed!\n");
        return 1;
    }
    btstack_init();
    
    printf("\n================================================\n");
    printf("System Ready! Type 'H' for help.\n");
    printf("================================================\n\n");
    
    // Main loop (Core 0)
    uint32_t last_status = 0;
    uint32_t last_blink = 0;
    bool led_state = false;
    
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // LED blink
        if (now - last_blink > (btstack_is_connected() ? 100 : 500)) {
            led_state = !led_state;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
            last_blink = now;
        }
        
        // Periodic status when connected
        if (nus_is_ready() && (now - last_status > 2000)) {
            char status[128];
            int len = snprintf(status, sizeof(status),
                "Loop:%lu Over:%lu M0:%ld M1:%ld M2:%ld M3:%ld\n",
                control_loop_counter, control_loop_overruns,
                motor_get_position(0), motor_get_position(1),
                motor_get_position(2), motor_get_position(3));
            nus_send_data((uint8_t*)status, len);
            last_status = now;
        }
        
        // Process USB serial commands
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            uint8_t byte = (uint8_t)c;
            process_ble_command(&byte, 1);
        }
        
        sleep_ms(10);
    }
    
    return 0;
}
