// SPDX-License-Identifier: MIT
// Pybricks Motor Control - Unified for Pico W (RP2040) and Pico 2 W (RP2350B)
// Build with -DPICO_2W for Pico 2 W, or without for Pico W

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
#include "pid_dma_rp2040.h"

// BTstack functions
extern void btstack_init(void);
extern bool btstack_is_connected(void);
extern bool nus_is_ready(void);
extern void nus_send_data(const uint8_t *data, uint16_t length);

// Motor test functions
extern void motor_test_parse_command(const char *cmd);

// ========== Platform Configuration ==========
#ifdef PICO_2W
    #define NUM_MOTORS 12
    #define PLATFORM_NAME "Pico 2 W"
    #define PLATFORM_CHIP "RP2350B (48 GPIO, 520KB RAM)"
#else
    #define NUM_MOTORS 4
    #define PLATFORM_NAME "Pico W"
    #define PLATFORM_CHIP "RP2040 (30 GPIO, 264KB RAM)"
#endif

// ========== Motor Pin Configuration ==========
static const struct {
    uint8_t enc_a;
    uint8_t enc_b;
    uint8_t pwm_a;
    uint8_t pwm_b;
} motor_pins[NUM_MOTORS] = {
    // Motors 0-3: GPIO 0-17 (both platforms)
    { .enc_a = 2,  .enc_b = 3,  .pwm_a = 4,  .pwm_b = 5  },
    { .enc_a = 6,  .enc_b = 7,  .pwm_a = 8,  .pwm_b = 9  },
    { .enc_a = 10, .enc_b = 11, .pwm_a = 12, .pwm_b = 13 },
    { .enc_a = 14, .enc_b = 15, .pwm_a = 16, .pwm_b = 17 },
#ifdef PICO_2W
    // Motors 4-7: GPIO 18-31 (Pico 2 W only)
    { .enc_a = 18, .enc_b = 19, .pwm_a = 20, .pwm_b = 21 },
    { .enc_a = 22, .enc_b = 23, .pwm_a = 24, .pwm_b = 25 },
    { .enc_a = 26, .enc_b = 27, .pwm_a = 28, .pwm_b = 29 },
    { .enc_a = 30, .enc_b = 31, .pwm_a = 32, .pwm_b = 33 },
    // Motors 8-11: GPIO 34-47 (Pico 2 W only)
    { .enc_a = 34, .enc_b = 35, .pwm_a = 36, .pwm_b = 37 },
    { .enc_a = 38, .enc_b = 39, .pwm_a = 40, .pwm_b = 41 },
    { .enc_a = 42, .enc_b = 43, .pwm_a = 44, .pwm_b = 45 },
    { .enc_a = 46, .enc_b = 47, .pwm_a = 0,  .pwm_b = 1  },  // Wrap around
#endif
};

// ========== Motor Control API ==========

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
    pid_dma_set_position_target(motor_id, target_position * 1000);
    pid_dma_enable(motor_id, true);
}

void motor_run_velocity(uint8_t motor_id, int32_t target_velocity) {
    if (motor_id >= NUM_MOTORS) return;
    pid_dma_set_speed_target(motor_id, target_velocity * 1000);
    pid_dma_enable(motor_id, true);
}

bool motor_at_target(uint8_t motor_id) {
    if (motor_id >= NUM_MOTORS) return true;
    volatile pid_dma_state_t *state = pid_dma_get_state_ptr(motor_id);
    if (!state || state->mode != 0) return true;
    int32_t error = state->target_position - state->current_position;
    if (error < 0) error = -error;
    return error < 5000;
}

int32_t motor_get_position(uint8_t motor_id) {
    int32_t pos, speed, pwm;
    pid_dma_get_state(motor_id, &pos, &speed, &pwm);
    return pos / 1000;
}

int32_t motor_get_velocity(uint8_t motor_id) {
    int32_t pos, speed, pwm;
    pid_dma_get_state(motor_id, &pos, &speed, &pwm);
    return speed / 1000;
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
        for (int i = 0; i < NUM_MOTORS; i++) {
            pid_dma_force_update(i);
        }
        control_loop_counter++;
        next_tick += 1000;
    }
}

// ========== Command Processing ==========

static void process_command(const char *cmd) {
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
            for (int i = 0; i < NUM_MOTORS; i++) {
                motor_reset_position(i);
            }
            printf("All encoders zeroed\n");
        } else if (cmd[1] == 'T') {
            printf("Test: All motors to 360 degrees\n");
            for (int i = 0; i < NUM_MOTORS; i++) {
                motor_run_target(i, 360);
            }
        } else if (cmd[1] == 'S') {
            printf("Stopping all motors\n");
            for (int i = 0; i < NUM_MOTORS; i++) {
                motor_stop(i);
            }
        } else if (cmd[1] >= '0' && cmd[1] <= '9') {
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
                int kp = 0, ki = 0, kd = 0;
                sscanf(value_str, "%d,%d,%d", &kp, &ki, &kd);
                pid_dma_set_gains(motor_id, kp << 8, ki << 8, kd << 8);
                printf("M%d: gains Kp=%d Ki=%d Kd=%d\n", motor_id, kp, ki, kd);
            }
        }
    } else if (cmd[0] == 'H' || cmd[0] == 'h' || cmd[0] == '?') {
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
        printf("Examples: M0D5000  M%dP360  M%dV1000\n\n", 
               NUM_MOTORS > 5 ? 5 : 1, NUM_MOTORS - 1);
    } else {
        // Pass to motor test parser for advanced commands
        motor_test_parse_command(cmd);
    }
}

static char cmd_buffer[128];
static uint8_t cmd_len = 0;

void process_ble_command(const uint8_t *data, uint16_t len) {
    for (int i = 0; i < len && cmd_len < sizeof(cmd_buffer) - 1; i++) {
        if (data[i] == '\n' || data[i] == '\r') {
            if (cmd_len > 0) {
                cmd_buffer[cmd_len] = '\0';
                printf("Command: %s\n", cmd_buffer);
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
    sleep_ms(1000);
    
    printf("\n");
    printf("================================================\n");
    printf("  Pybricks Motor Control - %s\n", PLATFORM_NAME);
    printf("================================================\n");
    printf("Platform: %s\n", PLATFORM_CHIP);
    printf("Motors: %d with DMA PID control\n", NUM_MOTORS);
    printf("Control: Dual-core @ 1 kHz\n\n");
    
    // Initialize DMA PID system
    printf("Initializing DMA PID system...\n");
    pid_dma_init();
    
    // Setup all motors
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
            printf("  M%d: enc=%d,%d pwm=%d,%d OK\n", i,
                   motor_pins[i].enc_a, motor_pins[i].enc_b,
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