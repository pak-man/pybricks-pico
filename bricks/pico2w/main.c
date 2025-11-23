#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "hardware/timer.h"
#include "hardware/sync.h"

// BTstack functions
extern void btstack_init(void);
extern bool btstack_is_connected(void);
extern bool nus_is_ready(void);
extern void nus_send_data(const uint8_t *data, uint16_t length);

// Motor control functions
extern void pbdrv_counter_init(void);
extern void pbdrv_counter_update(void);
extern void pbdrv_pwm_init(void);
extern int32_t pbdrv_counter_get_count_simple(uint8_t id);
extern int32_t pbdrv_counter_get_rate_simple(uint8_t id);
extern void pbdrv_counter_reset(uint8_t id);
extern void pbdrv_pwm_set_duty_simple(uint8_t id, int16_t duty);
extern void pbdrv_pwm_generate_trapezoid(int16_t *profile, uint32_t length,
                                         int16_t max_speed, uint32_t accel, uint32_t decel);
extern void pbdrv_pwm_run_profile(uint8_t id, int16_t *profile, uint32_t length);

#define NUM_MOTORS 12

// ========== PID Controller ==========

typedef struct {
    // PID gains
    float kp;           // Proportional gain
    float ki;           // Integral gain
    float kd;           // Derivative gain
    
    // Target
    int32_t target_position;
    int32_t target_velocity;
    
    // State
    int32_t last_error;
    float integral;
    float integral_limit;
    
    // Control mode
    enum {
        MODE_IDLE,
        MODE_POSITION,
        MODE_VELOCITY,
        MODE_DUTY
    } mode;
    
    // Direct duty cycle (when in MODE_DUTY)
    int16_t duty_override;
    
    // Limits
    int16_t max_duty;
    int32_t position_tolerance;
    
    // Status
    bool at_target;
} pid_controller_t;

static pid_controller_t pid_controllers[NUM_MOTORS];

// Initialize PID controller
void pid_init(uint8_t motor_id, float kp, float ki, float kd) {
    if (motor_id >= NUM_MOTORS) return;
    
    pid_controller_t *pid = &pid_controllers[motor_id];
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target_position = 0;
    pid->target_velocity = 0;
    pid->last_error = 0;
    pid->integral = 0;
    pid->integral_limit = 5000.0f;  // Anti-windup
    pid->mode = MODE_IDLE;
    pid->max_duty = 10000;  // 100%
    pid->position_tolerance = 10;  // 10 encoder ticks
    pid->at_target = false;
}

// PID update (called at 1 kHz)
int16_t pid_update(uint8_t motor_id) {
    if (motor_id >= NUM_MOTORS) return 0;
    
    pid_controller_t *pid = &pid_controllers[motor_id];
    
    // Get current state
    int32_t current_position = pbdrv_counter_get_count_simple(motor_id);
    int32_t current_velocity = pbdrv_counter_get_rate_simple(motor_id);
    
    int16_t output = 0;
    
    switch (pid->mode) {
        case MODE_IDLE:
            output = 0;
            break;
            
        case MODE_DUTY:
            // Direct duty cycle control (no PID)
            output = pid->duty_override;
            break;
            
        case MODE_POSITION: {
            // Position control with velocity feedforward
            int32_t position_error = pid->target_position - current_position;
            
            // Check if at target
            if (abs(position_error) <= pid->position_tolerance) {
                pid->at_target = true;
                output = 0;  // Hold position
                pid->integral = 0;
                break;
            }
            pid->at_target = false;
            
            // PID calculation
            pid->integral += position_error;
            
            // Anti-windup
            if (pid->integral > pid->integral_limit) {
                pid->integral = pid->integral_limit;
            } else if (pid->integral < -pid->integral_limit) {
                pid->integral = -pid->integral_limit;
            }
            
            int32_t derivative = position_error - pid->last_error;
            
            float output_f = pid->kp * position_error +
                            pid->ki * pid->integral +
                            pid->kd * derivative;
            
            pid->last_error = position_error;
            
            // Clamp output
            if (output_f > pid->max_duty) output_f = pid->max_duty;
            if (output_f < -pid->max_duty) output_f = -pid->max_duty;
            
            output = (int16_t)output_f;
            break;
        }
        
        case MODE_VELOCITY: {
            // Velocity control
            int32_t velocity_error = pid->target_velocity - current_velocity;
            
            pid->integral += velocity_error;
            
            // Anti-windup
            if (pid->integral > pid->integral_limit) {
                pid->integral = pid->integral_limit;
            } else if (pid->integral < -pid->integral_limit) {
                pid->integral = -pid->integral_limit;
            }
            
            int32_t derivative = velocity_error - pid->last_error;
            
            float output_f = pid->kp * velocity_error +
                            pid->ki * pid->integral +
                            pid->kd * derivative;
            
            pid->last_error = velocity_error;
            
            // Clamp
            if (output_f > pid->max_duty) output_f = pid->max_duty;
            if (output_f < -pid->max_duty) output_f = -pid->max_duty;
            
            output = (int16_t)output_f;
            break;
        }
    }
    
    return output;
}

// ========== Motor Control API ==========

void motor_stop(uint8_t motor_id) {
    if (motor_id >= NUM_MOTORS) return;
    pid_controllers[motor_id].mode = MODE_IDLE;
    pbdrv_pwm_set_duty_simple(motor_id, 0);
}

void motor_run_duty(uint8_t motor_id, int16_t duty) {
    if (motor_id >= NUM_MOTORS) return;
    pid_controller_t *pid = &pid_controllers[motor_id];
    pid->mode = MODE_DUTY;
    pid->duty_override = duty;
}

void motor_run_target(uint8_t motor_id, int32_t target_position) {
    if (motor_id >= NUM_MOTORS) return;
    pid_controller_t *pid = &pid_controllers[motor_id];
    pid->mode = MODE_POSITION;
    pid->target_position = target_position;
    pid->integral = 0;
    pid->at_target = false;
}

void motor_run_velocity(uint8_t motor_id, int32_t target_velocity) {
    if (motor_id >= NUM_MOTORS) return;
    pid_controller_t *pid = &pid_controllers[motor_id];
    pid->mode = MODE_VELOCITY;
    pid->target_velocity = target_velocity;
    pid->integral = 0;
}

bool motor_at_target(uint8_t motor_id) {
    if (motor_id >= NUM_MOTORS) return false;
    return pid_controllers[motor_id].at_target;
}

// ========== Core 1: Real-Time Control Loop ==========

static volatile bool core1_running = false;
static volatile uint32_t control_loop_counter = 0;
static volatile uint32_t control_loop_overruns = 0;

void core1_control_task(void) {
    printf("Core 1: Starting real-time control loop @ 1 kHz\n");
    
    uint64_t next_tick = time_us_64() + 1000;  // First tick in 1ms
    
    while (core1_running) {
        uint64_t now = time_us_64();
        
        // Check for overrun
        if (now > next_tick + 100) {  // >100us late
            control_loop_overruns++;
        }
        
        // Wait until next tick
        while (time_us_64() < next_tick) {
            tight_loop_contents();
        }
        
        // === 1 kHz Control Loop ===
        
        // 1. Update encoders (read from PIO FIFOs)
        pbdrv_counter_update();
        
        // 2. Run PID for all motors
        for (int i = 0; i < NUM_MOTORS; i++) {
            int16_t duty = pid_update(i);
            pbdrv_pwm_set_duty_simple(i, duty);
        }
        
        control_loop_counter++;
        next_tick += 1000;  // Next tick in 1ms
    }
}

// ========== Command Processing ==========

void process_command(const char *cmd) {
    // Command format:
    // M<id>D<duty>       - Run motor at duty cycle (e.g., M0D5000 = motor 0 at 50%)
    // M<id>P<pos>        - Run to position (e.g., M0P360 = motor 0 to 360 degrees)
    // M<id>V<vel>        - Run at velocity (e.g., M0V1000 = motor 0 at 1000 ticks/s)
    // M<id>S             - Stop motor (e.g., M0S)
    // MA                 - Status all motors
    // MZ                 - Zero all encoders
    // MT                 - Coordinated test (all motors)
    
    if (cmd[0] == 'M') {
        if (cmd[1] == 'A') {
            // Status all motors
            char status[512];
            int len = snprintf(status, sizeof(status), "Motors:\n");
            for (int i = 0; i < NUM_MOTORS; i++) {
                len += snprintf(status + len, sizeof(status) - len,
                    " %2d: pos=%6ld vel=%6ld mode=%d\n",
                    i,
                    pbdrv_counter_get_count_simple(i),
                    pbdrv_counter_get_rate_simple(i),
                    pid_controllers[i].mode
                );
            }
            printf("%s", status);
            if (nus_is_ready()) {
                nus_send_data((uint8_t*)status, len);
            }
            
        } else if (cmd[1] == 'Z') {
            // Zero all encoders
            for (int i = 0; i < NUM_MOTORS; i++) {
                pbdrv_counter_reset(i);
            }
            printf("All encoders zeroed\n");
            
        } else if (cmd[1] == 'T') {
            // Coordinated test
            printf("Coordinated test: All motors to 360 ticks\n");
            for (int i = 0; i < NUM_MOTORS; i++) {
                motor_run_target(i, 360);
            }
            
        } else if (cmd[1] >= '0' && cmd[1] <= '9') {
            // Parse motor ID (supports M0-M11)
            int motor_id = cmd[1] - '0';
            if (cmd[2] >= '0' && cmd[2] <= '9') {
                motor_id = motor_id * 10 + (cmd[2] - '0');
            }
            
            if (motor_id >= NUM_MOTORS) {
                printf("Invalid motor ID: %d\n", motor_id);
                return;
            }
            
            // Parse command
            char op = cmd[2];
            if (motor_id >= 10) op = cmd[3];  // Adjust for 2-digit ID
            
            if (op == 'D') {
                // Duty cycle
                int duty = atoi(&cmd[3 + (motor_id >= 10 ? 1 : 0)]);
                motor_run_duty(motor_id, duty);
                printf("Motor %d: duty %d\n", motor_id, duty);
                
            } else if (op == 'P') {
                // Position
                int pos = atoi(&cmd[3 + (motor_id >= 10 ? 1 : 0)]);
                motor_run_target(motor_id, pos);
                printf("Motor %d: target position %d\n", motor_id, pos);
                
            } else if (op == 'V') {
                // Velocity
                int vel = atoi(&cmd[3 + (motor_id >= 10 ? 1 : 0)]);
                motor_run_velocity(motor_id, vel);
                printf("Motor %d: target velocity %d\n", motor_id, vel);
                
            } else if (op == 'S') {
                // Stop
                motor_stop(motor_id);
                printf("Motor %d: stopped\n", motor_id);
            }
        }
    }
}

// Process BLE commands
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
    
    printf("\n");
    printf("================================================\n");
    printf("  Pybricks 12-Motor System - RP2350B\n");
    printf("================================================\n");
    printf("Firmware: 2.0.0\n");
    printf("Platform: RP2350B (48 GPIO, 520KB RAM)\n");
    printf("Motors: 12 with PID control\n");
    printf("Encoders: 12 with DMA + 3 PIO blocks\n");
    printf("Control: Dual-core @ 1 kHz\n\n");
    
    // Initialize hardware
    printf("Initializing motor control...\n");
    pbdrv_pwm_init();
    pbdrv_counter_init();
    
    // Initialize PID controllers
    printf("Initializing PID controllers...\n");
    for (int i = 0; i < NUM_MOTORS; i++) {
        // Tuned PID gains (adjust for your motors)
        pid_init(i, 
            10.0f,   // Kp - proportional
            0.1f,    // Ki - integral
            2.0f     // Kd - derivative
        );
    }
    
    // Start Core 1 real-time loop
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
    
    printf("\n");
    printf("================================================\n");
    printf("System Ready!\n");
    printf("================================================\n");
    printf("Connect via nRF Connect or Pybricks Code\n\n");
    printf("Commands (via NUS or USB):\n");
    printf("  M<id>D<duty>  - Run motor at duty (-10000 to +10000)\n");
    printf("  M<id>P<pos>   - Run to position (encoder ticks)\n");
    printf("  M<id>V<vel>   - Run at velocity (ticks/second)\n");
    printf("  M<id>S        - Stop motor\n");
    printf("  MA            - Status of all motors\n");
    printf("  MZ            - Zero all encoders\n");
    printf("  MT            - Test all motors\n\n");
    printf("Examples:\n");
    printf("  M0D5000       - Motor 0 at 50%% forward\n");
    printf("  M5P360        - Motor 5 to 360 ticks\n");
    printf("  M11V1000      - Motor 11 at 1000 ticks/s\n");
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
        
        // Send status every 1 second when connected
        if (nus_is_ready() && (now - last_status > 1000)) {
            char status[256];
            int len = snprintf(status, sizeof(status),
                "Loop:%lu Overruns:%lu M0:%ld M1:%ld M2:%ld M3:%ld\n",
                control_loop_counter, control_loop_overruns,
                pbdrv_counter_get_count_simple(0),
                pbdrv_counter_get_count_simple(1),
                pbdrv_counter_get_count_simple(2),
                pbdrv_counter_get_count_simple(3)
            );
            nus_send_data((uint8_t*)status, len);
            last_status = now;
        }
        
        // Process USB commands
        int c = getchar_timeout_us(0);
        if (c != PICO_ERROR_TIMEOUT) {
            uint8_t byte = (uint8_t)c;
            process_ble_command(&byte, 1);
        }
        
        sleep_ms(10);
    }
    
    return 0;
}
