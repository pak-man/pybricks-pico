#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/timer.h"
#include "../../lib/pbio/drv/pid_dma/pid_dma_rp2040.h"

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

// Motor test functions
extern void motor_test_parse_command(const char *cmd);

// Command buffer for BLE
static char cmd_buffer[64];
static uint8_t cmd_len = 0;

// 1 kHz timer callback for encoder updates
static bool timer_callback(struct repeating_timer *t) {
    pbdrv_counter_update();
    
    // Run DMA PID update for all enabled motors
    for (int i = 0; i < 4; i++) {
        pid_dma_force_update(i);
    }
    return true;
}

// Process received command from BLE
void process_ble_command(const uint8_t *data, uint16_t len) {
    for (int i = 0; i < len && cmd_len < sizeof(cmd_buffer) - 1; i++) {
        if (data[i] == '\n' || data[i] == '\r') {
            if (cmd_len > 0) {
                cmd_buffer[cmd_len] = '\0';
                printf("Command: %s\n", cmd_buffer);
                motor_test_parse_command(cmd_buffer);
                cmd_len = 0;
            }
        } else {
            cmd_buffer[cmd_len++] = data[i];
        }
    }
}

int main() {
    stdio_init_all();
    
    printf("\n========================================\n");
    printf("  Pybricks Motor Control - Pico W\n");
    printf("========================================\n");
    printf("Firmware: 1.0.0\n");
    printf("Motors: 4 (with PIO encoders + DMA)\n");
    printf("Control: 2x PWM per motor\n\n");
    
    // Initialize motor drivers
    printf("Initializing motor control...\n");
    
    // Initialize DMA PID system
    pid_dma_init();
    
    // Setup motor 0 (Port A) - adjust pins for your hardware
    pid_dma_hw_config_t motor0_config = {
        .encoder_pio = NULL,      // Use default (pio0)
        .encoder_pin_a = 2,
        .encoder_pin_b = 3,
        .pwm_pio = NULL,          // Use default (pio1)
        .pwm_pin_a = 4,
        .pwm_pin_b = 5,
        .counts_per_rev = 360,
    };
    if (pid_dma_motor_setup(0, &motor0_config) == 0) {
        printf("Motor 0 (Port A): OK\n");
    } else {
        printf("Motor 0 (Port A): FAILED\n");
    }
    
    // Setup motor 1 (Port B)
    pid_dma_hw_config_t motor1_config = {
        .encoder_pio = NULL,
        .encoder_pin_a = 6,
        .encoder_pin_b = 7,
        .pwm_pio = NULL,
        .pwm_pin_a = 8,
        .pwm_pin_b = 9,
        .counts_per_rev = 360,
    };
    if (pid_dma_motor_setup(1, &motor1_config) == 0) {
        printf("Motor 1 (Port B): OK\n");
    } else {
        printf("Motor 1 (Port B): FAILED\n");
    }
    
    // Setup motor 2 (Port C)
    pid_dma_hw_config_t motor2_config = {
        .encoder_pio = NULL,
        .encoder_pin_a = 10,
        .encoder_pin_b = 11,
        .pwm_pio = NULL,
        .pwm_pin_a = 12,
        .pwm_pin_b = 13,
        .counts_per_rev = 360,
    };
    if (pid_dma_motor_setup(2, &motor2_config) == 0) {
        printf("Motor 2 (Port C): OK\n");
    } else {
        printf("Motor 2 (Port C): FAILED\n");
    }
    
    // Setup motor 3 (Port D)
    pid_dma_hw_config_t motor3_config = {
        .encoder_pio = NULL,
        .encoder_pin_a = 14,
        .encoder_pin_b = 15,
        .pwm_pio = NULL,
        .pwm_pin_a = 16,
        .pwm_pin_b = 17,
        .counts_per_rev = 360,
    };
    if (pid_dma_motor_setup(3, &motor3_config) == 0) {
        printf("Motor 3 (Port D): OK\n");
    } else {
        printf("Motor 3 (Port D): FAILED\n");
    }
    
    pbdrv_pwm_init();       // PWM for motors
    pbdrv_counter_init();   // PIO encoders with DMA
    
    // Start 1kHz timer for encoder processing
    static struct repeating_timer timer;
    add_repeating_timer_us(-1000, timer_callback, NULL, &timer);
    printf("Motor control running at 1 kHz\n\n");
    
    // Initialize CYW43
    printf("Initializing Bluetooth...\n");
    if (cyw43_arch_init()) {
        printf("ERROR: CYW43 init failed!\n");
        return 1;
    }
    
    // Initialize Bluetooth
    btstack_init();
    
    printf("\n========================================\n");
    printf("Hub ready!\n");
    printf("Connect via nRF Connect app\n");
    printf("\nMotor Test Commands (send via NUS RX):\n");
    printf("  T0, T1, T2, T3  - Test single motor\n");
    printf("  A               - Test all motors\n");
    printf("  R0, R1, R2, R3  - Speed ramp test\n");
    printf("  M0+5000         - Set motor 0 to 50%%\n");
    printf("  S               - Stop all motors\n");
    printf("  C               - Show encoder counts\n");
    printf("  P0=90           - Position: motor 0 to 90 deg\n");
    printf("  V0=180          - Velocity: motor 0 at 180 deg/s\n");
    printf("========================================\n\n");
    
    // Main loop
    uint32_t last_blink = 0;
    uint32_t last_status = 0;
    bool led_state = false;
    
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // LED blink (fast when connected)
        uint32_t blink_interval = btstack_is_connected() ? 100 : 500;
        if (now - last_blink > blink_interval) {
            led_state = !led_state;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
            last_blink = now;
        }
        
        // Send encoder status every 1 second when connected
        if (nus_is_ready() && (now - last_status > 1000)) {
            char status[128];
            int len = snprintf(status, sizeof(status),
                "Enc: M0=%ld M1=%ld M2=%ld M3=%ld\n",
                pbdrv_counter_get_count_simple(0),
                pbdrv_counter_get_count_simple(1),
                pbdrv_counter_get_count_simple(2),
                pbdrv_counter_get_count_simple(3)
            );
            nus_send_data((uint8_t*)status, len);
            last_status = now;
        }
        
        sleep_ms(10);
    }
    
    return 0;
}
