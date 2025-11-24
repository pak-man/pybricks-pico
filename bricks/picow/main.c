#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/i2c.h"
#include "../../lib/pbio/drv/pid_dma/pid_dma_rp2040.h"
#include "../../lib/pbio/drv/imu/imu_bno085.h"
#include "../../lib/pbio/drv/storage/storage.h"
#include "../../lib/pbio/drv/battery/battery_rp2040.h"

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

// System test functions
extern void system_test_parse_command(const char *cmd);

// Command buffer for BLE
static char cmd_buffer[64];
static uint8_t cmd_len = 0;

// IMU update counter
static uint32_t imu_update_counter = 0;

// 1 kHz timer callback for encoder updates
static bool timer_callback(struct repeating_timer *t) {
    pbdrv_counter_update();
    
    // Run DMA PID update for all enabled motors
#ifdef PICO_2W
    for (int i = 0; i < 12; i++) {
#else
    for (int i = 0; i < 4; i++) {
#endif
        pid_dma_force_update(i);
    }
    
    // Update IMU at 100 Hz (every 10ms)
    imu_update_counter++;
    if (imu_update_counter >= 10) {
        pbdrv_imu_bno085_update();
        imu_update_counter = 0;
    }
    
    return true;
}

// Process received command from BLE
void process_ble_command(const uint8_t *data, uint16_t len) {
    for (int i = 0; i < len && cmd_len < sizeof(cmd_buffer) - 1; i++) {
        if (data[i] == '\n' || data[i] == '\r') {
            if (cmd_len > 0) {
                cmd_buffer[cmd_len] = '\0';
                printf("RX: %s\n", cmd_buffer);
                
                // All commands go through system_test
                system_test_parse_command(cmd_buffer);
                
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
#ifdef PICO_2W
    printf("  Pybricks Hub - Pico 2 W\n");
#else
    printf("  Pybricks Hub - Pico W\n");
#endif
    printf("========================================\n");
    printf("Firmware: 1.0.0\n");
#ifdef PICO_2W
    printf("Motors: 12 (with PIO encoders + DMA)\n");
#else
    printf("Motors: 4 (with PIO encoders + DMA)\n");
#endif
    printf("IMU: BNO085 (9-axis sensor fusion)\n");
    printf("Control: 2x PWM per motor\n\n");
    
    // Initialize storage
    printf("Initializing storage...\n");
    pbdrv_storage_init();
    
    // Initialize motor drivers
    printf("Initializing motor control...\n");
    pid_dma_init();
    
    // Setup motors based on platform
#ifdef PICO_2W
    // Pico 2 W: 12 motors on GPIO 0-47
    const struct { uint8_t enc_a, enc_b, pwm_a, pwm_b; } motor_pins[12] = {
        {0, 1, 2, 3},       // Motor 0
        {4, 5, 6, 7},       // Motor 1
        {8, 9, 10, 11},     // Motor 2
        {12, 13, 16, 17},   // Motor 3 (skip 14,15 for IMU I2C)
        {18, 19, 20, 21},   // Motor 4
        {22, 23, 24, 25},   // Motor 5
        {26, 27, 28, 29},   // Motor 6
        {30, 31, 32, 33},   // Motor 7
        {34, 35, 36, 37},   // Motor 8
        {38, 39, 40, 41},   // Motor 9
        {42, 43, 44, 45},   // Motor 10
        {46, 47, 0, 1},     // Motor 11 (wrap to GPIO 0,1 for PWM)
    };
    for (int i = 0; i < 12; i++) {
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
            printf("Motor %d: OK\n", i);
        }
    }
#else
    // Pico W: 4 motors (skip GPIO 4,5 for IMU I2C)
    pid_dma_hw_config_t motor0_config = {
        .encoder_pio = NULL, .encoder_pin_a = 2, .encoder_pin_b = 3,
        .pwm_pio = NULL, .pwm_pin_a = 6, .pwm_pin_b = 7,
        .counts_per_rev = 360,
    };
    pid_dma_motor_setup(0, &motor0_config);
    printf("Motor 0 (Port A): OK\n");
    
    pid_dma_hw_config_t motor1_config = {
        .encoder_pio = NULL, .encoder_pin_a = 8, .encoder_pin_b = 9,
        .pwm_pio = NULL, .pwm_pin_a = 10, .pwm_pin_b = 11,
        .counts_per_rev = 360,
    };
    pid_dma_motor_setup(1, &motor1_config);
    printf("Motor 1 (Port B): OK\n");
    
    pid_dma_hw_config_t motor2_config = {
        .encoder_pio = NULL, .encoder_pin_a = 12, .encoder_pin_b = 13,
        .pwm_pio = NULL, .pwm_pin_a = 14, .pwm_pin_b = 15,
        .counts_per_rev = 360,
    };
    pid_dma_motor_setup(2, &motor2_config);
    printf("Motor 2 (Port C): OK\n");
    
    pid_dma_hw_config_t motor3_config = {
        .encoder_pio = NULL, .encoder_pin_a = 16, .encoder_pin_b = 17,
        .pwm_pio = NULL, .pwm_pin_a = 18, .pwm_pin_b = 19,
        .counts_per_rev = 360,
    };
    pid_dma_motor_setup(3, &motor3_config);
    printf("Motor 3 (Port D): OK\n");
#endif
    
    pbdrv_pwm_init();
    pbdrv_counter_init();
    
    // Initialize IMU
    printf("\nInitializing IMU (BNO085)...\n");
#ifdef PICO_2W
    if (pbdrv_imu_bno085_init(i2c1, 14, 15, BNO085_I2C_ADDR_DEFAULT) == 0) {
        printf("IMU: OK (I2C1 @ 0x4A)\n");
    } else {
        printf("IMU: FAILED\n");
    }
#else
    if (pbdrv_imu_bno085_init(i2c0, 4, 5, BNO085_I2C_ADDR_DEFAULT) == 0) {
        printf("IMU: OK (I2C0 @ 0x4A)\n");
    } else {
        printf("IMU: FAILED\n");
    }
#endif
    
    // Start 1kHz timer
    static struct repeating_timer timer;
    add_repeating_timer_us(-1000, timer_callback, NULL, &timer);
    printf("Motor control + IMU running at 1 kHz / 100 Hz\n\n");
    
    // Initialize CYW43 + Bluetooth
    printf("Initializing Bluetooth...\n");
    if (cyw43_arch_init()) {
        printf("ERROR: CYW43 init failed!\n");
        return 1;
    }
    btstack_init();
    
    printf("\n========================================\n");
    printf("Hub ready!\n");
    printf("Connect via nRF Connect app\n");
    printf("\nCommands (send via NUS RX):\n");
    printf("  T0-T%d  - Test single motor\n", NUM_MOTORS - 1);
    printf("  A       - Test all motors\n");
    printf("  IMU     - Show IMU data\n");
    printf("  IMUCAL  - Save IMU calibration\n");
    printf("  M0+5000 - Set motor 0 to 50%%\n");
    printf("  S       - Stop all motors\n");
    printf("========================================\n\n");
    
    // Main loop
    uint32_t last_blink = 0;
    uint32_t last_status = 0;
    bool led_state = false;
    
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // LED blink
        uint32_t blink_interval = btstack_is_connected() ? 100 : 500;
        if (now - last_blink > blink_interval) {
            led_state = !led_state;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
            last_blink = now;
        }
        
        // Send status every 1 second
        if (nus_is_ready() && (now - last_status > 1000)) {
            char status[256];
            bno085_euler_t heading = pbdrv_imu_bno085_get_heading();
            
#ifdef PICO_2W
            int len = snprintf(status, sizeof(status),
                "Status: Motors=%d Heading=%.1f° Roll=%.1f° Pitch=%.1f°\n",
                12, heading.yaw, heading.roll, heading.pitch);
#else
            int len = snprintf(status, sizeof(status),
                "Enc: M0=%ld M1=%ld M2=%ld M3=%ld | Heading=%.1f°\n",
                pbdrv_counter_get_count_simple(0),
                pbdrv_counter_get_count_simple(1),
                pbdrv_counter_get_count_simple(2),
                pbdrv_counter_get_count_simple(3),
                heading.yaw);
#endif
            nus_send_data((uint8_t*)status, len);
            last_status = now;
        }
        
        sleep_ms(10);
    }
    
    return 0;
}