#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/timer.h"

// BTstack functions
extern void btstack_init(void);
extern bool btstack_is_connected(void);
extern bool nus_is_ready(void);
extern bool pybricks_is_ready(void);
extern void nus_send_data(const uint8_t *data, uint16_t length);
extern void pybricks_send_event(const uint8_t *data, uint16_t length);

// Motor control functions
extern void pbdrv_counter_init(void);
extern void pbdrv_counter_update(void);
extern void pbdrv_pwm_init(void);

// 1 kHz timer callback for encoder updates
static bool timer_callback(struct repeating_timer *t) {
    pbdrv_counter_update();
    return true;  // Keep repeating
}

int main() {
    stdio_init_all();
    
    printf("\n========================================\n");
    printf("  Pybricks Hub - Raspberry Pi Pico W\n");
    printf("========================================\n");
    printf("Firmware: 1.0.0\n");
    printf("Hub Type: Custom\n");
    printf("Ports: 4 (motors + encoders)\n\n");
    
    // Initialize motor drivers
    printf("Initializing motor control...\n");
    pbdrv_counter_init();  // Encoders
    pbdrv_pwm_init();      // PWM for motors
    
    // Start 1kHz timer for encoder updates
    static struct repeating_timer timer;
    add_repeating_timer_us(-1000, timer_callback, NULL, &timer);  // -1000us = 1kHz
    printf("Motor control running at 1 kHz\n\n");
    
    // Initialize CYW43
    printf("Initializing wireless...\n");
    if (cyw43_arch_init()) {
        printf("ERROR: CYW43 init failed!\n");
        return 1;
    }
    
    // Initialize Bluetooth with Pybricks protocol
    btstack_init();
    
    printf("\nHub ready!\n");
    printf("Connect via Pybricks Code: https://code.pybricks.com\n");
    printf("Or use nRF Connect to explore services\n\n");
    
    // Main loop
    uint32_t last_blink = 0;
    uint32_t last_status = 0;
    bool led_state = false;
    uint32_t status_counter = 0;
    
    while (1) {
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        // LED blinks to show connection status
        uint32_t blink_interval = btstack_is_connected() ? 100 : 500;
        
        if (now - last_blink > blink_interval) {
            led_state = !led_state;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
            last_blink = now;
        }
        
        // Send status updates every 3 seconds if connected
        if (btstack_is_connected() && (now - last_status > 3000)) {
            
            // Send via NUS if enabled
            if (nus_is_ready()) {
                char nus_msg[48];
                int len = snprintf(nus_msg, sizeof(nus_msg), 
                    "Status #%lu - Motors ready\n", status_counter);
                nus_send_data((uint8_t*)nus_msg, len);
            }
            
            // Send via Pybricks protocol if enabled
            if (pybricks_is_ready()) {
                uint8_t pybricks_status[] = {
                    0x01,  // Event type: status
                    0x00,  // Status: OK
                    (uint8_t)(status_counter & 0xFF),
                    (uint8_t)((status_counter >> 8) & 0xFF)
                };
                pybricks_send_event(pybricks_status, sizeof(pybricks_status));
            }
            
            status_counter++;
            last_status = now;
        }
        
        sleep_ms(10);
    }
    
    return 0;
}
