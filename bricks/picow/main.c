#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/i2c.h"
#include "lwip/apps/httpd.h"
#include "lwip/ip4_addr.h"
#include "../../lib/pbio/drv/pid_dma/pid_dma_rp2040.h"
#include "../../lib/pbio/drv/imu/imu_bno085.h"
#include "../../lib/pbio/drv/storage/storage.h"
#include "../../lib/pbio/drv/battery/battery_rp2040.h"

// WiFi credentials - CHANGE THESE!
#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASSWORD"

// BTstack functions
extern void btstack_init(void);
extern bool btstack_is_connected(void);
extern bool nus_is_ready(void);
extern void nus_send_data(const uint8_t *data, uint16_t length);

// Motor control
extern void pbdrv_counter_init(void);
extern void pbdrv_counter_update(void);
extern void pbdrv_pwm_init(void);
extern int32_t pbdrv_counter_get_count_simple(uint8_t id);
extern void system_test_parse_command(const char *cmd);

// State
static char cmd_buffer[64];
static uint8_t cmd_len = 0;
static uint32_t imu_update_counter = 0;
static bool wifi_connected = false;

// 1 kHz timer callback
static bool timer_callback(struct repeating_timer *t) {
    pbdrv_counter_update();
    
#ifdef PICO_2W
    for (int i = 0; i < 12; i++)
#else
    for (int i = 0; i < 4; i++)
#endif
        pid_dma_force_update(i);
    
    if (++imu_update_counter >= 10) {
        pbdrv_imu_bno085_update();
        imu_update_counter = 0;
    }
    
    return true;
}

void process_ble_command(const uint8_t *data, uint16_t len) {
    for (int i = 0; i < len && cmd_len < sizeof(cmd_buffer) - 1; i++) {
        if (data[i] == '\n' || data[i] == '\r') {
            if (cmd_len > 0) {
                cmd_buffer[cmd_len] = '\0';
                printf("RX: %s\n", cmd_buffer);
                system_test_parse_command(cmd_buffer);
                cmd_len = 0;
            }
        } else {
            cmd_buffer[cmd_len++] = data[i];
        }
    }
}

static void setup_motors(void) {
    printf("Initializing motor control...\n");
    pid_dma_init();
    
#ifdef PICO_2W
    const struct { uint8_t enc_a, enc_b, pwm_a, pwm_b; } motor_pins[12] = {
        {0,1,2,3}, {4,5,6,7}, {8,9,10,11}, {12,13,16,17},
        {18,19,20,21}, {22,23,24,25}, {26,27,28,29}, {30,31,32,33},
        {34,35,36,37}, {38,39,40,41}, {42,43,44,45}, {46,47,0,1}
    };
    for (int i = 0; i < 12; i++) {
#else
    const struct { uint8_t enc_a, enc_b, pwm_a, pwm_b; } motor_pins[4] = {
        {2,3,6,7}, {8,9,10,11}, {12,13,14,15}, {16,17,18,19}
    };
    for (int i = 0; i < 4; i++) {
#endif
        pid_dma_hw_config_t config = {
            .encoder_pio = NULL, .encoder_pin_a = motor_pins[i].enc_a,
            .encoder_pin_b = motor_pins[i].enc_b, .pwm_pio = NULL,
            .pwm_pin_a = motor_pins[i].pwm_a, .pwm_pin_b = motor_pins[i].pwm_b,
            .counts_per_rev = 360,
        };
        if (pid_dma_motor_setup(i, &config) == 0)
            printf("Motor %d: OK\n", i);
    }
    
    pbdrv_pwm_init();
    pbdrv_counter_init();
}

// ============================================================================
// HTTP Server Handlers
// ============================================================================

static const char *cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
    if (strcmp(pcParam[0], "motor") == 0) {
        int motor = atoi(pcValue[0]);
        if (iNumParams > 1 && strcmp(pcParam[1], "duty") == 0) {
            int duty = atoi(pcValue[1]);
            extern void pbdrv_pwm_set_duty_simple(uint8_t id, int16_t duty);
            pbdrv_pwm_set_duty_simple(motor, duty);
        }
        return "/index.shtml";
    }
    return "/index.shtml";
}

static const tCGI cgi_handlers[] = {
    {"/motor", cgi_handler},
};

static u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen) {
    size_t printed = 0;
    
    switch (iIndex) {
        case 0: // <!--#status-->
            printed = snprintf(pcInsert, iInsertLen, 
                "WiFi: %s<br>BLE: %s<br>Motors: %d",
                wifi_connected ? "Connected" : "Disconnected",
                btstack_is_connected() ? "Connected" : "Advertising",
                NUM_MOTORS);
            break;
            
        case 1: { // <!--#imu-->
            bno085_euler_t h = pbdrv_imu_bno085_get_heading();
            printed = snprintf(pcInsert, iInsertLen,
                "Heading: %.1f°<br>Roll: %.1f°<br>Pitch: %.1f°",
                h.yaw, h.roll, h.pitch);
            break;
        }
    }
    
    return (u16_t)printed;
}

static const char *ssi_tags[] = {"status", "imu"};

int main() {
    stdio_init_all();
    
    printf("\n========================================\n");
#ifdef PICO_2W
    printf("  Pybricks Hub - Pico 2 W\n");
    printf("  Motors: 12 (PIO + DMA)\n");
#else
    printf("  Pybricks Hub - Pico W\n");
    printf("  Motors: 4 (PIO + DMA)\n");
#endif
    
#ifdef WIFI_MODE_POLL
    printf("  WiFi: Polling mode\n");
#elif defined(WIFI_MODE_THREADSAFE_BACKGROUND)
    printf("  WiFi: Threadsafe background\n");
#elif defined(WIFI_MODE_SYS)
    printf("  WiFi: FreeRTOS\n");
#endif
    
    printf("========================================\n\n");
    
    pbdrv_storage_init();
    setup_motors();
    
    printf("\nInitializing IMU (BNO085)...\n");
#ifdef PICO_2W
    if (pbdrv_imu_bno085_init(i2c1, 14, 15, BNO085_I2C_ADDR_DEFAULT) == 0)
#else
    if (pbdrv_imu_bno085_init(i2c0, 4, 5, BNO085_I2C_ADDR_DEFAULT) == 0)
#endif
        printf("IMU: OK\n");
    else
        printf("IMU: FAILED\n");
    
    static struct repeating_timer timer;
    add_repeating_timer_us(-1000, timer_callback, NULL, &timer);
    printf("Motor control + IMU: 1kHz / 100Hz\n\n");
    
    // Initialize WiFi + Bluetooth
    printf("Initializing WiFi + Bluetooth...\n");
    if (cyw43_arch_init()) {
        printf("ERROR: CYW43 init failed!\n");
        return 1;
    }
    
    cyw43_arch_enable_sta_mode();
    printf("WiFi: Station mode enabled\n");
    
    printf("Connecting to '%s'...\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, 
                                           CYW43_AUTH_WPA2_AES_PSK, 30000) == 0) {
        wifi_connected = true;
        printf("WiFi: Connected!\n");
        printf("IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
        
        httpd_init();
        http_set_cgi_handlers(cgi_handlers, LWIP_ARRAYSIZE(cgi_handlers));
        http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
        printf("HTTP: Server on port 80\n");
    } else {
        wifi_connected = false;
        printf("WiFi: Failed (continuing)\n");
    }
    
    btstack_init();
    printf("Bluetooth: Ready\n");
    
    printf("\n========================================\n");
    printf("Hub ready!\n");
    if (wifi_connected)
        printf("Web: http://%s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    printf("BLE: nRF Connect\n");
    printf("========================================\n\n");
    
    // Main loop
    uint32_t last_blink = 0, last_status = 0;
    bool led_state = false;
    
    while (1) {
#ifdef WIFI_MODE_POLL
        // CRITICAL: Must poll in POLL mode
        cyw43_arch_poll();
#endif
        
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        uint32_t blink = wifi_connected ? 1000 : (btstack_is_connected() ? 100 : 500);
        if (now - last_blink > blink) {
            led_state = !led_state;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
            last_blink = now;
        }
        
        if (nus_is_ready() && (now - last_status > 1000)) {
            char status[256];
            bno085_euler_t h = pbdrv_imu_bno085_get_heading();
            
#ifdef PICO_2W
            int len = snprintf(status, sizeof(status),
                "Motors=12 Heading=%.1f° Roll=%.1f° Pitch=%.1f° WiFi=%s\n",
                h.yaw, h.roll, h.pitch, wifi_connected ? "OK" : "OFF");
#else
            int len = snprintf(status, sizeof(status),
                "M0=%ld M1=%ld M2=%ld M3=%ld Heading=%.1f° WiFi=%s\n",
                pbdrv_counter_get_count_simple(0), pbdrv_counter_get_count_simple(1),
                pbdrv_counter_get_count_simple(2), pbdrv_counter_get_count_simple(3),
                h.yaw, wifi_connected ? "OK" : "OFF");
#endif
            nus_send_data((uint8_t*)status, len);
            last_status = now;
        }
        
#ifdef WIFI_MODE_POLL
        sleep_ms(1);
#else
        sleep_ms(10);
#endif
    }
    
    return 0;
}
