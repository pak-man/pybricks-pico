#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "btstack.h"
#include "lwip/apps/httpd.h"
#include "lwip/apps/sntp.h"
#ifdef ENABLE_OTA
#include "ota_handler.h"
#endif

// WiFi credentials - CHANGE THESE!
#define WIFI_SSID "YOUR_SSID"
#define WIFI_PASSWORD "YOUR_PASSWORD"

// OTA upload state
static uint8_t *ota_upload_buffer = NULL;
static size_t ota_upload_size = 0;
static size_t ota_upload_received = 0;

// CGI handler for motor control
const char *motor_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
    int motor = -1;
    int duty = 0;
    
    for (int i = 0; i < iNumParams; i++) {
        if (strcmp(pcParam[i], "motor") == 0) {
            motor = atoi(pcValue[i]);
        } else if (strcmp(pcParam[i], "duty") == 0) {
            duty = atoi(pcValue[i]);
        }
    }
    
    if (motor >= 0 && motor < 4) {
        printf("Motor %d set to %d\n", motor, duty);
        // TODO: Call actual motor control function
    }
    
    return "/index.shtml";
}

static const tCGI motor_cgi = {"/motor", motor_cgi_handler};
static tCGI cgi_handlers[] = {motor_cgi};

// SSI handler for dynamic content
u16_t ssi_handler(int iIndex, char *pcInsert, int iInsertLen) {
    size_t printed;
    
    switch (iIndex) {
        case 0: // <!--#status-->
            printed = snprintf(pcInsert, iInsertLen, 
                "WiFi: %s<br>IP: %s<br>BLE: %s",
                cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA) == CYW43_LINK_UP ? "Connected" : "Disconnected",
                ip4addr_ntoa(netif_ip4_addr(netif_list)),
                "Active"
            );
            return (u16_t)printed;
            
        case 1: // <!--#imu-->
            printed = snprintf(pcInsert, iInsertLen,
                "Accel: X:0.0 Y:0.0 Z:0.0<br>Gyro: X:0.0 Y:0.0 Z:0.0"
            );
            return (u16_t)printed;
            
        case 2: // <!--#version-->
            #ifdef ENABLE_OTA
            printed = snprintf(pcInsert, iInsertLen, "%s", ota_get_version());
            if (ota_is_after_update()) {
                printed += snprintf(pcInsert + printed, iInsertLen - printed, " (Just updated!)");
            }
            if (ota_is_after_rollback()) {
                printed += snprintf(pcInsert + printed, iInsertLen - printed, " (Rolled back)");
            }
            #else
            printed = snprintf(pcInsert, iInsertLen, "1.0.0");
            #endif
            return (u16_t)printed;
            
        default:
            return 0;
    }
}

static const char *ssi_tags[] = {"status", "imu", "version"};

// HTTP POST handler for OTA upload
#ifdef ENABLE_OTA
err_t httpd_post_begin(void *connection, const char *uri, const char *http_request,
                      u16_t http_request_len, int content_len, char *response_uri,
                      u16_t response_uri_len, u8_t *post_auto_wnd) {
    
    if (strcmp(uri, "/upload") == 0) {
        printf("[OTA] Upload started, size: %d bytes\n", content_len);
        
        if (ota_begin() != 0) {
            printf("[OTA] Failed to initialize\n");
            return ERR_VAL;
        }
        
        ota_upload_size = content_len;
        ota_upload_received = 0;
        return ERR_OK;
    }
    
    return ERR_VAL;
}

err_t httpd_post_receive_data(void *connection, struct pbuf *p) {
    struct pbuf *q = p;
    
    while (q != NULL) {
        if (ota_write((uint8_t*)q->payload, q->len) != 0) {
            printf("[OTA] Write failed\n");
            ota_abort();
            return ERR_VAL;
        }
        
        ota_upload_received += q->len;
        
        if (ota_upload_received % 10240 == 0) {
            printf("[OTA] Progress: %zu/%zu bytes\n", ota_upload_received, ota_upload_size);
        }
        
        q = q->next;
    }
    
    pbuf_free(p);
    return ERR_OK;
}

void httpd_post_finished(void *connection, char *response_uri, u16_t response_uri_len) {
    printf("[OTA] Upload finished, total: %zu bytes\n", ota_upload_received);
    
    if (ota_end() != 0) {
        snprintf(response_uri, response_uri_len, "/index.shtml?error=verify_failed");
        ota_abort();
        return;
    }
    
    snprintf(response_uri, response_uri_len, "/index.shtml");
    
    // Schedule update in 3 seconds
    printf("[OTA] Scheduling update...\n");
    sleep_ms(3000);
    ota_perform_update();
}
#endif

int main() {
    stdio_init_all();
    printf("Pybricks Pico W starting...\n");
    
    #ifdef ENABLE_OTA
    printf("Firmware version: %s\n", ota_get_version());
    
    // Check OTA status
    if (ota_is_after_update()) {
        printf("[OTA] Just updated! Committing firmware...\n");
        ota_commit_firmware();
    }
    
    if (ota_is_after_rollback()) {
        printf("[OTA] WARNING: Previous firmware failed, rolled back!\n");
    }
    #else
    printf("Firmware version: 1.0.0 (OTA disabled)\n");
    #endif
    
    // Initialize WiFi
    if (cyw43_arch_init()) {
        printf("Failed to initialize cyw43\n");
        return 1;
    }
    
    cyw43_arch_enable_sta_mode();
    
    printf("Connecting to WiFi...\n");
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, 
                                           CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect to WiFi\n");
        return 1;
    }
    
    printf("Connected! IP: %s\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    
    // Initialize HTTP server
    httpd_init();
    http_set_cgi_handlers(cgi_handlers, LWIP_ARRAYSIZE(cgi_handlers));
    http_set_ssi_handler(ssi_handler, ssi_tags, LWIP_ARRAYSIZE(ssi_tags));
    
    printf("HTTP server running\n");
    printf("Access web interface at: http://%s/\n", ip4addr_ntoa(netif_ip4_addr(netif_list)));
    
    // Commit firmware after successful boot (prevents rollback)
    #ifdef ENABLE_OTA
    sleep_ms(5000);
    if (ota_is_after_update()) {
        ota_commit_firmware();
        printf("[OTA] Firmware committed successfully\n");
    }
    #endif
    
    // Main loop
    uint32_t last_blink = 0;
    bool led_on = false;
    
    while (1) {
        #ifdef WIFI_MODE_POLL
        cyw43_arch_poll();
        #endif
        
        // Blink LED
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_blink > 1000) {
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
            led_on = !led_on;
            last_blink = now;
        }
        
        sleep_ms(10);
    }
    
    cyw43_arch_deinit();
    return 0;
}
