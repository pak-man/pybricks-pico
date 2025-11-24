#include <stdio.h>
#include <string.h>
#include "pico/cyw43_arch.h"
#include "btstack.h"

// External command processor
extern void process_ble_command(const uint8_t *data, uint16_t len);

// BLE connection state
static hci_con_handle_t con_handle = HCI_CON_HANDLE_INVALID;
static bool nus_ready = false;

// Nordic UART Service (NUS) UUIDs
static const uint8_t nus_service_uuid[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E};
static const uint8_t nus_rx_uuid[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E};
static const uint8_t nus_tx_uuid[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E};

static uint16_t nus_rx_handle = 0;
static uint16_t nus_tx_handle = 0;
static uint16_t nus_tx_ccc_handle = 0;

// TX buffer for queued notifications
#define TX_BUFFER_SIZE 512
static uint8_t tx_buffer[TX_BUFFER_SIZE];
static uint16_t tx_buffer_len = 0;
static bool tx_notification_enabled = false;

// GATT service definition
static uint8_t adv_data[] = {
    0x02, 0x01, 0x06,  // Flags: General discoverable, BR/EDR not supported
    0x11, 0x07,        // 128-bit service UUID list
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
};

static uint8_t scan_resp_data[] = {
    0x0D, 0x09, 'P', 'y', 'b', 'r', 'i', 'c', 'k', 's', ' ', 'H', 'u', 'b',
};

// Check if connected
bool btstack_is_connected(void) {
    return con_handle != HCI_CON_HANDLE_INVALID;
}

// Check if NUS is ready for TX
bool nus_is_ready(void) {
    return btstack_is_connected() && tx_notification_enabled;
}

// Send data via NUS TX characteristic
void nus_send_data(const uint8_t *data, uint16_t length) {
    if (!nus_is_ready() || length == 0) return;
    
    // Queue data in buffer if space available
    if (tx_buffer_len + length <= TX_BUFFER_SIZE) {
        memcpy(tx_buffer + tx_buffer_len, data, length);
        tx_buffer_len += length;
    }
    
    // Try to send immediately if possible
    if (tx_buffer_len > 0 && att_server_can_send_packet_now(con_handle)) {
        uint16_t chunk_size = tx_buffer_len > 20 ? 20 : tx_buffer_len;
        att_server_notify(con_handle, nus_tx_handle, tx_buffer, chunk_size);
        
        // Remove sent data from buffer
        if (chunk_size < tx_buffer_len) {
            memmove(tx_buffer, tx_buffer + chunk_size, tx_buffer_len - chunk_size);
        }
        tx_buffer_len -= chunk_size;
    }
}

// ATT packet handler
static void att_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);
    
    if (packet_type != HCI_EVENT_PACKET) return;
    
    switch (hci_event_packet_get_type(packet)) {
        case ATT_EVENT_CONNECTED:
            printf("BLE: Device connected\n");
            break;
            
        case ATT_EVENT_DISCONNECTED:
            printf("BLE: Device disconnected\n");
            con_handle = HCI_CON_HANDLE_INVALID;
            tx_notification_enabled = false;
            tx_buffer_len = 0;
            break;
            
        case ATT_EVENT_CAN_SEND_NOW:
            // Try to send more queued data
            if (tx_buffer_len > 0 && tx_notification_enabled) {
                uint16_t chunk_size = tx_buffer_len > 20 ? 20 : tx_buffer_len;
                att_server_notify(con_handle, nus_tx_handle, tx_buffer, chunk_size);
                
                if (chunk_size < tx_buffer_len) {
                    memmove(tx_buffer, tx_buffer + chunk_size, tx_buffer_len - chunk_size);
                }
                tx_buffer_len -= chunk_size;
                
                // Request another send if more data queued
                if (tx_buffer_len > 0) {
                    att_server_request_can_send_now_event(con_handle);
                }
            }
            break;
    }
}

// ATT read/write handler
static uint16_t att_read_callback(hci_con_handle_t connection_handle, uint16_t att_handle, 
                                   uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(connection_handle);
    UNUSED(offset);
    UNUSED(buffer);
    UNUSED(buffer_size);
    UNUSED(att_handle);
    return 0;
}

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, 
                              uint16_t transaction_mode, uint16_t offset, 
                              uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    
    // Handle NUS RX characteristic write (data from client)
    if (att_handle == nus_rx_handle) {
        printf("BLE RX: %d bytes\n", buffer_size);
        process_ble_command(buffer, buffer_size);
        return 0;
    }
    
    // Handle TX CCC (notification enable/disable)
    if (att_handle == nus_tx_ccc_handle) {
        tx_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
        printf("BLE: TX notifications %s\n", tx_notification_enabled ? "enabled" : "disabled");
        con_handle = connection_handle;
        return 0;
    }
    
    return 0;
}

// HCI packet handler
static void hci_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);
    
    if (packet_type != HCI_EVENT_PACKET) return;
    
    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                printf("BLE: Stack ready, starting advertising\n");
                gap_advertisements_set_data(sizeof(adv_data), adv_data);
                gap_scan_response_set_data(sizeof(scan_resp_data), scan_resp_data);
                gap_advertisements_enable(1);
            }
            break;
            
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("BLE: Connection closed, restarting advertising\n");
            gap_advertisements_enable(1);
            break;
    }
}

// Initialize BTstack
void btstack_init(void) {
    printf("Initializing BTstack...\n");
    
    // Setup BTstack memory pools
    l2cap_init();
    sm_init();
    
    // Setup ATT server
    att_server_init(NULL, att_read_callback, att_write_callback);
    
    // Register for HCI events
    static btstack_packet_callback_registration_t hci_event_callback_registration;
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    
    // Register for ATT events
    att_server_register_packet_handler(att_packet_handler);
    
    // Setup GATT database (simplified - in real implementation would use .gatt file)
    // This is a placeholder - you'd normally generate this from a .gatt file
    printf("BLE: GATT database initialized\n");
    
    // For now, use hardcoded handles (should match your .gatt file)
    nus_rx_handle = 0x0003;  // Handle for NUS RX characteristic
    nus_tx_handle = 0x0005;  // Handle for NUS TX characteristic
    nus_tx_ccc_handle = 0x0006;  // Handle for TX CCC descriptor
    
    // Turn on Bluetooth
    hci_power_control(HCI_POWER_ON);
    
    printf("BLE: Initialization complete\n");
}