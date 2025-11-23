// SPDX-License-Identifier: MIT
// BTstack with Pybricks Protocol for Pico W

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"

#include "btstack.h"

// Device identification
#define DEVICE_NAME "Pybricks Pico W"
#define MANUFACTURER_NAME "Pybricks"
#define FW_VERSION "1.0.0"
#define HW_VERSION "Pico W"

// Pybricks Service UUID: c5f50001-8280-46da-89f4-6d8051e4aeef
static const uint8_t pybricks_service_uuid[] = {
    0xef, 0xae, 0xe4, 0x51, 0x80, 0x6d, 0xf4, 0x89,
    0xda, 0x46, 0x80, 0x82, 0x01, 0x00, 0xf5, 0xc5
};

// Pybricks Command/Event characteristic: c5f50002-8280-46da-89f4-6d8051e4aeef
static const uint8_t pybricks_command_event_uuid[] = {
    0xef, 0xae, 0xe4, 0x51, 0x80, 0x6d, 0xf4, 0x89,
    0xda, 0x46, 0x80, 0x82, 0x02, 0x00, 0xf5, 0xc5
};

// Pybricks Hub Capabilities characteristic: c5f50003-8280-46da-89f4-6d8051e4aeef
static const uint8_t pybricks_capabilities_uuid[] = {
    0xef, 0xae, 0xe4, 0x51, 0x80, 0x6d, 0xf4, 0x89,
    0xda, 0x46, 0x80, 0x82, 0x03, 0x00, 0xf5, 0xc5
};

// Nordic UART Service UUID: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const uint8_t nus_service_uuid[] = {
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E
};

// Connection state
static hci_con_handle_t connection_handle = HCI_CON_HANDLE_INVALID;
static uint16_t nus_tx_handle = 0;
static uint16_t nus_tx_cccd_handle = 0;
static uint16_t pybricks_cmd_handle = 0;
static uint16_t pybricks_cmd_cccd_handle = 0;
static bool nus_tx_enabled = false;
static bool pybricks_notifications_enabled = false;

// Hub capabilities - customize based on your hardware
static const uint8_t hub_capabilities[] = {
    0x00,  // Max command size LSB
    0x01,  // Max command size MSB (256 bytes)
    0x00,  // Max response size LSB  
    0x01,  // Max response size MSB (256 bytes)
    0x01,  // Hub type: Custom/Generic hub
    0x04,  // Number of ports
    0x00,  // Flags
};

// Forward declarations
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static uint16_t att_read_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);
static int att_write_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size);

// ATT database with complete Pybricks services
const uint8_t profile_data[] = {
    // ATT DB Version
    1,
    
    // ========== GAP Service ==========
    // 0x0001 PRIMARY_SERVICE-GAP_SERVICE
    0x0a, 0x00, 0x02, 0x00, 0x01, 0x00, 0x00, 0x28, 0x00, 0x18,
    
    // 0x0002 CHARACTERISTIC-GAP_DEVICE_NAME - READ
    0x0d, 0x00, 0x02, 0x00, 0x02, 0x00, 0x03, 0x28, 0x02, 0x03, 0x00, 0x00, 0x2a,
    
    // 0x0003 VALUE-GAP_DEVICE_NAME - READ
    0x1e, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00, 0x2a, 
    'P', 'y', 'b', 'r', 'i', 'c', 'k', 's', ' ', 'P', 'i', 'c', 'o', ' ', 'W',
    
    // ========== Device Information Service ==========
    // 0x0004 PRIMARY_SERVICE-DEVICE_INFORMATION
    0x0a, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00, 0x28, 0x0a, 0x18,
    
    // 0x0005 CHARACTERISTIC-MANUFACTURER_NAME - READ
    0x0d, 0x00, 0x02, 0x00, 0x05, 0x00, 0x03, 0x28, 0x02, 0x06, 0x00, 0x29, 0x2a,
    
    // 0x0006 VALUE-MANUFACTURER_NAME
    0x17, 0x00, 0x02, 0x00, 0x06, 0x00, 0x29, 0x2a, 
    'P', 'y', 'b', 'r', 'i', 'c', 'k', 's',
    
    // 0x0007 CHARACTERISTIC-FIRMWARE_VERSION - READ
    0x0d, 0x00, 0x02, 0x00, 0x07, 0x00, 0x03, 0x28, 0x02, 0x08, 0x00, 0x26, 0x2a,
    
    // 0x0008 VALUE-FIRMWARE_VERSION
    0x13, 0x00, 0x02, 0x00, 0x08, 0x00, 0x26, 0x2a,
    '1', '.', '0', '.', '0',
    
    // ========== Pybricks Service ==========
    // 0x0009 PRIMARY_SERVICE-PYBRICKS_SERVICE (128-bit)
    0x18, 0x00, 0x02, 0x00, 0x09, 0x00, 0x00, 0x28,
    0xef, 0xae, 0xe4, 0x51, 0x80, 0x6d, 0xf4, 0x89,
    0xda, 0x46, 0x80, 0x82, 0x01, 0x00, 0xf5, 0xc5,
    
    // 0x000A CHARACTERISTIC-PYBRICKS_COMMAND_EVENT (write/notify)
    0x1b, 0x00, 0x02, 0x00, 0x0a, 0x00, 0x03, 0x28, 0x14, 0x0b, 0x00,
    0xef, 0xae, 0xe4, 0x51, 0x80, 0x6d, 0xf4, 0x89,
    0xda, 0x46, 0x80, 0x82, 0x02, 0x00, 0xf5, 0xc5,
    
    // 0x000B VALUE-PYBRICKS_COMMAND_EVENT
    0x16, 0x00, 0x14, 0x00, 0x0b, 0x00,
    0xef, 0xae, 0xe4, 0x51, 0x80, 0x6d, 0xf4, 0x89,
    0xda, 0x46, 0x80, 0x82, 0x02, 0x00, 0xf5, 0xc5,
    
    // 0x000C CLIENT_CHARACTERISTIC_CONFIGURATION
    0x0a, 0x00, 0x0a, 0x03, 0x0c, 0x00, 0x02, 0x29, 0x00, 0x00,
    
    // 0x000D CHARACTERISTIC-PYBRICKS_CAPABILITIES (read)
    0x1b, 0x00, 0x02, 0x00, 0x0d, 0x00, 0x03, 0x28, 0x02, 0x0e, 0x00,
    0xef, 0xae, 0xe4, 0x51, 0x80, 0x6d, 0xf4, 0x89,
    0xda, 0x46, 0x80, 0x82, 0x03, 0x00, 0xf5, 0xc5,
    
    // 0x000E VALUE-PYBRICKS_CAPABILITIES
    0x16, 0x00, 0x02, 0x00, 0x0e, 0x00,
    0xef, 0xae, 0xe4, 0x51, 0x80, 0x6d, 0xf4, 0x89,
    0xda, 0x46, 0x80, 0x82, 0x03, 0x00, 0xf5, 0xc5,
    
    // ========== Nordic UART Service ==========
    // 0x000F PRIMARY_SERVICE-NUS_SERVICE (128-bit)
    0x18, 0x00, 0x02, 0x00, 0x0f, 0x00, 0x00, 0x28,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E,
    
    // 0x0010 CHARACTERISTIC-NUS_RX (write without response)
    0x1b, 0x00, 0x02, 0x00, 0x10, 0x00, 0x03, 0x28, 0x04, 0x11, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E,
    
    // 0x0011 VALUE-NUS_RX - WRITE
    0x16, 0x00, 0x04, 0x00, 0x11, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E,
    
    // 0x0012 CHARACTERISTIC-NUS_TX (notify)
    0x1b, 0x00, 0x02, 0x00, 0x12, 0x00, 0x03, 0x28, 0x10, 0x13, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E,
    
    // 0x0013 VALUE-NUS_TX - READ | NOTIFY
    0x16, 0x00, 0x12, 0x00, 0x13, 0x00,
    0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
    0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E,
    
    // 0x0014 CLIENT_CHARACTERISTIC_CONFIGURATION for NUS_TX
    0x0a, 0x00, 0x0a, 0x03, 0x14, 0x00, 0x02, 0x29, 0x00, 0x00,
    
    // END
    0x00, 0x00,
};

// Send data via NUS TX
void nus_send_data(const uint8_t *data, uint16_t length) {
    if (connection_handle == HCI_CON_HANDLE_INVALID || !nus_tx_enabled) {
        return;
    }
    att_server_notify(connection_handle, nus_tx_handle, data, length);
}

// Send Pybricks event
void pybricks_send_event(const uint8_t *data, uint16_t length) {
    if (connection_handle == HCI_CON_HANDLE_INVALID || !pybricks_notifications_enabled) {
        return;
    }
    att_server_notify(connection_handle, pybricks_cmd_handle, data, length);
    printf("Pybricks event sent: %d bytes\n", length);
}

// Initialize Bluetooth
void btstack_init(void) {
    printf("Initializing BTstack with Pybricks protocol...\n");
    
    // Store characteristic handles
    pybricks_cmd_handle = 0x000B;
    pybricks_cmd_cccd_handle = 0x000C;
    nus_tx_handle = 0x0013;
    nus_tx_cccd_handle = 0x0014;
    
    // Initialize L2CAP
    l2cap_init();
    
    // Initialize LE Security Manager
    sm_init();
    
    // Initialize ATT Server
    att_server_init(profile_data, att_read_callback, att_write_callback);
    
    // Setup advertisements
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    
    // Advertisement data with Pybricks service UUID
    const uint8_t adv_data[] = {
        0x02, 0x01, 0x06,  // Flags
        0x11, 0x06,        // 128-bit Service UUID
        0xef, 0xae, 0xe4, 0x51, 0x80, 0x6d, 0xf4, 0x89,
        0xda, 0x46, 0x80, 0x82, 0x01, 0x00, 0xf5, 0xc5  // Pybricks UUID
    };
    gap_advertisements_set_data(sizeof(adv_data), (uint8_t*)adv_data);
    
    // Scan response with device name
    uint8_t scan_resp_data[31];
    scan_resp_data[0] = strlen(DEVICE_NAME) + 1;
    scan_resp_data[1] = 0x09;
    memcpy(&scan_resp_data[2], DEVICE_NAME, strlen(DEVICE_NAME));
    gap_scan_response_set_data(strlen(DEVICE_NAME) + 2, scan_resp_data);
    
    // Register for events
    static btstack_packet_callback_registration_t hci_event_callback_registration;
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
    att_server_register_packet_handler(packet_handler);
    
    // Power on
    hci_power_control(HCI_POWER_ON);
    
    printf("Pybricks protocol initialized\n");
}

// ATT read callback
static uint16_t att_read_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(con_handle);
    
    // Pybricks capabilities
    if (att_handle == 0x000E) {
        uint16_t len = sizeof(hub_capabilities);
        if (offset >= len) return 0;
        uint16_t bytes_to_copy = len - offset;
        if (bytes_to_copy > buffer_size) bytes_to_copy = buffer_size;
        memcpy(buffer, hub_capabilities + offset, bytes_to_copy);
        return bytes_to_copy;
    }
    
    return 0;
}

// ATT write callback
static int att_write_callback(hci_con_handle_t con_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size) {
    UNUSED(transaction_mode);
    UNUSED(offset);
    
    // Pybricks command/event CCCD
    if (att_handle == pybricks_cmd_cccd_handle) {
        pybricks_notifications_enabled = (buffer[0] == 0x01);
        printf("Pybricks notifications %s\n", pybricks_notifications_enabled ? "enabled" : "disabled");
        return 0;
    }
    
    // Pybricks command
    if (att_handle == pybricks_cmd_handle) {
        printf("Pybricks command: %d bytes\n", buffer_size);
        // TODO: Process Pybricks protocol commands
        return 0;
    }
    
    // NUS TX CCCD
    if (att_handle == nus_tx_cccd_handle) {
        nus_tx_enabled = (buffer[0] == 0x01);
        printf("NUS TX %s\n", nus_tx_enabled ? "enabled" : "disabled");
        return 0;
    }
    
    // NUS RX (program data)
    if (att_handle == 0x0011) {
        printf("NUS RX: %d bytes\n", buffer_size);
	
	// Process as motor command
    	extern void process_ble_command(const uint8_t *data, uint16_t len);
    	process_ble_command(buffer, buffer_size);        
	
	return 0;
    }
    
    return 0;
}

// Packet handler
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);
    
    if (packet_type != HCI_EVENT_PACKET) return;
    
    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                printf("Bluetooth ready - advertising as Pybricks hub\n");
                gap_advertisements_enable(1);
            }
            break;
            
        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) == HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                connection_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                printf("Pybricks hub connected! Handle: 0x%04x\n", connection_handle);
            }
            break;
            
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("Disconnected\n");
            connection_handle = HCI_CON_HANDLE_INVALID;
            nus_tx_enabled = false;
            pybricks_notifications_enabled = false;
            gap_advertisements_enable(1);
            break;
    }
}

// Status functions
bool btstack_is_connected(void) {
    return connection_handle != HCI_CON_HANDLE_INVALID;
}

bool nus_is_ready(void) {
    return btstack_is_connected() && nus_tx_enabled;
}

bool pybricks_is_ready(void) {
    return btstack_is_connected() && pybricks_notifications_enabled;
}
