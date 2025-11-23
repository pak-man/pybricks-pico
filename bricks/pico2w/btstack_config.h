// BTstack configuration for Pico W

#ifndef BTSTACK_CONFIG_H
#define BTSTACK_CONFIG_H

// Port related features
#define HAVE_EHCILL
#define HAVE_EMBEDDED_TIME_MS

// BTstack features (check if not already defined)
#ifndef ENABLE_BLE
#define ENABLE_BLE
#endif

#define ENABLE_LE_PERIPHERAL
#define ENABLE_L2CAP_LE_CREDIT_BASED_FLOW_CONTROL_MODE
#define ENABLE_LE_DATA_LENGTH_EXTENSION
#define ENABLE_LE_SECURE_CONNECTIONS

// BTstack configuration
#define HCI_ACL_PAYLOAD_SIZE (1691 + 4)
#define MAX_NR_GATT_CLIENTS 1
#define MAX_NR_HCI_CONNECTIONS 1
#define MAX_NR_L2CAP_SERVICES  2
#define MAX_NR_L2CAP_CHANNELS  2
#define MAX_NR_RFCOMM_MULTIPLEXERS 0
#define MAX_NR_RFCOMM_SERVICES 0
#define MAX_NR_RFCOMM_CHANNELS 0
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES 0
#define MAX_NR_SM_LOOKUP_ENTRIES 3
#define MAX_NR_WHITELIST_ENTRIES 1
#define MAX_NR_LE_DEVICE_DB_ENTRIES 1

// ATT DB size (for storing GATT database)
#define MAX_ATT_DB_SIZE 512

// NVM (non-volatile memory) for device bonding
#define NVM_NUM_DEVICE_DB_ENTRIES 1

// CYW43-specific HCI transport settings
#define HCI_OUTGOING_PRE_BUFFER_SIZE 4
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT 4

// Logging and debug
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

#endif // BTSTACK_CONFIG_H
