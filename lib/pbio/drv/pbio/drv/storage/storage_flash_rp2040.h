// SPDX-License-Identifier: MIT
// Internal flash storage driver for Pico W / Pico 2 W

#ifndef _PBDRV_STORAGE_FLASH_RP2040_H_
#define _PBDRV_STORAGE_FLASH_RP2040_H_

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// File storage (like EV3 File Access block)
// ============================================================================

// Initialize flash storage
int pbdrv_storage_flash_init(void);

// Read file from flash
// Returns 0 on success, negative on error
int pbdrv_storage_flash_read(const char *name, uint8_t *buffer, uint32_t max_size, uint32_t *actual_size);

// Write file to flash
// Returns 0 on success, negative on error
int pbdrv_storage_flash_write(const char *name, const uint8_t *buffer, uint32_t size);

// Delete file from flash
int pbdrv_storage_flash_delete(const char *name);

// Check if file exists
bool pbdrv_storage_flash_exists(const char *name);

// Get file size
int pbdrv_storage_flash_size(const char *name, uint32_t *size);

// Sync cached data to flash
int pbdrv_storage_flash_sync(void);

// Format storage (erase all files)
int pbdrv_storage_flash_format(void);

// ============================================================================
// Settings storage (key-value pairs)
// ============================================================================

// Get setting value
int pbdrv_storage_setting_get(const char *key, uint8_t *value, uint32_t max_size, uint32_t *actual_size);

// Set setting value
int pbdrv_storage_setting_set(const char *key, const uint8_t *value, uint32_t size);

// Delete setting
int pbdrv_storage_setting_delete(const char *key);

// ============================================================================
// Info functions
// ============================================================================

// Get total storage size in bytes
uint32_t pbdrv_storage_flash_get_total_size(void);

// Get free storage size in bytes
uint32_t pbdrv_storage_flash_get_free_size(void);

// Get number of files stored
uint32_t pbdrv_storage_flash_get_file_count(void);

#endif // _PBDRV_STORAGE_FLASH_RP2040_H_
