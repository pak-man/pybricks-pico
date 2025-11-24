// SPDX-License-Identifier: MIT
// Unified storage driver for Pico W / Pico 2 W
// Combines internal flash + optional SD card

#ifndef _PBDRV_STORAGE_H_
#define _PBDRV_STORAGE_H_

#include <stdint.h>
#include <stdbool.h>
#include <pbdrv/config.h>

// ============================================================================
// Storage types
// ============================================================================

typedef enum {
    PBDRV_STORAGE_FLASH = 0,    // Internal flash (always available)
    PBDRV_STORAGE_SD = 1,       // SD card (optional)
} pbdrv_storage_type_t;

// ============================================================================
// Unified File API
// ============================================================================

// Initialize all storage subsystems
int pbdrv_storage_init(void);

// Read file
// storage: PBDRV_STORAGE_FLASH or PBDRV_STORAGE_SD
int pbdrv_storage_read_file(pbdrv_storage_type_t storage, const char *name, 
                            uint8_t *buffer, uint32_t max_size, uint32_t *actual_size);

// Write file
int pbdrv_storage_write_file(pbdrv_storage_type_t storage, const char *name,
                             const uint8_t *buffer, uint32_t size);

// Delete file
int pbdrv_storage_delete_file(pbdrv_storage_type_t storage, const char *name);

// Check if file exists
bool pbdrv_storage_file_exists(pbdrv_storage_type_t storage, const char *name);

// Get file size
int pbdrv_storage_file_size(pbdrv_storage_type_t storage, const char *name, uint32_t *size);

// ============================================================================
// Storage info
// ============================================================================

// Check if SD card is available
bool pbdrv_storage_sd_available(void);

// Direct SD card access (wrappers)
bool pbdrv_storage_sd_is_present(void);
uint64_t pbdrv_storage_sd_get_capacity(void);

// Get total size
uint32_t pbdrv_storage_get_total_size(pbdrv_storage_type_t storage);

// Get free size
uint32_t pbdrv_storage_get_free_size(pbdrv_storage_type_t storage);

// ============================================================================
// Settings API (always uses internal flash)
// ============================================================================

int pbdrv_storage_get_setting(const char *key, uint8_t *value, uint32_t max_size, uint32_t *actual_size);
int pbdrv_storage_set_setting(const char *key, const uint8_t *value, uint32_t size);
int pbdrv_storage_delete_setting(const char *key);

// Convenience functions for common types
int pbdrv_storage_get_setting_int(const char *key, int32_t *value);
int pbdrv_storage_set_setting_int(const char *key, int32_t value);
int pbdrv_storage_get_setting_str(const char *key, char *value, uint32_t max_size);
int pbdrv_storage_set_setting_str(const char *key, const char *value);

// ============================================================================
// User program storage (pybricks compatible)
// ============================================================================

// Read user program (from SD if available, else flash)
int pbdrv_storage_read_program(uint8_t *buffer, uint32_t offset, uint32_t size);

// Write user program
int pbdrv_storage_write_program(const uint8_t *buffer, uint32_t offset, uint32_t size);

// Get max program size
uint32_t pbdrv_storage_get_program_max_size(void);

#endif // _PBDRV_STORAGE_H_
