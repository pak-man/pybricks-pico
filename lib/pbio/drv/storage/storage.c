// SPDX-License-Identifier: MIT
// Unified storage driver for Pico W / Pico 2 W

#include <pbdrv/config.h>
#include <string.h>
#include <stdio.h>
#include "storage.h"

#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
#include "storage_flash_rp2040.h"
#endif

#if PBDRV_CONFIG_STORAGE_SD_RP2040
#include "storage_sd_rp2040.h"
#endif

// ============================================================================
// Initialization
// ============================================================================

static bool storage_initialized = false;
static bool sd_available = false;

int pbdrv_storage_init(void) {
    if (storage_initialized) {
        return 0;
    }
    
    int err = 0;
    
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
    err = pbdrv_storage_flash_init();
    if (err != 0) {
        printf("Storage: Flash init failed (%d)\n", err);
    }
#endif

#if PBDRV_CONFIG_STORAGE_SD_RP2040
    err = pbdrv_storage_sd_init();
    if (err == 0) {
        sd_available = true;
        printf("Storage: SD card available\n");
    } else {
        sd_available = false;
        printf("Storage: No SD card\n");
    }
#endif

    storage_initialized = true;
    return 0;
}

// ============================================================================
// Unified File API
// ============================================================================

int pbdrv_storage_read_file(pbdrv_storage_type_t storage, const char *name,
                            uint8_t *buffer, uint32_t max_size, uint32_t *actual_size) {
    if (!storage_initialized) {
        pbdrv_storage_init();
    }
    
    if (storage == PBDRV_STORAGE_FLASH) {
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
        return pbdrv_storage_flash_read(name, buffer, max_size, actual_size);
#else
        return -1;
#endif
    } else if (storage == PBDRV_STORAGE_SD) {
#if PBDRV_CONFIG_STORAGE_SD_RP2040
        if (!sd_available) return -1;
        // SD card uses block-level access, need filename->block mapping
        // For now, return not implemented
        return -99;  // TODO: Implement FAT filesystem
#else
        return -1;
#endif
    }
    return -1;
}

int pbdrv_storage_write_file(pbdrv_storage_type_t storage, const char *name,
                             const uint8_t *buffer, uint32_t size) {
    if (!storage_initialized) {
        pbdrv_storage_init();
    }
    
    if (storage == PBDRV_STORAGE_FLASH) {
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
        return pbdrv_storage_flash_write(name, buffer, size);
#else
        return -1;
#endif
    } else if (storage == PBDRV_STORAGE_SD) {
#if PBDRV_CONFIG_STORAGE_SD_RP2040
        if (!sd_available) return -1;
        return -99;  // TODO: Implement FAT filesystem
#else
        return -1;
#endif
    }
    return -1;
}

int pbdrv_storage_delete_file(pbdrv_storage_type_t storage, const char *name) {
    if (!storage_initialized) {
        pbdrv_storage_init();
    }
    
    if (storage == PBDRV_STORAGE_FLASH) {
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
        return pbdrv_storage_flash_delete(name);
#else
        return -1;
#endif
    }
    return -1;
}

bool pbdrv_storage_file_exists(pbdrv_storage_type_t storage, const char *name) {
    if (!storage_initialized) {
        pbdrv_storage_init();
    }
    
    if (storage == PBDRV_STORAGE_FLASH) {
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
        return pbdrv_storage_flash_exists(name);
#else
        return false;
#endif
    }
    return false;
}

int pbdrv_storage_file_size(pbdrv_storage_type_t storage, const char *name, uint32_t *size) {
    if (!storage_initialized) {
        pbdrv_storage_init();
    }
    
    if (storage == PBDRV_STORAGE_FLASH) {
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
        return pbdrv_storage_flash_size(name, size);
#else
        return -1;
#endif
    }
    return -1;
}

// ============================================================================
// Storage info
// ============================================================================

bool pbdrv_storage_sd_available(void) {
    if (!storage_initialized) {
        pbdrv_storage_init();
    }
    return sd_available;
}

uint32_t pbdrv_storage_get_total_size(pbdrv_storage_type_t storage) {
    if (!storage_initialized) {
        pbdrv_storage_init();
    }
    
    if (storage == PBDRV_STORAGE_FLASH) {
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
        return pbdrv_storage_flash_get_total_size();
#else
        return 0;
#endif
    } else if (storage == PBDRV_STORAGE_SD) {
#if PBDRV_CONFIG_STORAGE_SD_RP2040
        return (uint32_t)pbdrv_storage_sd_get_capacity();
#else
        return 0;
#endif
    }
    return 0;
}

uint32_t pbdrv_storage_get_free_size(pbdrv_storage_type_t storage) {
    if (!storage_initialized) {
        pbdrv_storage_init();
    }
    
    if (storage == PBDRV_STORAGE_FLASH) {
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
        return pbdrv_storage_flash_get_free_size();
#else
        return 0;
#endif
    }
    // SD free size requires FAT filesystem
    return 0;
}

// ============================================================================
// Settings API
// ============================================================================

int pbdrv_storage_get_setting(const char *key, uint8_t *value, uint32_t max_size, uint32_t *actual_size) {
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
    return pbdrv_storage_setting_get(key, value, max_size, actual_size);
#else
    return -1;
#endif
}

int pbdrv_storage_set_setting(const char *key, const uint8_t *value, uint32_t size) {
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
    return pbdrv_storage_setting_set(key, value, size);
#else
    return -1;
#endif
}

int pbdrv_storage_delete_setting(const char *key) {
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
    return pbdrv_storage_setting_delete(key);
#else
    return -1;
#endif
}

// Convenience functions
int pbdrv_storage_get_setting_int(const char *key, int32_t *value) {
    uint32_t size;
    return pbdrv_storage_get_setting(key, (uint8_t *)value, sizeof(int32_t), &size);
}

int pbdrv_storage_set_setting_int(const char *key, int32_t value) {
    return pbdrv_storage_set_setting(key, (uint8_t *)&value, sizeof(int32_t));
}

int pbdrv_storage_get_setting_str(const char *key, char *value, uint32_t max_size) {
    uint32_t size;
    int err = pbdrv_storage_get_setting(key, (uint8_t *)value, max_size - 1, &size);
    if (err == 0) {
        value[size] = '\0';
    }
    return err;
}

int pbdrv_storage_set_setting_str(const char *key, const char *value) {
    return pbdrv_storage_set_setting(key, (uint8_t *)value, strlen(value));
}

// ============================================================================
// User program storage
// ============================================================================

int pbdrv_storage_read_program(uint8_t *buffer, uint32_t offset, uint32_t size) {
    if (!storage_initialized) {
        pbdrv_storage_init();
    }
    
#if PBDRV_CONFIG_STORAGE_SD_RP2040
    // Prefer SD card for program storage
    if (sd_available) {
        uint32_t start_sector = PBDRV_CONFIG_STORAGE_PROGRAM_START_SECTOR + (offset / 512);
        uint32_t sector_offset = offset % 512;
        uint32_t bytes_read = 0;
        uint8_t temp[512];
        
        while (bytes_read < size) {
            int err = pbdrv_storage_sd_read_block(start_sector, temp);
            if (err != 0) return err;
            
            uint32_t copy_start = (bytes_read == 0) ? sector_offset : 0;
            uint32_t copy_len = 512 - copy_start;
            if (copy_len > (size - bytes_read)) {
                copy_len = size - bytes_read;
            }
            
            memcpy(buffer + bytes_read, temp + copy_start, copy_len);
            bytes_read += copy_len;
            start_sector++;
        }
        return 0;
    }
#endif

#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
    // Fall back to flash
    uint32_t actual;
    return pbdrv_storage_flash_read("program", buffer, size, &actual);
#else
    return -1;
#endif
}

int pbdrv_storage_write_program(const uint8_t *buffer, uint32_t offset, uint32_t size) {
    if (!storage_initialized) {
        pbdrv_storage_init();
    }
    
#if PBDRV_CONFIG_STORAGE_SD_RP2040
    if (sd_available) {
        if (offset + size > PBDRV_CONFIG_STORAGE_PROGRAM_MAX_SIZE) {
            return -1;
        }
        
        uint32_t start_sector = PBDRV_CONFIG_STORAGE_PROGRAM_START_SECTOR + (offset / 512);
        uint32_t sector_offset = offset % 512;
        uint32_t bytes_written = 0;
        uint8_t temp[512];
        
        while (bytes_written < size) {
            if (sector_offset != 0 || (size - bytes_written) < 512) {
                int err = pbdrv_storage_sd_read_block(start_sector, temp);
                if (err != 0) return err;
            }
            
            uint32_t copy_start = (bytes_written == 0) ? sector_offset : 0;
            uint32_t copy_len = 512 - copy_start;
            if (copy_len > (size - bytes_written)) {
                copy_len = size - bytes_written;
            }
            
            memcpy(temp + copy_start, buffer + bytes_written, copy_len);
            
            int err = pbdrv_storage_sd_write_block(start_sector, temp);
            if (err != 0) return err;
            
            bytes_written += copy_len;
            start_sector++;
            sector_offset = 0;
        }
        return 0;
    }
#endif

#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
    return pbdrv_storage_flash_write("program", buffer, size);
#else
    return -1;
#endif
}

uint32_t pbdrv_storage_get_program_max_size(void) {
#if PBDRV_CONFIG_STORAGE_SD_RP2040
    if (sd_available) {
        return PBDRV_CONFIG_STORAGE_PROGRAM_MAX_SIZE;
    }
#endif
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
    return pbdrv_storage_flash_get_total_size() / 2;  // Half for programs
#else
    return 0;
#endif
}

// Wrapper functions for direct SD card access
bool pbdrv_storage_sd_is_present(void) {
#if PBDRV_CONFIG_STORAGE_SD_RP2040
    return sd_available;
#else
    return false;
#endif
}

uint64_t pbdrv_storage_sd_get_capacity(void) {
#if PBDRV_CONFIG_STORAGE_SD_RP2040
    extern uint64_t pbdrv_storage_sd_get_capacity_internal(void);
    return pbdrv_storage_sd_get_capacity_internal();
#else
    return 0;
#endif
}