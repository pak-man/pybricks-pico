// SPDX-License-Identifier: MIT
// SD Card storage driver for Pico W / Pico 2 W

#ifndef _PBDRV_STORAGE_SD_RP2040_H_
#define _PBDRV_STORAGE_SD_RP2040_H_

#include <stdint.h>
#include <stdbool.h>

// SD card types
typedef enum {
    SDCARD_TYPE_UNKNOWN = 0,
    SDCARD_TYPE_SD1,        // SD v1
    SDCARD_TYPE_SD2,        // SD v2 (SDSC)
    SDCARD_TYPE_SDHC,       // SDHC/SDXC
} sdcard_type_t;

// ============================================================================
// Low-level SD card functions
// ============================================================================

// Initialize SD card (SPI mode)
int pbdrv_storage_sd_init(void);

// Deinitialize SD card
void pbdrv_storage_sd_deinit(void);

// Get SD card type
sdcard_type_t pbdrv_storage_sd_get_type(void);

// Get capacity in bytes (internal implementation - called by storage.c)
uint64_t pbdrv_storage_sd_get_capacity_internal(void);

// Get sector count (512-byte sectors)
uint32_t pbdrv_storage_sd_get_sector_count(void);

// ============================================================================
// Block-level access (512-byte blocks)
// ============================================================================

// Read single block
int pbdrv_storage_sd_read_block(uint32_t block, uint8_t *buffer);

// Write single block
int pbdrv_storage_sd_write_block(uint32_t block, const uint8_t *buffer);

// Read multiple blocks
int pbdrv_storage_sd_read_blocks(uint32_t block, uint8_t *buffer, uint32_t count);

// Write multiple blocks
int pbdrv_storage_sd_write_blocks(uint32_t block, const uint8_t *buffer, uint32_t count);

// ============================================================================
// Note: The following functions are defined in storage.c (not here):
//   - pbdrv_storage_sd_is_present()
//   - pbdrv_storage_sd_get_capacity()
//   - pbdrv_storage_read_program()
//   - pbdrv_storage_write_program()
// ============================================================================

#endif // _PBDRV_STORAGE_SD_RP2040_H_