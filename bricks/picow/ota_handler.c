#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "pico_fota_bootloader/core.h"
#include "lwip/apps/httpd.h"

#define OTA_BUFFER_SIZE 4096
#define FIRMWARE_VERSION "1.0.0"

static uint8_t ota_buffer[OTA_BUFFER_SIZE];
static size_t ota_buffer_pos = 0;
static size_t ota_total_bytes = 0;
static bool ota_in_progress = false;

// Get firmware version
const char* ota_get_version(void) {
    return FIRMWARE_VERSION;
}

// Check if system is after firmware update
bool ota_is_after_update(void) {
    return pfb_is_after_firmware_update();
}

// Check if system is after rollback
bool ota_is_after_rollback(void) {
    return pfb_is_after_rollback();
}

// Commit firmware (prevents rollback on next reboot)
void ota_commit_firmware(void) {
    pfb_firmware_commit();
    printf("[OTA] Firmware committed\n");
}

// Initialize OTA process
int ota_begin(void) {
    if (ota_in_progress) {
        return -1;
    }
    
    printf("[OTA] Initializing download slot...\n");
    pfb_initialize_download_slot();
    
    ota_buffer_pos = 0;
    ota_total_bytes = 0;
    ota_in_progress = true;
    
    printf("[OTA] Ready to receive firmware\n");
    return 0;
}

// Write data chunk to OTA buffer
int ota_write(const uint8_t* data, size_t len) {
    if (!ota_in_progress) {
        return -1;
    }
    
    size_t remaining = len;
    size_t offset = 0;
    
    while (remaining > 0) {
        size_t to_copy = OTA_BUFFER_SIZE - ota_buffer_pos;
        if (to_copy > remaining) {
            to_copy = remaining;
        }
        
        memcpy(ota_buffer + ota_buffer_pos, data + offset, to_copy);
        ota_buffer_pos += to_copy;
        offset += to_copy;
        remaining -= to_copy;
        
        // Flush buffer when we have 256-byte aligned chunk
        if (ota_buffer_pos >= 256 && (ota_buffer_pos % 256) == 0) {
            size_t flush_size = (ota_buffer_pos / 256) * 256;
            
            if (pfb_write_to_flash_aligned_256_bytes(ota_buffer, ota_total_bytes, flush_size)) {
                printf("[OTA] Flash write error at offset %zu\n", ota_total_bytes);
                ota_in_progress = false;
                return -2;
            }
            
            ota_total_bytes += flush_size;
            
            // Move remaining bytes to start of buffer
            if (ota_buffer_pos > flush_size) {
                memmove(ota_buffer, ota_buffer + flush_size, ota_buffer_pos - flush_size);
            }
            ota_buffer_pos -= flush_size;
            
            printf("[OTA] Written %zu bytes\n", ota_total_bytes);
        }
    }
    
    return 0;
}

// Finalize OTA and verify
int ota_end(void) {
    if (!ota_in_progress) {
        return -1;
    }
    
    // Flush remaining data (pad to 256 bytes)
    if (ota_buffer_pos > 0) {
        size_t padded_size = ((ota_buffer_pos + 255) / 256) * 256;
        
        // Pad with 0xFF (flash erased state)
        memset(ota_buffer + ota_buffer_pos, 0xFF, padded_size - ota_buffer_pos);
        
        if (pfb_write_to_flash_aligned_256_bytes(ota_buffer, ota_total_bytes, padded_size)) {
            printf("[OTA] Final flash write error\n");
            ota_in_progress = false;
            return -2;
        }
        
        ota_total_bytes += padded_size;
    }
    
    printf("[OTA] Total bytes written: %zu\n", ota_total_bytes);
    
    // Verify SHA256
    printf("[OTA] Verifying SHA256...\n");
    if (pfb_firmware_sha256_check(ota_total_bytes)) {
        printf("[OTA] SHA256 verification failed!\n");
        ota_in_progress = false;
        return -3;
    }
    
    printf("[OTA] SHA256 OK\n");
    
    // Mark download slot as valid
    pfb_mark_download_slot_as_valid();
    
    ota_in_progress = false;
    printf("[OTA] Firmware ready for installation\n");
    
    return 0;
}

// Perform the update (reboot)
void ota_perform_update(void) {
    printf("[OTA] Performing update, rebooting...\n");
    sleep_ms(100);
    pfb_perform_update();
    // Never returns
}

// Get OTA progress
void ota_get_progress(size_t* bytes_written, bool* in_progress) {
    *bytes_written = ota_total_bytes;
    *in_progress = ota_in_progress;
}

// Abort OTA
void ota_abort(void) {
    ota_in_progress = false;
    ota_buffer_pos = 0;
    ota_total_bytes = 0;
    printf("[OTA] Aborted\n");
}
