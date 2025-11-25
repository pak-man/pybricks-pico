#ifndef OTA_HANDLER_H
#define OTA_HANDLER_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

// Get current firmware version
const char* ota_get_version(void);

// Check if system just updated
bool ota_is_after_update(void);

// Check if system rolled back
bool ota_is_after_rollback(void);

// Commit firmware (prevents rollback)
void ota_commit_firmware(void);

// Start OTA process
int ota_begin(void);

// Write firmware data
int ota_write(const uint8_t* data, size_t len);

// Finalize and verify firmware
int ota_end(void);

// Reboot and install new firmware
void ota_perform_update(void);

// Get upload progress
void ota_get_progress(size_t* bytes_written, bool* in_progress);

// Cancel upload
void ota_abort(void);

#endif // OTA_HANDLER_H
