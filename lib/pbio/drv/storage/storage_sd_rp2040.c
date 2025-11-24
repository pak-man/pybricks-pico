// SPDX-License-Identifier: MIT
// SD Card storage driver for Pico W / Pico 2 W
// Uses SPI interface

#include <pbdrv/config.h>

#if PBDRV_CONFIG_STORAGE_SD_RP2040

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "storage_sd_rp2040.h"

// ============================================================================
// Configuration (from pbdrvconfig.h)
// ============================================================================

#define SDCARD_SPI_INSTANCE     PBDRV_CONFIG_STORAGE_SD_SPI_INSTANCE
#define SDCARD_PIN_MISO         PBDRV_CONFIG_STORAGE_SD_PIN_MISO
#define SDCARD_PIN_CS           PBDRV_CONFIG_STORAGE_SD_PIN_CS
#define SDCARD_PIN_SCK          PBDRV_CONFIG_STORAGE_SD_PIN_SCK
#define SDCARD_PIN_MOSI         PBDRV_CONFIG_STORAGE_SD_PIN_MOSI
#define SDCARD_PIN_CD           PBDRV_CONFIG_STORAGE_SD_PIN_CD
#define SDCARD_SPI_INIT_FREQ    PBDRV_CONFIG_STORAGE_SD_SPI_INIT_FREQ
#define SDCARD_SPI_FREQ         PBDRV_CONFIG_STORAGE_SD_SPI_FREQ

#define STORAGE_SETTINGS_SECTOR       PBDRV_CONFIG_STORAGE_SETTINGS_SECTOR
#define STORAGE_PROGRAM_START_SECTOR  PBDRV_CONFIG_STORAGE_PROGRAM_START_SECTOR
#define STORAGE_PROGRAM_MAX_SIZE      PBDRV_CONFIG_STORAGE_PROGRAM_MAX_SIZE

// SD card commands
#define CMD0    (0x40 + 0)      // GO_IDLE_STATE
#define CMD1    (0x40 + 1)      // SEND_OP_COND
#define CMD8    (0x40 + 8)      // SEND_IF_COND
#define CMD9    (0x40 + 9)      // SEND_CSD
#define CMD10   (0x40 + 10)     // SEND_CID
#define CMD12   (0x40 + 12)     // STOP_TRANSMISSION
#define CMD16   (0x40 + 16)     // SET_BLOCKLEN
#define CMD17   (0x40 + 17)     // READ_SINGLE_BLOCK
#define CMD18   (0x40 + 18)     // READ_MULTIPLE_BLOCK
#define CMD24   (0x40 + 24)     // WRITE_BLOCK
#define CMD25   (0x40 + 25)     // WRITE_MULTIPLE_BLOCK
#define CMD55   (0x40 + 55)     // APP_CMD
#define CMD58   (0x40 + 58)     // READ_OCR
#define ACMD41  (0xC0 + 41)     // SD_SEND_OP_COND

// R1 response bits
#define R1_IDLE_STATE       0x01
#define R1_ERASE_RESET      0x02
#define R1_ILLEGAL_CMD      0x04
#define R1_CRC_ERROR        0x08
#define R1_ERASE_SEQ_ERROR  0x10
#define R1_ADDRESS_ERROR    0x20
#define R1_PARAM_ERROR      0x40

// ============================================================================
// Types
// ============================================================================

// sdcard_type_t defined in storage_sd_rp2040.h

typedef struct {
    bool initialized;
    bool mounted;
    sdcard_type_t type;
    uint32_t sector_count;
    uint32_t sector_size;
    uint64_t capacity_bytes;
} sdcard_state_t;

static sdcard_state_t sd_state = {0};

// ============================================================================
// Low-level SPI functions
// ============================================================================

static inline void cs_select(void) {
    gpio_put(SDCARD_PIN_CS, 0);
    sleep_us(1);
}

static inline void cs_deselect(void) {
    sleep_us(1);
    gpio_put(SDCARD_PIN_CS, 1);
}

static uint8_t spi_transfer(uint8_t data) {
    uint8_t rx;
    spi_write_read_blocking(SDCARD_SPI_INSTANCE, &data, &rx, 1);
    return rx;
}

static void spi_transfer_buf(const uint8_t *tx, uint8_t *rx, size_t len) {
    if (tx && rx) {
        spi_write_read_blocking(SDCARD_SPI_INSTANCE, tx, rx, len);
    } else if (tx) {
        spi_write_blocking(SDCARD_SPI_INSTANCE, tx, len);
    } else if (rx) {
        spi_read_blocking(SDCARD_SPI_INSTANCE, 0xFF, rx, len);
    }
}

// Wait for card ready (not busy)
static bool sd_wait_ready(uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    
    while (!time_reached(deadline)) {
        if (spi_transfer(0xFF) == 0xFF) {
            return true;
        }
    }
    return false;
}

// Send command and get R1 response
static uint8_t sd_send_cmd(uint8_t cmd, uint32_t arg) {
    uint8_t response;
    
    // Handle ACMD (application command)
    if (cmd & 0x80) {
        cmd &= 0x7F;
        response = sd_send_cmd(CMD55, 0);
        if (response > 1) return response;
    }
    
    // Wait for ready
    if (!sd_wait_ready(500)) {
        return 0xFF;
    }
    
    // Send command packet
    uint8_t packet[6];
    packet[0] = cmd;
    packet[1] = (arg >> 24) & 0xFF;
    packet[2] = (arg >> 16) & 0xFF;
    packet[3] = (arg >> 8) & 0xFF;
    packet[4] = arg & 0xFF;
    
    // CRC (only CMD0 and CMD8 need valid CRC)
    if (cmd == CMD0) {
        packet[5] = 0x95;   // Valid CRC for CMD0(0)
    } else if (cmd == CMD8) {
        packet[5] = 0x87;   // Valid CRC for CMD8(0x1AA)
    } else {
        packet[5] = 0x01;   // Dummy CRC + stop bit
    }
    
    spi_transfer_buf(packet, NULL, 6);
    
    // Wait for response (1-8 bytes)
    for (int i = 0; i < 8; i++) {
        response = spi_transfer(0xFF);
        if (!(response & 0x80)) {
            return response;
        }
    }
    
    return 0xFF;
}

// ============================================================================
// SD Card Initialization
// ============================================================================

int pbdrv_storage_sd_init(void) {
    if (sd_state.initialized) {
        return 0;
    }
    
    printf("Initializing SD card...\n");
    
    // Initialize SPI
    spi_init(SDCARD_SPI_INSTANCE, SDCARD_SPI_INIT_FREQ);
    gpio_set_function(SDCARD_PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(SDCARD_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SDCARD_PIN_MOSI, GPIO_FUNC_SPI);
    
    // CS pin
    gpio_init(SDCARD_PIN_CS);
    gpio_set_dir(SDCARD_PIN_CS, GPIO_OUT);
    gpio_put(SDCARD_PIN_CS, 1);
    
    // Card detect pin (optional)
    if (SDCARD_PIN_CD != 0xFF) {
        gpio_init(SDCARD_PIN_CD);
        gpio_set_dir(SDCARD_PIN_CD, GPIO_IN);
        gpio_pull_up(SDCARD_PIN_CD);
    }
    
    // Send 80+ clock pulses with CS high
    cs_deselect();
    for (int i = 0; i < 10; i++) {
        spi_transfer(0xFF);
    }
    
    cs_select();
    
    // CMD0 - Go idle state
    uint8_t response;
    for (int retry = 0; retry < 10; retry++) {
        response = sd_send_cmd(CMD0, 0);
        if (response == R1_IDLE_STATE) break;
        sleep_ms(10);
    }
    
    if (response != R1_IDLE_STATE) {
        printf("SD: CMD0 failed (0x%02X)\n", response);
        cs_deselect();
        return -1;
    }
    
    // CMD8 - Check voltage range (SD v2)
    response = sd_send_cmd(CMD8, 0x1AA);
    
    if (response == R1_IDLE_STATE) {
        // SD v2 card
        uint8_t ocr[4];
        spi_transfer_buf(NULL, ocr, 4);
        
        if (ocr[2] != 0x01 || ocr[3] != 0xAA) {
            printf("SD: CMD8 voltage mismatch\n");
            cs_deselect();
            return -2;
        }
        
        // ACMD41 - Initialize (with HCS bit for SDHC)
        absolute_time_t deadline = make_timeout_time_ms(1000);
        while (!time_reached(deadline)) {
            response = sd_send_cmd(ACMD41, 0x40000000);
            if (response == 0) break;
            sleep_ms(10);
        }
        
        if (response != 0) {
            printf("SD: ACMD41 timeout\n");
            cs_deselect();
            return -3;
        }
        
        // CMD58 - Read OCR to check CCS bit
        response = sd_send_cmd(CMD58, 0);
        if (response == 0) {
            spi_transfer_buf(NULL, ocr, 4);
            sd_state.type = (ocr[0] & 0x40) ? SDCARD_TYPE_SDHC : SDCARD_TYPE_SD2;
        }
        
    } else if (response == (R1_IDLE_STATE | R1_ILLEGAL_CMD)) {
        // SD v1 or MMC
        sd_state.type = SDCARD_TYPE_SD1;
        
        // Try ACMD41 first (SD)
        response = sd_send_cmd(ACMD41, 0);
        
        absolute_time_t deadline = make_timeout_time_ms(1000);
        while (!time_reached(deadline)) {
            response = sd_send_cmd(ACMD41, 0);
            if (response == 0) break;
            sleep_ms(10);
        }
        
        if (response != 0) {
            // Try CMD1 (MMC)
            deadline = make_timeout_time_ms(1000);
            while (!time_reached(deadline)) {
                response = sd_send_cmd(CMD1, 0);
                if (response == 0) break;
                sleep_ms(10);
            }
        }
        
        if (response != 0) {
            printf("SD: Init timeout\n");
            cs_deselect();
            return -4;
        }
        
    } else {
        printf("SD: Unknown card type\n");
        cs_deselect();
        return -5;
    }
    
    // Set block size to 512 bytes (for non-SDHC)
    if (sd_state.type != SDCARD_TYPE_SDHC) {
        response = sd_send_cmd(CMD16, 512);
        if (response != 0) {
            printf("SD: CMD16 failed\n");
            cs_deselect();
            return -6;
        }
    }
    
    cs_deselect();
    
    // Switch to full speed
    spi_set_baudrate(SDCARD_SPI_INSTANCE, SDCARD_SPI_FREQ);
    
    sd_state.sector_size = 512;
    sd_state.initialized = true;
    
    printf("SD: Initialized (type=%d)\n", sd_state.type);
    
    // Read capacity from CSD
    pbdrv_storage_sd_get_capacity_impl();
    
    return 0;
}

// ============================================================================
// Block Read/Write
// ============================================================================

int pbdrv_storage_sd_read_block(uint32_t block, uint8_t *buffer) {
    if (!sd_state.initialized) return -1;
    
    // Convert block to byte address for non-SDHC
    uint32_t addr = (sd_state.type == SDCARD_TYPE_SDHC) ? block : (block * 512);
    
    cs_select();
    
    uint8_t response = sd_send_cmd(CMD17, addr);
    if (response != 0) {
        cs_deselect();
        return -2;
    }
    
    // Wait for data token
    absolute_time_t deadline = make_timeout_time_ms(200);
    uint8_t token;
    do {
        token = spi_transfer(0xFF);
        if (time_reached(deadline)) {
            cs_deselect();
            return -3;
        }
    } while (token == 0xFF);
    
    if (token != 0xFE) {
        cs_deselect();
        return -4;
    }
    
    // Read data
    spi_transfer_buf(NULL, buffer, 512);
    
    // Read and discard CRC
    spi_transfer(0xFF);
    spi_transfer(0xFF);
    
    cs_deselect();
    
    return 0;
}

int pbdrv_storage_sd_write_block(uint32_t block, const uint8_t *buffer) {
    if (!sd_state.initialized) return -1;
    
    uint32_t addr = (sd_state.type == SDCARD_TYPE_SDHC) ? block : (block * 512);
    
    cs_select();
    
    uint8_t response = sd_send_cmd(CMD24, addr);
    if (response != 0) {
        cs_deselect();
        return -2;
    }
    
    // Wait for ready
    if (!sd_wait_ready(500)) {
        cs_deselect();
        return -3;
    }
    
    // Send data token
    spi_transfer(0xFE);
    
    // Send data
    spi_transfer_buf(buffer, NULL, 512);
    
    // Send dummy CRC
    spi_transfer(0xFF);
    spi_transfer(0xFF);
    
    // Check data response
    response = spi_transfer(0xFF);
    if ((response & 0x1F) != 0x05) {
        cs_deselect();
        return -4;
    }
    
    // Wait for write to complete
    if (!sd_wait_ready(500)) {
        cs_deselect();
        return -5;
    }
    
    cs_deselect();
    
    return 0;
}

// Multi-block read
int pbdrv_storage_sd_read_blocks(uint32_t block, uint8_t *buffer, uint32_t count) {
    if (!sd_state.initialized) return -1;
    
    uint32_t addr = (sd_state.type == SDCARD_TYPE_SDHC) ? block : (block * 512);
    
    cs_select();
    
    uint8_t response = sd_send_cmd(CMD18, addr);
    if (response != 0) {
        cs_deselect();
        return -2;
    }
    
    for (uint32_t i = 0; i < count; i++) {
        // Wait for data token
        absolute_time_t deadline = make_timeout_time_ms(200);
        uint8_t token;
        do {
            token = spi_transfer(0xFF);
            if (time_reached(deadline)) {
                sd_send_cmd(CMD12, 0);
                cs_deselect();
                return -3;
            }
        } while (token == 0xFF);
        
        if (token != 0xFE) {
            sd_send_cmd(CMD12, 0);
            cs_deselect();
            return -4;
        }
        
        // Read data
        spi_transfer_buf(NULL, buffer + (i * 512), 512);
        
        // Read CRC
        spi_transfer(0xFF);
        spi_transfer(0xFF);
    }
    
    // Stop transmission
    sd_send_cmd(CMD12, 0);
    sd_wait_ready(100);
    
    cs_deselect();
    
    return 0;
}

// Multi-block write
int pbdrv_storage_sd_write_blocks(uint32_t block, const uint8_t *buffer, uint32_t count) {
    if (!sd_state.initialized) return -1;
    
    uint32_t addr = (sd_state.type == SDCARD_TYPE_SDHC) ? block : (block * 512);
    
    cs_select();
    
    uint8_t response = sd_send_cmd(CMD25, addr);
    if (response != 0) {
        cs_deselect();
        return -2;
    }
    
    for (uint32_t i = 0; i < count; i++) {
        // Wait for ready
        if (!sd_wait_ready(500)) {
            cs_deselect();
            return -3;
        }
        
        // Send multi-block data token
        spi_transfer(0xFC);
        
        // Send data
        spi_transfer_buf(buffer + (i * 512), NULL, 512);
        
        // Send dummy CRC
        spi_transfer(0xFF);
        spi_transfer(0xFF);
        
        // Check response
        response = spi_transfer(0xFF);
        if ((response & 0x1F) != 0x05) {
            cs_deselect();
            return -4;
        }
    }
    
    // Wait for ready
    if (!sd_wait_ready(500)) {
        cs_deselect();
        return -5;
    }
    
    // Send stop token
    spi_transfer(0xFD);
    
    // Wait for write to complete
    if (!sd_wait_ready(500)) {
        cs_deselect();
        return -6;
    }
    
    cs_deselect();
    
    return 0;
}

// ============================================================================
// Card Information
// ============================================================================

uint64_t pbdrv_storage_sd_get_capacity_impl(void) {
    if (!sd_state.initialized) return 0;
    if (sd_state.capacity_bytes > 0) return sd_state.capacity_bytes;
    
    cs_select();
    
    uint8_t response = sd_send_cmd(CMD9, 0);
    if (response != 0) {
        cs_deselect();
        return 0;
    }
    
    // Wait for data token
    absolute_time_t deadline = make_timeout_time_ms(200);
    uint8_t token;
    do {
        token = spi_transfer(0xFF);
        if (time_reached(deadline)) {
            cs_deselect();
            return 0;
        }
    } while (token == 0xFF);
    
    if (token != 0xFE) {
        cs_deselect();
        return 0;
    }
    
    // Read CSD
    uint8_t csd[16];
    spi_transfer_buf(NULL, csd, 16);
    
    // Read CRC
    spi_transfer(0xFF);
    spi_transfer(0xFF);
    
    cs_deselect();
    
    // Parse CSD
    uint64_t capacity = 0;
    
    if ((csd[0] >> 6) == 0) {
        // CSD v1
        uint32_t c_size = ((csd[6] & 0x03) << 10) | (csd[7] << 2) | ((csd[8] >> 6) & 0x03);
        uint32_t c_size_mult = ((csd[9] & 0x03) << 1) | ((csd[10] >> 7) & 0x01);
        uint32_t read_bl_len = csd[5] & 0x0F;
        
        uint32_t block_nr = (c_size + 1) << (c_size_mult + 2);
        uint32_t block_len = 1 << read_bl_len;
        
        capacity = (uint64_t)block_nr * block_len;
        
    } else if ((csd[0] >> 6) == 1) {
        // CSD v2 (SDHC/SDXC)
        uint32_t c_size = ((csd[7] & 0x3F) << 16) | (csd[8] << 8) | csd[9];
        capacity = ((uint64_t)c_size + 1) * 512 * 1024;
    }
    
    sd_state.capacity_bytes = capacity;
    sd_state.sector_count = capacity / 512;
    
    printf("SD: Capacity = %llu MB (%lu sectors)\n", 
           capacity / (1024 * 1024), sd_state.sector_count);
    
    return capacity;
}

// ============================================================================
// Deinit
// ============================================================================

void pbdrv_storage_sd_deinit(void) {
    if (!sd_state.initialized) return;
    
    spi_deinit(SDCARD_SPI_INSTANCE);
    
    gpio_set_function(SDCARD_PIN_MISO, GPIO_FUNC_NULL);
    gpio_set_function(SDCARD_PIN_SCK, GPIO_FUNC_NULL);
    gpio_set_function(SDCARD_PIN_MOSI, GPIO_FUNC_NULL);
    gpio_set_function(SDCARD_PIN_CS, GPIO_FUNC_NULL);
    
    memset(&sd_state, 0, sizeof(sd_state));
}

#endif // PBDRV_CONFIG_STORAGE_SD_RP2040