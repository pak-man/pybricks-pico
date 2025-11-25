# Pybricks OTA Firmware Update Implementation Guide

## Overview

This implements Over-The-Air firmware updates using `pico_fota_bootloader` by Jakub Zimnol.

## Memory Layout (Pico W - 2MB Flash)

```
0x10000000 - 0x10009000 (36KB)    : Bootloader
0x10009000 - 0x10009018 (24 bytes): Status flags (swap, rollback, valid)
0x10009018 - 0x1000A000 (4KB)     : Padding
0x1000A000 - 0x10107000 (1004KB)  : App Slot (running firmware)
0x10107000 - 0x10204000 (1004KB)  : Download Slot (OTA target)
```

## Setup Steps

### 1. Add Bootloader as Submodule

```bash
cd ~/pybricks-micropython/bricks/picow
git submodule add https://github.com/JZimnol/pico_fota_bootloader.git
git submodule update --init pico_fota_bootloader
```

### 2. Copy OTA Files

Copy these files from /home/claude/ to ~/pybricks-micropython/bricks/picow/:

- `CMakeLists_OTA.txt` → `CMakeLists.txt`
- `ota_handler.c`
- `ota_handler.h`
- `httpd_content_ota.c` → `httpd_content.c`
- `main_ota.c` → `main.c`

### 3. Set AES Encryption Key

**IMPORTANT**: Change the default encryption key in CMakeLists.txt:

```cmake
set(PFB_AES_KEY "YOUR_RANDOM_32_CHAR_HEX_KEY")
```

Generate a random key:
```bash
openssl rand -hex 16
```

All builds (bootloader + firmware) must use the SAME key.

### 4. Build

```bash
cd ~/pybricks-micropython/bricks/picow
mkdir -p build && cd build
cmake -DPFB_AES_KEY="your_key_here" ..
make -j$(nproc)
```

### 5. Initial Flash (via USB)

First time setup requires two .uf2 files:

1. **Flash bootloader** (only once):
   - Hold BOOTSEL, plug in USB
   - Copy `build/pico_fota_bootloader/pico_fota_bootloader.uf2`
   - Device reboots into bootloader (waiting for app)

2. **Flash application**:
   - Hold BOOTSEL again, plug in USB
   - Copy `build/pybricks_picow.uf2`
   - Device boots into your app

### 6. Configure WiFi

Edit `main.c` before building:

```c
#define WIFI_SSID "your_network"
#define WIFI_PASSWORD "your_password"
```

## OTA Update Process

### Web Interface Method

1. Build new firmware version
2. File to upload: `build/pybricks_picow_fota_image_encrypted.bin`
3. Navigate to `http://[device_ip]/`
4. Click "OTA Firmware Update" section
5. Select `.bin` file
6. Click "Upload & Install"
7. Device verifies SHA256, then reboots
8. Bootloader swaps firmware and boots new version

### Rollback Protection

The bootloader has automatic rollback:

- New firmware must call `ota_commit_firmware()` after successful boot
- If device crashes before commit, next boot rolls back to old firmware
- Main.c commits after 5 seconds if everything works

## File Descriptions

### ota_handler.c/h
- `ota_begin()` - Initialize download slot
- `ota_write()` - Write firmware chunks (auto-aligned to 256 bytes)
- `ota_end()` - Verify SHA256, mark slot valid
- `ota_perform_update()` - Trigger reboot/swap
- `ota_commit_firmware()` - Prevent rollback
- `ota_is_after_update()` - Check if just updated
- `ota_is_after_rollback()` - Check if rolled back

### httpd_content_ota.c
- Web interface with file upload form
- JavaScript XHR upload with progress bar
- SSI tag <!--#version--> shows firmware version

### main_ota.c
- HTTP POST handlers for `/upload` endpoint
- Receives multipart/form-data
- Streams data to flash via `ota_write()`
- Commits firmware after successful boot

### CMakeLists_OTA.txt
- Links `pico_fota_bootloader_lib`
- Calls `pfb_compile_with_bootloader()` (sets linker script)
- Defines `PFB_AES_KEY` for encryption
- Generates extra outputs: `.bin`, `.uf2`, `_fota_image_encrypted.bin`

## Build Outputs

After successful build:

```
build/
├── pico_fota_bootloader/
│   ├── pico_fota_bootloader.uf2    (flash once via USB)
│   └── pico_fota_bootloader.elf
└── pybricks_picow.uf2               (initial app via USB)
    pybricks_picow.bin
    pybricks_picow_fota_image.bin
    pybricks_picow_fota_image_encrypted.bin  (upload via web)
```

## Features

- **SHA256 verification** - Detects corrupted downloads
- **AES-ECB encryption** - Protects firmware in transit
- **Automatic rollback** - Reverts to old firmware if new one crashes
- **Web upload** - No USB needed after initial flash
- **Progress tracking** - Upload progress bar in web UI
- **Version display** - Shows current version + update status

## Troubleshooting

**Bootloader won't start app:**
- Check both bootloader and app built with same AES key
- Ensure app.uf2 was flashed after bootloader.uf2

**Upload fails:**
- Check file is `*_fota_image_encrypted.bin` (not .uf2)
- Ensure stable WiFi connection
- File size must be < 1004KB

**Device reboots but doesn't update:**
- Check serial output for SHA256 mismatch
- Rebuild with matching AES key
- Verify file integrity

**Device rolls back on every boot:**
- New firmware crashes before calling `ota_commit_firmware()`
- Check serial logs for error messages
- Old firmware automatically restored

## Security Notes

- Change default AES key in production
- Store AES key securely (not in git)
- Consider adding HTTPS for web interface
- Implement authentication for upload endpoint

## Version Management

Update version in CMakeLists.txt:

```cmake
target_compile_definitions(${OUTPUT_NAME} PRIVATE
    FIRMWARE_VERSION="1.0.1"
)
```

Version appears in web interface and bootloader logs.
