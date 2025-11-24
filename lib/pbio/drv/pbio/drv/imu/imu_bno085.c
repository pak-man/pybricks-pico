// SPDX-License-Identifier: MIT
// BNO085 IMU driver for Pico W / Pico 2 W
// Based on Hillcrest BNO080/085 SH-2 protocol

#include <pbdrv/config.h>

#if PBDRV_CONFIG_IMU_BNO085

#include <string.h>
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "imu_bno085.h"

// Include storage for calibration persistence
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
#include "../storage/storage_flash_rp2040.h"
#endif

// ============================================================================
// BNO085 SH-2 Protocol Constants
// ============================================================================

// Channels
#define CHANNEL_COMMAND         0
#define CHANNEL_EXECUTABLE      1
#define CHANNEL_CONTROL         2
#define CHANNEL_REPORTS         3
#define CHANNEL_WAKE_REPORTS    4
#define CHANNEL_GYRO            5

// Report IDs
#define REPORT_ACCELEROMETER            0x01
#define REPORT_GYROSCOPE                0x02
#define REPORT_MAGNETIC_FIELD           0x03
#define REPORT_LINEAR_ACCELERATION      0x04
#define REPORT_ROTATION_VECTOR          0x05
#define REPORT_GRAVITY                  0x06
#define REPORT_GAME_ROTATION_VECTOR     0x08
#define REPORT_GEOMAGNETIC_ROTATION     0x09
#define REPORT_GYRO_INTEGRATED_RV       0x2A

// Commands
#define CMD_ERRORS                      0x01
#define CMD_COUNTER                     0x02
#define CMD_TARE                        0x03
#define CMD_INITIALIZE                  0x04
#define CMD_SAVE_DCD                    0x06
#define CMD_ME_CALIBRATE                0x07
#define CMD_DCD_PERIOD_SAVE             0x09
#define CMD_GET_OSCILLATOR_TYPE         0x0A
#define CMD_CLEAR_DCD_AND_RESET         0x0B

// ============================================================================
// Internal state
// ============================================================================

static struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    bno085_data_t data;
    bool initialized;
    uint32_t sequence_number[6];  // Per channel
    uint16_t packet_size;
} bno085_state = {0};

// ============================================================================
// Low-level I2C communication
// ============================================================================

// Read packet header (first 4 bytes)
static int read_header(uint16_t *length, uint8_t *channel, uint8_t *seq) {
    uint8_t header[4];
    int ret = i2c_read_blocking(bno085_state.i2c, bno085_state.addr, header, 4, false);
    if (ret != 4) return -1;
    
    *length = (header[1] << 8) | header[0];
    *length &= 0x7FFF;  // Clear continuation bit
    *channel = header[2];
    *seq = header[3];
    return 0;
}

// Read SHTP packet
static int read_packet(uint8_t *buffer, uint16_t max_len, uint8_t *channel) {
    uint16_t length;
    uint8_t seq;
    
    if (read_header(&length, channel, &seq) != 0) return -1;
    if (length == 0 || length > max_len) return -1;
    
    // Read payload (length includes header)
    uint16_t payload_len = length - 4;
    int ret = i2c_read_blocking(bno085_state.i2c, bno085_state.addr, buffer, payload_len, false);
    return (ret == payload_len) ? payload_len : -1;
}

// Send SHTP packet
static int send_packet(uint8_t channel, const uint8_t *data, uint16_t length) {
    uint8_t buffer[length + 4];
    uint16_t total_len = length + 4;
    
    // Build header
    buffer[0] = total_len & 0xFF;
    buffer[1] = (total_len >> 8) & 0xFF;
    buffer[2] = channel;
    buffer[3] = bno085_state.sequence_number[channel]++;
    
    memcpy(buffer + 4, data, length);
    
    int ret = i2c_write_blocking(bno085_state.i2c, bno085_state.addr, buffer, total_len, false);
    return (ret == total_len) ? 0 : -1;
}

// ============================================================================
// Command functions
// ============================================================================

// Enable feature report
static int enable_report(uint8_t report_id, uint32_t interval_us) {
    uint8_t cmd[17] = {0};
    cmd[0] = 0x21;  // Set feature command
    cmd[1] = report_id;
    cmd[2] = 0;  // Feature flags
    cmd[3] = 0;  // Change sensitivity (relative)
    cmd[4] = 0;  // Change sensitivity (relative)
    
    // Report interval (microseconds)
    cmd[5] = interval_us & 0xFF;
    cmd[6] = (interval_us >> 8) & 0xFF;
    cmd[7] = (interval_us >> 16) & 0xFF;
    cmd[8] = (interval_us >> 24) & 0xFF;
    
    return send_packet(CHANNEL_CONTROL, cmd, 17);
}

// ============================================================================
// Report parsing
// ============================================================================

// Parse rotation vector report (quaternion)
static void parse_rotation_vector(const uint8_t *data, uint16_t len) {
    if (len < 14) return;
    
    // Q1.14 fixed point (divide by 16384)
    int16_t quat_i = (data[5] << 8) | data[4];
    int16_t quat_j = (data[7] << 8) | data[6];
    int16_t quat_k = (data[9] << 8) | data[8];
    int16_t quat_real = (data[11] << 8) | data[10];
    int16_t accuracy = (data[13] << 8) | data[12];  // Radians, Q1.12
    
    bno085_state.data.rotation.x = quat_i / 16384.0f;
    bno085_state.data.rotation.y = quat_j / 16384.0f;
    bno085_state.data.rotation.z = quat_k / 16384.0f;
    bno085_state.data.rotation.w = quat_real / 16384.0f;
    
    // Convert to euler angles
    bno085_state.data.heading = pbdrv_imu_quaternion_to_euler(bno085_state.data.rotation);
    bno085_state.data.data_valid = true;
}

// Parse linear acceleration report
static void parse_acceleration(const uint8_t *data, uint16_t len) {
    if (len < 10) return;
    
    // Q8.8 fixed point (m/s²)
    int16_t x = (data[5] << 8) | data[4];
    int16_t y = (data[7] << 8) | data[6];
    int16_t z = (data[9] << 8) | data[8];
    
    bno085_state.data.acceleration.x = x / 256.0f;
    bno085_state.data.acceleration.y = y / 256.0f;
    bno085_state.data.acceleration.z = z / 256.0f;
}

// Parse gyroscope report
static void parse_gyro(const uint8_t *data, uint16_t len) {
    if (len < 10) return;
    
    // Q10.6 fixed point (rad/s)
    int16_t x = (data[5] << 8) | data[4];
    int16_t y = (data[7] << 8) | data[6];
    int16_t z = (data[9] << 8) | data[8];
    
    bno085_state.data.gyro.x = x / 64.0f;
    bno085_state.data.gyro.y = y / 64.0f;
    bno085_state.data.gyro.z = z / 64.0f;
}

// Parse gravity report
static void parse_gravity(const uint8_t *data, uint16_t len) {
    if (len < 10) return;
    
    // Q8.8 fixed point (m/s²)
    int16_t x = (data[5] << 8) | data[4];
    int16_t y = (data[7] << 8) | data[6];
    int16_t z = (data[9] << 8) | data[8];
    
    bno085_state.data.gravity.x = x / 256.0f;
    bno085_state.data.gravity.y = y / 256.0f;
    bno085_state.data.gravity.z = z / 256.0f;
}

// Parse magnetometer report
static void parse_magnetometer(const uint8_t *data, uint16_t len) {
    if (len < 10) return;
    
    // Q4.12 fixed point (µT)
    int16_t x = (data[5] << 8) | data[4];
    int16_t y = (data[7] << 8) | data[6];
    int16_t z = (data[9] << 8) | data[8];
    
    bno085_state.data.magnetometer.x = x / 4096.0f;
    bno085_state.data.magnetometer.y = y / 4096.0f;
    bno085_state.data.magnetometer.z = z / 4096.0f;
}

// ============================================================================
// Initialization
// ============================================================================

int pbdrv_imu_bno085_init(void *i2c, uint8_t sda_pin, uint8_t scl_pin, uint8_t addr) {
    bno085_state.i2c = (i2c_inst_t *)i2c;
    bno085_state.addr = addr;
    
    // Initialize I2C
    i2c_init(bno085_state.i2c, 400000);  // 400 kHz
    gpio_set_function(sda_pin, GPIO_FUNC_I2C);
    gpio_set_function(scl_pin, GPIO_FUNC_I2C);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);
    
    sleep_ms(100);  // Wait for BNO085 to boot
    
    // Soft reset
    uint8_t reset_cmd = 1;
    send_packet(CHANNEL_EXECUTABLE, &reset_cmd, 1);
    sleep_ms(500);
    
    // Enable reports (100 Hz default)
    uint32_t interval_us = 10000;  // 100 Hz
    enable_report(REPORT_ROTATION_VECTOR, interval_us);
    enable_report(REPORT_LINEAR_ACCELERATION, interval_us);
    enable_report(REPORT_GYROSCOPE, interval_us);
    enable_report(REPORT_GRAVITY, interval_us);
    enable_report(REPORT_MAGNETIC_FIELD, interval_us);
    
    bno085_state.initialized = true;
    printf("BNO085: Initialized on I2C 0x%02X\n", addr);
    
    // Try to load calibration
    #if PBDRV_CONFIG_STORAGE_FLASH_RP2040
    pbdrv_imu_bno085_load_calibration();
    #endif
    
    return 0;
}

int pbdrv_imu_bno085_reset(void) {
    if (!bno085_state.initialized) return -1;
    
    uint8_t reset_cmd = 1;
    return send_packet(CHANNEL_EXECUTABLE, &reset_cmd, 1);
}

// ============================================================================
// Data retrieval
// ============================================================================

int pbdrv_imu_bno085_update(void) {
    if (!bno085_state.initialized) return -1;
    
    uint8_t buffer[128];
    uint8_t channel;
    
    // Read all available reports
    for (int i = 0; i < 10; i++) {
        int len = read_packet(buffer, sizeof(buffer), &channel);
        if (len <= 0) break;
        
        if (channel == CHANNEL_REPORTS || channel == CHANNEL_WAKE_REPORTS) {
            uint8_t report_id = buffer[0];
            
            switch (report_id) {
                case REPORT_ROTATION_VECTOR:
                case REPORT_GAME_ROTATION_VECTOR:
                    parse_rotation_vector(buffer, len);
                    break;
                case REPORT_LINEAR_ACCELERATION:
                    parse_acceleration(buffer, len);
                    break;
                case REPORT_GYROSCOPE:
                    parse_gyro(buffer, len);
                    break;
                case REPORT_GRAVITY:
                    parse_gravity(buffer, len);
                    break;
                case REPORT_MAGNETIC_FIELD:
                    parse_magnetometer(buffer, len);
                    break;
            }
        }
    }
    
    return 0;
}

const bno085_data_t *pbdrv_imu_bno085_get_data(void) {
    return &bno085_state.data;
}

bno085_quaternion_t pbdrv_imu_bno085_get_rotation(void) {
    return bno085_state.data.rotation;
}

bno085_euler_t pbdrv_imu_bno085_get_heading(void) {
    return bno085_state.data.heading;
}

bno085_vec3_t pbdrv_imu_bno085_get_acceleration(void) {
    return bno085_state.data.acceleration;
}

bno085_vec3_t pbdrv_imu_bno085_get_gyro(void) {
    return bno085_state.data.gyro;
}

bno085_vec3_t pbdrv_imu_bno085_get_gravity(void) {
    return bno085_state.data.gravity;
}

float pbdrv_imu_bno085_get_heading_angle(void) {
    return bno085_state.data.heading.yaw;
}

float pbdrv_imu_bno085_get_tilt_angle(void) {
    float gx = bno085_state.data.gravity.x;
    float gy = bno085_state.data.gravity.y;
    float gz = bno085_state.data.gravity.z;
    
    float magnitude = sqrtf(gx*gx + gy*gy + gz*gz);
    if (magnitude < 0.1f) return 0.0f;
    
    // Angle from vertical (Z axis)
    return acosf(fabs(gz) / magnitude) * 180.0f / M_PI;
}

// ============================================================================
// Calibration
// ============================================================================

uint8_t pbdrv_imu_bno085_get_calibration(void) {
    return bno085_state.data.calibration_status;
}

bool pbdrv_imu_bno085_is_calibrated(void) {
    // Check if all sensors are calibrated (status 3)
    uint8_t status = bno085_state.data.calibration_status;
    return ((status & 0x03) == 3) &&  // Mag
           ((status & 0x0C) == 0x0C) &&  // Accel
           ((status & 0x30) == 0x30) &&  // Gyro
           ((status & 0xC0) == 0xC0);    // System
}

int pbdrv_imu_bno085_save_calibration(void) {
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
    uint8_t cmd[1] = {CMD_SAVE_DCD};
    int err = send_packet(CHANNEL_CONTROL, cmd, 1);
    if (err != 0) return err;
    
    sleep_ms(500);  // Wait for save
    
    // Also save to our flash storage
    return pbdrv_storage_flash_write("imu_cal", (uint8_t *)&bno085_state.data.calibration_status, 1);
#else
    return -1;
#endif
}

int pbdrv_imu_bno085_load_calibration(void) {
#if PBDRV_CONFIG_STORAGE_FLASH_RP2040
    uint8_t cal_data;
    uint32_t actual;
    int err = pbdrv_storage_flash_read("imu_cal", &cal_data, 1, &actual);
    if (err == 0 && actual == 1) {
        bno085_state.data.calibration_status = cal_data;
        printf("BNO085: Loaded calibration status: 0x%02X\n", cal_data);
        return 0;
    }
#endif
    return -1;
}

// ============================================================================
// Configuration
// ============================================================================

int pbdrv_imu_bno085_set_rate(uint16_t rate_hz) {
    uint32_t interval_us = 1000000 / rate_hz;
    
    enable_report(REPORT_ROTATION_VECTOR, interval_us);
    enable_report(REPORT_LINEAR_ACCELERATION, interval_us);
    enable_report(REPORT_GYROSCOPE, interval_us);
    enable_report(REPORT_GRAVITY, interval_us);
    enable_report(REPORT_MAGNETIC_FIELD, interval_us);
    
    return 0;
}

int pbdrv_imu_bno085_enable_rotation(bool enable) {
    return enable_report(REPORT_ROTATION_VECTOR, enable ? 10000 : 0);
}

int pbdrv_imu_bno085_enable_acceleration(bool enable) {
    return enable_report(REPORT_LINEAR_ACCELERATION, enable ? 10000 : 0);
}

int pbdrv_imu_bno085_enable_gyro(bool enable) {
    return enable_report(REPORT_GYROSCOPE, enable ? 10000 : 0);
}

int pbdrv_imu_bno085_enable_magnetometer(bool enable) {
    return enable_report(REPORT_MAGNETIC_FIELD, enable ? 10000 : 0);
}

// ============================================================================
// Utility functions
// ============================================================================

bno085_euler_t pbdrv_imu_quaternion_to_euler(bno085_quaternion_t q) {
    bno085_euler_t euler;
    
    // Roll (X-axis rotation)
    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    euler.roll = atan2f(sinr_cosp, cosr_cosp) * 180.0f / M_PI;
    
    // Pitch (Y-axis rotation)
    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (fabsf(sinp) >= 1)
        euler.pitch = copysignf(90.0f, sinp);
    else
        euler.pitch = asinf(sinp) * 180.0f / M_PI;
    
    // Yaw (Z-axis rotation)
    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    euler.yaw = atan2f(siny_cosp, cosy_cosp) * 180.0f / M_PI;
    
    // Normalize yaw to 0-360
    if (euler.yaw < 0) euler.yaw += 360.0f;
    
    return euler;
}

void pbdrv_imu_quaternion_to_matrix(bno085_quaternion_t q, float matrix[9]) {
    float xx = q.x * q.x;
    float yy = q.y * q.y;
    float zz = q.z * q.z;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float yz = q.y * q.z;
    float wx = q.w * q.x;
    float wy = q.w * q.y;
    float wz = q.w * q.z;
    
    matrix[0] = 1 - 2 * (yy + zz);
    matrix[1] = 2 * (xy - wz);
    matrix[2] = 2 * (xz + wy);
    matrix[3] = 2 * (xy + wz);
    matrix[4] = 1 - 2 * (xx + zz);
    matrix[5] = 2 * (yz - wx);
    matrix[6] = 2 * (xz - wy);
    matrix[7] = 2 * (yz + wx);
    matrix[8] = 1 - 2 * (xx + yy);
}

#endif // PBDRV_CONFIG_IMU_BNO085
