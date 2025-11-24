// SPDX-License-Identifier: MIT
// BNO085 IMU driver for Pico W / Pico 2 W
// 9-axis sensor fusion with quaternions, euler angles, gravity vector

#ifndef _PBDRV_IMU_BNO085_H_
#define _PBDRV_IMU_BNO085_H_

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// BNO085 I2C Configuration
// ============================================================================

#define BNO085_I2C_ADDR_DEFAULT     0x4A
#define BNO085_I2C_ADDR_ALT         0x4B

// ============================================================================
// Data structures
// ============================================================================

// Quaternion (rotation representation)
typedef struct {
    float w, x, y, z;
} bno085_quaternion_t;

// Euler angles (degrees)
typedef struct {
    float roll;   // Rotation around X axis
    float pitch;  // Rotation around Y axis  
    float yaw;    // Rotation around Z axis (heading)
} bno085_euler_t;

// 3D vector
typedef struct {
    float x, y, z;
} bno085_vec3_t;

// Complete IMU data
typedef struct {
    bno085_quaternion_t rotation;       // Rotation as quaternion
    bno085_euler_t heading;             // Euler angles
    bno085_vec3_t acceleration;         // Linear acceleration (m/s²)
    bno085_vec3_t gyro;                 // Angular velocity (rad/s)
    bno085_vec3_t magnetometer;         // Magnetic field (µT)
    bno085_vec3_t gravity;              // Gravity vector (m/s²)
    uint8_t calibration_status;         // 0-3 for each sensor
    bool data_valid;
} bno085_data_t;

// ============================================================================
// Initialization
// ============================================================================

// Initialize BNO085 on specified I2C bus
// i2c: I2C instance (i2c0 or i2c1)
// sda_pin: GPIO pin for SDA
// scl_pin: GPIO pin for SCL
// addr: I2C address (BNO085_I2C_ADDR_DEFAULT or BNO085_I2C_ADDR_ALT)
int pbdrv_imu_bno085_init(void *i2c, uint8_t sda_pin, uint8_t scl_pin, uint8_t addr);

// Reset and recalibrate IMU
int pbdrv_imu_bno085_reset(void);

// ============================================================================
// Data retrieval
// ============================================================================

// Update IMU data (call periodically, e.g. 100 Hz)
int pbdrv_imu_bno085_update(void);

// Get complete IMU data
const bno085_data_t *pbdrv_imu_bno085_get_data(void);

// Get rotation quaternion
bno085_quaternion_t pbdrv_imu_bno085_get_rotation(void);

// Get euler angles (roll, pitch, yaw in degrees)
bno085_euler_t pbdrv_imu_bno085_get_heading(void);

// Get linear acceleration (gravity removed, m/s²)
bno085_vec3_t pbdrv_imu_bno085_get_acceleration(void);

// Get angular velocity (rad/s)
bno085_vec3_t pbdrv_imu_bno085_get_gyro(void);

// Get gravity vector (m/s²)
bno085_vec3_t pbdrv_imu_bno085_get_gravity(void);

// Get heading angle (yaw, 0-360°, 0=north)
float pbdrv_imu_bno085_get_heading_angle(void);

// Get tilt angle from vertical (0=upright, 90=horizontal, 180=upside down)
float pbdrv_imu_bno085_get_tilt_angle(void);

// ============================================================================
// Calibration
// ============================================================================

// Get calibration status (0=uncalibrated, 3=fully calibrated)
// Returns: bits 0-1=mag, 2-3=accel, 4-5=gyro, 6-7=system
uint8_t pbdrv_imu_bno085_get_calibration(void);

// Check if IMU is fully calibrated
bool pbdrv_imu_bno085_is_calibrated(void);

// Save calibration to flash (persistent across reboots)
int pbdrv_imu_bno085_save_calibration(void);

// Load calibration from flash
int pbdrv_imu_bno085_load_calibration(void);

// ============================================================================
// Configuration
// ============================================================================

// Set report rate (Hz) - default is 100 Hz
int pbdrv_imu_bno085_set_rate(uint16_t rate_hz);

// Enable/disable specific sensors
int pbdrv_imu_bno085_enable_rotation(bool enable);
int pbdrv_imu_bno085_enable_acceleration(bool enable);
int pbdrv_imu_bno085_enable_gyro(bool enable);
int pbdrv_imu_bno085_enable_magnetometer(bool enable);

// ============================================================================
// Utility functions
// ============================================================================

// Convert quaternion to euler angles
bno085_euler_t pbdrv_imu_quaternion_to_euler(bno085_quaternion_t q);

// Get rotation matrix from quaternion (3x3, row-major)
void pbdrv_imu_quaternion_to_matrix(bno085_quaternion_t q, float matrix[9]);

#endif // _PBDRV_IMU_BNO085_H_
