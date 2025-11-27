#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "../../lib/pbio/drv/imu/imu_bno085.h"
#include "../../lib/pbio/drv/storage/storage.h"
#include "../../lib/pbio/drv/storage/storage_flash_rp2040.h"
#include "../../lib/pbio/drv/storage/storage_sd_rp2040.h"
#include "../../lib/pbio/drv/battery/battery_rp2040.h"
#ifdef PICO_2W
#include "../../lib/pbio/drv/motion_dsp/dsp_verification.h"
#include "../../lib/pbio/drv/motion_dsp/encoder_filter_dsp.h"
#endif

extern bool nus_is_ready(void);
extern void nus_send_data(const uint8_t *data, uint16_t length);

// Helper to send formatted response
static void send_response(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (nus_is_ready() && len > 0) {
        nus_send_data((uint8_t*)buffer, len);
    }
    printf("%s", buffer);
}

// Test LED (CYW43 onboard)
void test_led(const char *args) {
    extern void cyw43_arch_gpio_put(uint32_t pin, bool on);
    
    if (strcmp(args, "ON") == 0) {
        cyw43_arch_gpio_put(0, true);
        send_response("LED: ON\n");
    } else if (strcmp(args, "OFF") == 0) {
        cyw43_arch_gpio_put(0, false);
        send_response("LED: OFF\n");
    } else if (strcmp(args, "BLINK") == 0) {
        for (int i = 0; i < 5; i++) {
            cyw43_arch_gpio_put(0, true);
            sleep_ms(200);
            cyw43_arch_gpio_put(0, false);
            sleep_ms(200);
        }
        send_response("LED: Blink test complete\n");
    } else {
        send_response("LED usage: LED ON|OFF|BLINK\n");
    }
}

// Test IMU
void test_imu(const char *args) {
    if (strcmp(args, "STATUS") == 0) {
        bno085_calibration_t cal = pbdrv_imu_bno085_get_calibration();
        bool calibrated = pbdrv_imu_bno085_is_calibrated();
        send_response("IMU Cal: Mag=%d Accel=%d Gyro=%d System=%d %s\n",
            cal.mag, cal.accel, cal.gyro, cal.system,
            calibrated ? "[CALIBRATED]" : "[UNCALIBRATED]");
            
    } else if (strcmp(args, "DATA") == 0) {
        bno085_euler_t euler = pbdrv_imu_bno085_get_heading();
        bno085_vec3_t accel = pbdrv_imu_bno085_get_acceleration();
        bno085_vec3_t gyro = pbdrv_imu_bno085_get_gyro();
        float tilt = pbdrv_imu_bno085_get_tilt_angle();
        float heading = pbdrv_imu_bno085_get_heading_angle();
        
        send_response("IMU Data:\n");
        send_response("  Heading: %.1f° (Yaw=%.1f Roll=%.1f Pitch=%.1f)\n",
            heading, euler.yaw, euler.roll, euler.pitch);
        send_response("  Tilt: %.1f°\n", tilt);
        send_response("  Accel: X=%.2f Y=%.2f Z=%.2f m/s²\n",
            accel.x, accel.y, accel.z);
        send_response("  Gyro: X=%.2f Y=%.2f Z=%.2f rad/s\n",
            gyro.x, gyro.y, gyro.z);
            
    } else if (strcmp(args, "SAVECAL") == 0) {
        if (pbdrv_imu_bno085_save_calibration() == 0) {
            send_response("IMU: Calibration saved to flash\n");
        } else {
            send_response("IMU: Failed to save calibration\n");
        }
        
    } else if (strcmp(args, "LOADCAL") == 0) {
        if (pbdrv_imu_bno085_load_calibration() == 0) {
            send_response("IMU: Calibration loaded from flash\n");
        } else {
            send_response("IMU: No calibration found\n");
        }
        
    } else if (strcmp(args, "RESET") == 0) {
        pbdrv_imu_bno085_reset();
        send_response("IMU: Reset complete\n");
        
    } else {
        send_response("IMU usage: IMU STATUS|DATA|SAVECAL|LOADCAL|RESET\n");
    }
}

// Test flash storage
void test_flash(const char *args) {
    if (strcmp(args, "WRITE") == 0) {
        const char *test_data = "Hello from flash storage!";
        if (pbdrv_storage_flash_write("test.txt", (uint8_t*)test_data, strlen(test_data)) == 0) {
            send_response("Flash: Wrote 'test.txt' (%d bytes)\n", strlen(test_data));
        } else {
            send_response("Flash: Write failed\n");
        }
        
    } else if (strcmp(args, "READ") == 0) {
        uint8_t buffer[128];
        uint32_t size;
        if (pbdrv_storage_flash_read("test.txt", buffer, sizeof(buffer), &size) == 0) {
            buffer[size] = '\0';
            send_response("Flash: Read 'test.txt' (%d bytes): %s\n", size, buffer);
        } else {
            send_response("Flash: Read failed or file not found\n");
        }
        
    } else if (strcmp(args, "DELETE") == 0) {
        if (pbdrv_storage_flash_delete("test.txt") == 0) {
            send_response("Flash: Deleted 'test.txt'\n");
        } else {
            send_response("Flash: Delete failed\n");
        }
        
    } else if (strcmp(args, "LIST") == 0) {
        send_response("Flash: Listing files...\n");
        // Note: storage_flash driver doesn't have list function yet
        // Check if test.txt exists
        if (pbdrv_storage_flash_exists("test.txt")) {
            uint32_t size;
            pbdrv_storage_flash_size("test.txt", &size);
            send_response("  test.txt (%d bytes)\n", size);
        }
        if (pbdrv_storage_flash_exists("imu_cal")) {
            uint32_t size;
            pbdrv_storage_flash_size("imu_cal", &size);
            send_response("  imu_cal (%d bytes)\n", size);
        }
        
    } else if (strncmp(args, "SETTING ", 8) == 0) {
        // SETTING key=value or SETTING key
        const char *keyval = args + 8;
        char key[17], value[33];
        
        if (strchr(keyval, '=')) {
            // Set setting
            sscanf(keyval, "%16[^=]=%32s", key, value);
            if (pbdrv_storage_setting_set(key, (uint8_t*)value, strlen(value)) == 0) {
                send_response("Flash: Set '%s' = '%s'\n", key, value);
            } else {
                send_response("Flash: Failed to set '%s'\n", key);
            }
        } else {
            // Get setting
            strncpy(key, keyval, sizeof(key) - 1);
            key[sizeof(key) - 1] = '\0';
            uint8_t val_buf[33];
            uint32_t size;
            if (pbdrv_storage_setting_get(key, val_buf, sizeof(val_buf) - 1, &size) == 0) {
                val_buf[size] = '\0';
                send_response("Flash: Get '%s' = '%s'\n", key, val_buf);
            } else {
                send_response("Flash: Setting '%s' not found\n", key);
            }
        }
        
    } else if (strcmp(args, "FORMAT") == 0) {
        send_response("Flash: Formatting... (this will erase all data!)\n");
        if (pbdrv_storage_flash_format() == 0) {
            send_response("Flash: Format complete\n");
        } else {
            send_response("Flash: Format failed\n");
        }
        
    } else {
        send_response("Flash usage:\n");
        send_response("  FLASH WRITE|READ|DELETE|LIST|FORMAT\n");
        send_response("  FLASH SETTING key=value (set)\n");
        send_response("  FLASH SETTING key (get)\n");
    }
}

// Test SD card storage
void test_sd(const char *args) {
    if (strcmp(args, "STATUS") == 0) {
        if (pbdrv_storage_sd_is_present()) {
            uint64_t capacity = pbdrv_storage_sd_get_capacity();
            send_response("SD: Present, Capacity=%llu MB\n", capacity / (1024*1024));
        } else {
            send_response("SD: Not detected\n");
        }
        
    } else if (strcmp(args, "WRITE") == 0) {
        uint8_t test_data[512];
        for (int i = 0; i < 512; i++) {
            test_data[i] = i & 0xFF;
        }
        
        if (pbdrv_storage_sd_write_block(100, test_data) == 0) {
            send_response("SD: Wrote test pattern to sector 100\n");
        } else {
            send_response("SD: Write failed\n");
        }
        
    } else if (strcmp(args, "READ") == 0) {
        uint8_t buffer[512];
        if (pbdrv_storage_sd_read_block(100, buffer) == 0) {
            send_response("SD: Read sector 100, first 32 bytes:\n");
            for (int i = 0; i < 32; i++) {
                send_response("%02X ", buffer[i]);
                if ((i + 1) % 16 == 0) send_response("\n");
            }
        } else {
            send_response("SD: Read failed\n");
        }
        
    } else if (strcmp(args, "VERIFY") == 0) {
        uint8_t buffer[512];
        if (pbdrv_storage_sd_read_block(100, buffer) == 0) {
            bool ok = true;
            for (int i = 0; i < 512; i++) {
                if (buffer[i] != (i & 0xFF)) {
                    ok = false;
                    break;
                }
            }
            send_response("SD: Verify %s\n", ok ? "PASSED" : "FAILED");
        } else {
            send_response("SD: Read failed\n");
        }
        
    } else {
        send_response("SD usage: SD STATUS|WRITE|READ|VERIFY\n");
    }
}

// Test watchdog
void test_watchdog(const char *args) {
    if (strcmp(args, "STATUS") == 0) {
        bool enabled = watchdog_enable_caused_reboot();
        send_response("Watchdog: %s (last reboot cause: %s)\n",
            enabled ? "ENABLED" : "DISABLED",
            enabled ? "WATCHDOG" : "NORMAL");
            
    } else if (strcmp(args, "ENABLE") == 0) {
        watchdog_enable(5000, false); // 5 second timeout
        send_response("Watchdog: Enabled with 5s timeout\n");
        send_response("  Send 'WDT UPDATE' every 4s to prevent reset\n");
        
    } else if (strcmp(args, "UPDATE") == 0) {
        watchdog_update();
        send_response("Watchdog: Updated (timer reset)\n");
        
    } else if (strcmp(args, "TEST") == 0) {
        send_response("Watchdog: Enabling 2s timeout without updates...\n");
        send_response("  Device will reset in 2 seconds!\n");
        watchdog_enable(2000, false);
        // Don't update - let it timeout
        
    } else {
        send_response("Watchdog usage: WDT STATUS|ENABLE|UPDATE|TEST\n");
    }
}

// Test battery monitoring
void test_battery(const char *args) {
    if (strcmp(args, "READ") == 0) {
        uint16_t voltage_mv = pbdrv_battery_get_voltage_now();
        uint16_t current_ma = pbdrv_battery_get_current_now();
        float voltage_v = voltage_mv / 1000.0f;
        
        send_response("Battery:\n");
        send_response("  Voltage: %d mV (%.2f V)\n", voltage_mv, voltage_v);
        send_response("  Current: %d mA\n", current_ma);
        
        // Battery level estimation (for LiPo, adjust for your battery)
        int percent = 0;
        if (voltage_mv >= 4100) percent = 100;
        else if (voltage_mv >= 3900) percent = 75;
        else if (voltage_mv >= 3700) percent = 50;
        else if (voltage_mv >= 3500) percent = 25;
        else percent = 0;
        
        send_response("  Level: %d%%\n", percent);
        
    } else {
        send_response("Battery usage: BAT READ\n");
    }
}

// Test DSP (Pico 2 W only)
void test_dsp(const char *args) {
#ifdef PICO_2W
    if (strcmp(args, "VERIFY") == 0) {
        send_response("DSP: Running verification...\n");
        dsp_verify_instructions();
        send_response("DSP: Verification complete (check serial output)\n");
        
    } else if (strcmp(args, "BENCH") == 0) {
        send_response("DSP: Running benchmark...\n");
        dsp_benchmark_result_t result = dsp_benchmark_multiply_accumulate(10000);
        
        send_response("DSP Benchmark Results:\n");
        send_response("  Scalar: %lu cycles\n", result.scalar_cycles);
        send_response("  DSP:    %lu cycles\n", result.dsp_cycles);
        send_response("  Speedup: %.2fx\n", result.speedup);
        
    } else {
        send_response("DSP usage: DSP VERIFY|BENCH\n");
    }
#else
    send_response("DSP: Not available on Pico W (RP2040)\n");
#endif
}

// Test motors
void test_motor(const char *args) {
    extern void pbdrv_pwm_set_duty_simple(uint8_t id, int16_t duty);
    extern int32_t pbdrv_counter_get_count_simple(uint8_t id);
    extern void pbdrv_counter_reset(uint8_t id);
    
    if (strncmp(args, "TEST ", 5) == 0) {
        // MOTOR TEST 0 - Test single motor
        int motor = atoi(args + 5);
        if (motor < 0 || motor >= NUM_MOTORS) {
            send_response("Motor: Invalid motor %d (0-%d)\n", motor, NUM_MOTORS-1);
            return;
        }
        
        send_response("Motor: Testing M%d...\n", motor);
        pbdrv_counter_reset(motor);
        
        // Forward
        pbdrv_pwm_set_duty_simple(motor, 5000);
        sleep_ms(1000);
        int32_t pos1 = pbdrv_counter_get_count_simple(motor);
        
        // Stop
        pbdrv_pwm_set_duty_simple(motor, 0);
        sleep_ms(500);
        
        // Reverse
        pbdrv_pwm_set_duty_simple(motor, -5000);
        sleep_ms(1000);
        int32_t pos2 = pbdrv_counter_get_count_simple(motor);
        
        // Stop
        pbdrv_pwm_set_duty_simple(motor, 0);
        
        send_response("Motor: M%d Forward=%ld Reverse=%ld\n", motor, pos1, pos2);
        
    } else if (strcmp(args, "ALL") == 0) {
        // MOTOR ALL - Test all motors
        send_response("Motor: Testing all %d motors...\n", NUM_MOTORS);
        
        for (int i = 0; i < NUM_MOTORS; i++) {
            pbdrv_counter_reset(i);
            pbdrv_pwm_set_duty_simple(i, 5000);
        }
        
        sleep_ms(2000);
        
        send_response("Motor: Positions: ");
        for (int i = 0; i < NUM_MOTORS; i++) {
            send_response("M%d=%ld ", i, pbdrv_counter_get_count_simple(i));
            pbdrv_pwm_set_duty_simple(i, 0);
        }
        send_response("\n");
        
    } else if (strncmp(args, "RUN ", 4) == 0) {
        // MOTOR RUN 0 5000 - Set motor duty
        int motor, duty;
        if (sscanf(args + 4, "%d %d", &motor, &duty) == 2) {
            if (motor >= 0 && motor < NUM_MOTORS) {
                pbdrv_pwm_set_duty_simple(motor, duty);
                send_response("Motor: M%d set to %d\n", motor, duty);
            } else {
                send_response("Motor: Invalid motor %d\n", motor);
            }
        } else {
            send_response("Motor: Usage: MOTOR RUN <id> <duty>\n");
        }
        
    } else if (strcmp(args, "STOP") == 0) {
        // MOTOR STOP - Stop all motors
        for (int i = 0; i < NUM_MOTORS; i++) {
            pbdrv_pwm_set_duty_simple(i, 0);
        }
        send_response("Motor: All stopped\n");
        
    } else if (strcmp(args, "COUNT") == 0) {
        // MOTOR COUNT - Show encoder counts
        send_response("Motor: Counts: ");
        for (int i = 0; i < NUM_MOTORS; i++) {
            send_response("M%d=%ld ", i, pbdrv_counter_get_count_simple(i));
        }
        send_response("\n");
        
    } else if (strcmp(args, "RESET") == 0) {
        // MOTOR RESET - Reset all encoders
        for (int i = 0; i < NUM_MOTORS; i++) {
            pbdrv_counter_reset(i);
        }
        send_response("Motor: All encoders reset\n");
        
    } else {
        send_response("Motor usage:\n");
        send_response("  MOTOR TEST <id>      Test single motor\n");
        send_response("  MOTOR ALL            Test all motors\n");
        send_response("  MOTOR RUN <id> <duty>  Set motor duty (-10000 to 10000)\n");
        send_response("  MOTOR STOP           Stop all motors\n");
        send_response("  MOTOR COUNT          Show encoder counts\n");
        send_response("  MOTOR RESET          Reset encoders\n");
    }
}

// System info
void test_system(const char *args) {
    if (strcmp(args, "INFO") == 0) {
        send_response("System Info:\n");
#ifdef PICO_2W
        send_response("  Board: Pico 2 W (RP2350B)\n");
        send_response("  Motors: 12\n");
#else
        send_response("  Board: Pico W (RP2040)\n");
        send_response("  Motors: 4\n");
#endif
        send_response("  Firmware: 1.0.0\n");
        send_response("  SDK: %s\n", PICO_SDK_VERSION_STRING);
        
    } else if (strcmp(args, "REBOOT") == 0) {
        send_response("System: Rebooting in 2 seconds...\n");
        sleep_ms(2000);
        watchdog_enable(1, false);
        while(1); // Wait for watchdog reset
        
    } else {
        send_response("System usage: SYS INFO|REBOOT\n");
    }
}

// Main test command parser
void system_test_parse_command(const char *cmd) {
    if (strncmp(cmd, "LED ", 4) == 0) {
        test_led(cmd + 4);
    } else if (strncmp(cmd, "IMU ", 4) == 0) {
        test_imu(cmd + 4);
    } else if (strncmp(cmd, "FLASH ", 6) == 0) {
        test_flash(cmd + 6);
    } else if (strncmp(cmd, "SD ", 3) == 0) {
        test_sd(cmd + 3);
    } else if (strncmp(cmd, "WDT ", 4) == 0) {
        test_watchdog(cmd + 4);
    } else if (strncmp(cmd, "BAT ", 4) == 0) {
        test_battery(cmd + 4);
    } else if (strncmp(cmd, "DSP ", 4) == 0) {
	#ifdef PICO_2W
            test_dsp(cmd + 4);
	#else
	    send_response("ERROR", "Requires Pico 2W");
	#endif
    } else if (strncmp(cmd, "MOTOR ", 6) == 0) {
        test_motor(cmd + 6);
    } else if (strncmp(cmd, "SYS ", 4) == 0) {
        test_system(cmd + 4);
    } else if (strncmp(cmd, "FILTER TEST", 11) == 0) {
	#ifdef PICO_2W
    	    printf("Running encoder filter test suite...\n");
	    encoder_filter_run_tests();
	    send_response("FILTER TEST", "Complete - check serial");
	#else
	    send_response("ERROR", "Requires Pico 2W");
	#endif
    } else if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0) {
        send_response("\nAvailable test commands:\n");
        send_response("  LED ON|OFF|BLINK\n");
        send_response("  IMU STATUS|DATA|SAVECAL|LOADCAL|RESET\n");
        send_response("  FLASH WRITE|READ|DELETE|LIST|FORMAT\n");
        send_response("  FLASH SETTING key=value | key\n");
        send_response("  SD STATUS|WRITE|READ|VERIFY\n");
        send_response("  WDT STATUS|ENABLE|UPDATE|TEST\n");
        send_response("  BAT READ\n");
        send_response("  DSP VERIFY|BENCH (Pico 2W only)\n");
        send_response("  FILTER TEST    - Run encoder filter tests (DSP)\n");
        send_response("  MOTOR TEST <id>|ALL|RUN <id> <duty>|STOP|COUNT|RESET\n");
        send_response("  SYS INFO|REBOOT\n");
        send_response("  HELP or ? - This help\n");
    }
}