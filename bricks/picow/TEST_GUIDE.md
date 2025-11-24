# Pybricks Hub System Test Guide

## Overview
Complete BLE-based testing for all subsystems: Motors, IMU, Storage (Flash + SD), Watchdog, Battery, and LED.

## Connection
1. Power on the Pico W/2W hub
2. Open nRF Connect app (iOS/Android) or nRF Connect desktop
3. Scan for "Pybricks Hub"
4. Connect and enable Nordic UART Service (NUS)
5. Send commands via RX characteristic

## Quick Start
Send: `HELP` or `?` to see all available commands

## Test Commands

### 1. LED Control
```
LED ON          - Turn LED on
LED OFF         - Turn LED off
LED BLINK       - Blink 5 times
```

### 2. IMU (BNO085)
```
IMU STATUS      - Show calibration status
IMU DATA        - Show all sensor data (heading, accel, gyro)
IMU SAVECAL     - Save calibration to flash
IMU LOADCAL     - Load calibration from flash
IMU RESET       - Reset IMU sensor
```

**Calibration Process:**
1. Send `IMU STATUS` - check calibration levels (0-3 for each sensor)
2. Move hub in figure-8 pattern (magnetometer)
3. Rotate around all axes (accelerometer)
4. Keep stationary briefly (gyroscope)
5. Wait until all sensors show level 3
6. Send `IMU SAVECAL` to save

### 3. Flash Storage
```
FLASH WRITE           - Write test file
FLASH READ            - Read test file
FLASH DELETE          - Delete test file
FLASH LIST            - List stored files
FLASH FORMAT          - Format flash (erases all!)
FLASH SETTING key=val - Set a setting
FLASH SETTING key     - Get a setting
```

**Examples:**
```
FLASH SETTING name=MyHub
FLASH SETTING name
FLASH SETTING speed=100
```

### 4. SD Card Storage
```
SD STATUS      - Check if SD card present, show capacity
SD WRITE       - Write test pattern to sector 100
SD READ        - Read sector 100
SD VERIFY      - Verify test pattern
```

**Expected Flow:**
1. `SD STATUS` → Should show capacity if card present
2. `SD WRITE` → Write test data
3. `SD READ` → Display first 32 bytes
4. `SD VERIFY` → Confirm data integrity

### 5. Watchdog Timer
```
WDT STATUS     - Check if enabled
WDT ENABLE     - Enable with 5s timeout
WDT UPDATE     - Reset timer (pet the dog)
WDT TEST       - Force reset in 2s
```

**Safety Test:**
```
WDT ENABLE     # Enable watchdog
# Wait 3 seconds
WDT UPDATE     # Should prevent reset
# Repeat WDT UPDATE every 4s to keep system alive
```

**Reset Test:**
```
WDT TEST       # Device will reset in 2s
# After reboot, send WDT STATUS to confirm watchdog reset
```

### 6. Battery Monitor
```
BAT READ       - Show voltage, current, estimated %
```

**Expected Output:**
```
Battery:
  Voltage: 4150 mV (4.15 V)
  Current: 120 mA
  Level: 100%
```

### 7. System Commands
```
SYS INFO       - Show system information
SYS REBOOT     - Reboot the hub
```

### 8. Motor Control (from motor_test.c)
```
T0-T3 / T0-T11 - Test single motor
A              - Test all motors
M0+5000        - Set motor 0 to 50% speed
M0-3000        - Set motor 0 to -30% speed
S              - Stop all motors
C              - Show encoder counts
P0=90          - Position control: motor 0 to 90°
V0=180         - Velocity control: motor 0 at 180°/s
R0             - Speed ramp test
```

## Complete Test Sequence

### Initial System Check
```
1. Connect via BLE
2. Send: SYS INFO
3. Verify: Board type, motor count, firmware version
4. Send: BAT READ
5. Verify: Battery voltage > 3.5V
```

### Storage Test
```
1. Send: SD STATUS
   → Should show card capacity or "Not detected"
   
2. Send: FLASH WRITE
   → "Flash: Wrote 'test.txt'"
   
3. Send: FLASH READ
   → Should show: "Hello from flash storage!"
   
4. Send: FLASH SETTING robot=pybricks
   → "Flash: Set 'robot' = 'pybricks'"
   
5. Send: FLASH SETTING robot
   → "Flash: Get 'robot' = 'pybricks'"
   
6. If SD card present:
   - Send: SD WRITE
   - Send: SD VERIFY
   → Should show "PASSED"
```

### IMU Test
```
1. Send: IMU STATUS
   → Check calibration: "Mag=X Accel=X Gyro=X System=X"
   
2. Send: IMU DATA
   → Should show heading, tilt, acceleration, gyro
   
3. Rotate hub 90° clockwise
   → Send: IMU DATA
   → Heading should change ~90°
   
4. Tilt hub forward/back
   → Send: IMU DATA
   → Pitch should change
   
5. If calibrated:
   Send: IMU SAVECAL
   → "IMU: Calibration saved to flash"
```

### LED Test
```
1. Send: LED ON
   → LED should light up
   
2. Send: LED OFF
   → LED should turn off
   
3. Send: LED BLINK
   → LED should blink 5 times
```

### Motor Test (Pico W example)
```
1. Send: T0
   → Motor 0 should run forward, then backward
   
2. Send: M0+5000
   → Motor 0 should run at 50% forward
   
3. Send: S
   → Motor 0 should stop
   
4. Send: C
   → Should show encoder counts for all motors
   
5. Send: P0=360
   → Motor 0 should rotate to 360° position
```

### Watchdog Test
```
1. Send: WDT STATUS
   → "Watchdog: DISABLED"
   
2. Send: WDT ENABLE
   → "Watchdog: Enabled with 5s timeout"
   
3. Wait 3 seconds, send: WDT UPDATE
   → "Watchdog: Updated (timer reset)"
   → Device should NOT reset
   
4. Send: WDT TEST
   → Device will reset in 2 seconds
   
5. After reboot, send: WDT STATUS
   → Should show: "last reboot cause: WATCHDOG"
```

## Troubleshooting

### IMU Not Responding
- Check I2C wiring (Pico W: GP4/GP5, Pico 2W: GP14/GP15)
- Verify BNO085 address is 0x4A
- Try: `IMU RESET`

### SD Card Not Detected
- Check SPI wiring: GP16(MISO), GP17(CS), GP18(SCK), GP19(MOSI)
- Ensure card is formatted (FAT32 recommended)
- Try reseating the card

### Flash Write Fails
- Check available space: `FLASH LIST`
- If full: `FLASH FORMAT` (WARNING: erases all data)

### Motor Not Moving
- Verify encoder wiring (check phase A/B)
- Verify PWM wiring
- Check power supply
- Send: `C` to verify encoder counts changing

### Battery Reading Zero
- Check ADC configuration in pbdrvconfig.h
- Verify voltage divider circuit
- Test with known voltage source

### BLE Connection Drops
- Check power supply (low voltage causes instability)
- Reduce distance to BLE central device
- Check for interference from motors

## Success Criteria

✅ **LED**: Responds to ON/OFF/BLINK commands  
✅ **IMU**: Returns valid heading/acceleration data  
✅ **Flash**: Can write/read/delete files and settings  
✅ **SD Card**: Detected, can read/write sectors  
✅ **Watchdog**: Can enable, update, and trigger reset  
✅ **Battery**: Returns valid voltage (3.0-4.2V for LiPo)  
✅ **Motors**: Respond to position/velocity commands  
✅ **BLE**: Stable connection, bidirectional communication  

## Notes

- All commands are case-sensitive
- Commands must end with newline (automatically added by most apps)
- Maximum command length: 128 characters
- Status messages sent automatically every 5 seconds when connected
- Watchdog auto-updates every 1 second when enabled
- IMU updates at 100 Hz in background
- Motor control runs at 1 kHz
