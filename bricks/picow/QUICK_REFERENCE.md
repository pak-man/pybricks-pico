# Pybricks Hub - Quick Command Reference

## Essential Commands
```
HELP or ?           Show all commands
SYS INFO            System information
SYS REBOOT          Restart hub
```

## LED
```
LED ON | OFF | BLINK
```

## IMU (BNO085)
```
IMU STATUS          Calibration status
IMU DATA            Full sensor data
IMU SAVECAL         Save calibration
IMU LOADCAL         Load calibration
```

## Storage
```
FLASH WRITE | READ | DELETE | LIST | FORMAT
FLASH SETTING key=value    Set
FLASH SETTING key          Get

SD STATUS | WRITE | READ | VERIFY
```

## Watchdog
```
WDT STATUS | ENABLE | UPDATE | TEST
```

## Battery
```
BAT READ            Voltage, current, %
```

## Motors
```
T0-T3 (or T0-T11)   Test motor
A                   Test all
M0+5000             50% forward
M0-3000             30% reverse
S                   Stop all
C                   Show counts
P0=90               Position (degrees)
V0=180              Velocity (deg/s)
R0                  Ramp test
```

## Status Messages (Auto-sent every 5s)
- Motor count
- IMU heading + calibration
- Connection status

## Pin Assignments

### Pico W (4 motors)
- I2C0: GP4(SDA), GP5(SCL) - IMU
- SPI0: GP16-19 - SD card
- Motors: GP2-3,6-19

### Pico 2 W (12 motors)
- I2C1: GP14(SDA), GP15(SCL) - IMU
- SPI0: GP16-19 - SD card
- Motors: GP0-47 (skip 14-15)

## Emergency
```
S                   Stop all motors
WDT TEST            Force reboot (2s)
```
