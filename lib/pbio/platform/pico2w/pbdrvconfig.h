// SPDX-License-Identifier: MIT
// pbio driver configuration for RP2350B (Pico 2 W)

#ifndef _PBDRVCONFIG_H_
#define _PBDRVCONFIG_H_

// Include DMA PID config
#include "pbdrvconfig_pid_dma.h"

// Platform
#define PBDRV_CONFIG_PICO2W                            (1)
#define PBDRV_CONFIG_RP2350B                           (1)

// Number of motors
#define PBDRV_CONFIG_MOTOR_DRIVER_NUM_DEV              (12)
#define PBDRV_CONFIG_COUNTER_NUM_DEV                   (12)
#define PBDRV_CONFIG_PWM_NUM_DEV                       (12)

// System clock rate
#define PBDRV_CONFIG_SYS_CLOCK_RATE                    (1000)

#endif // _PBDRVCONFIG_H_
