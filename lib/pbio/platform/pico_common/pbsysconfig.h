// SPDX-License-Identifier: MIT
// System configuration for Pico W / Pico 2 W

#ifndef _PBSYSCONFIG_H_
#define _PBSYSCONFIG_H_

// Hub identification
#ifdef PICO_2W
#define PBSYS_CONFIG_HUB_PICO2W     (1)
#define PBSYS_CONFIG_HUB_PICOW      (0)
#else
#define PBSYS_CONFIG_HUB_PICOW      (1)
#define PBSYS_CONFIG_HUB_PICO2W     (0)
#endif

// Minimal config for standalone motor control
#define PBSYS_CONFIG_MAIN           (0)
#define PBSYS_CONFIG_BLUETOOTH      (0)

#endif // _PBSYSCONFIG_H_
