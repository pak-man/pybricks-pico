#ifndef MICROPY_INCLUDED_PICOW_MPCONFIGPORT_H
#define MICROPY_INCLUDED_PICOW_MPCONFIGPORT_H

#define MICROPY_HW_BOARD_NAME "Pybricks Pico W"
#define MICROPY_HW_MCU_NAME "RP2040"
#define PYBRICKS_HUB_NAME "picow"

// Minimal config for now
#define MICROPY_ALLOC_PATH_MAX (128)
#define MICROPY_ENABLE_GC (1)
#define MICROPY_FLOAT_IMPL (MICROPY_FLOAT_IMPL_FLOAT)

typedef long mp_int_t;
typedef unsigned long mp_uint_t;

#include <alloca.h>

#endif