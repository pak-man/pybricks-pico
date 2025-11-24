// SPDX-License-Identifier: MIT
// Pico W Hub type for Pybricks

#include "py/mpconfig.h"

#if PYBRICKS_PY_HUBS && PYBRICKS_HUB_PICOW

#include <pbio/util.h>

#include <pybricks/common.h>
#include <pybricks/hubs.h>

#include <pybricks/util_mp/pb_obj_helper.h>
#include <pybricks/util_pb/pb_error.h>

#include "py/misc.h"
#include "py/obj.h"
#include "py/runtime.h"

#include "pico/cyw43_arch.h"

typedef struct _hubs_PICOW_obj_t {
    mp_obj_base_t base;
    mp_obj_t system;
} hubs_PICOW_obj_t;

// LED control methods
static mp_obj_t hubs_PICOW_led_on(mp_obj_t self_in) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(hubs_PICOW_led_on_obj, hubs_PICOW_led_on);

static mp_obj_t hubs_PICOW_led_off(mp_obj_t self_in) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(hubs_PICOW_led_off_obj, hubs_PICOW_led_off);

static mp_obj_t hubs_PICOW_led_toggle(mp_obj_t self_in) {
    static bool led_state = false;
    led_state = !led_state;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
    return mp_const_none;
}
static MP_DEFINE_CONST_FUN_OBJ_1(hubs_PICOW_led_toggle_obj, hubs_PICOW_led_toggle);

static mp_obj_t hubs_PICOW_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    hubs_PICOW_obj_t *self = mp_obj_malloc(hubs_PICOW_obj_t, type);
    self->system = MP_OBJ_FROM_PTR(&pb_type_System);
    return MP_OBJ_FROM_PTR(self);
}

static const mp_rom_map_elem_t hubs_PICOW_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_system),     MP_ROM_PTR(&pb_type_System) },
    { MP_ROM_QSTR(MP_QSTR_led_on),     MP_ROM_PTR(&hubs_PICOW_led_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_led_off),    MP_ROM_PTR(&hubs_PICOW_led_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_led_toggle), MP_ROM_PTR(&hubs_PICOW_led_toggle_obj) },
};
static MP_DEFINE_CONST_DICT(hubs_PICOW_locals_dict, hubs_PICOW_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(pb_type_ThisHub,
    PYBRICKS_HUB_CLASS_NAME,
    MP_TYPE_FLAG_NONE,
    make_new, hubs_PICOW_make_new,
    locals_dict, &hubs_PICOW_locals_dict);

#endif // PYBRICKS_PY_HUBS && PYBRICKS_HUB_PICOW
