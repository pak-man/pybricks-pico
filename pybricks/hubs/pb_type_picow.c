// SPDX-License-Identifier: MIT

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

typedef struct _hubs_PICOW_obj_t {
    mp_obj_base_t base;
    mp_obj_t system;
} hubs_PICo_obj_t;

static mp_obj_t hubs_PICOW_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    hubs_PICOW_obj_t *self = mp_obj_malloc(hubs_PICOW_obj_t, type);
    self->system = MP_OBJ_FROM_PTR(&pb_type_System);
    return MP_OBJ_FROM_PTR(self);
}

static const pb_attr_dict_entry_t hubs_PICOW_attr_dict[] = {
    PB_DEFINE_CONST_ATTR_RO(MP_QSTR_system, hubs_PICOW_obj_t, system),
    PB_ATTR_DICT_SENTINEL
};

MP_DEFINE_CONST_OBJ_TYPE(pb_type_ThisHub,
    PYBRICKS_HUB_CLASS_NAME,
    MP_TYPE_FLAG_NONE,
    make_new, hubs_PICOW_make_new,
    attr, pb_attribute_handler,
    protocol, hubs_PICOW_attr_dict);

#endif // PYBRICKS_PY_HUBS && PYBRICKS_HUB_PICOW
