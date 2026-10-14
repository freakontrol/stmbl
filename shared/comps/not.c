#include "not_comp.h"
#include "hal.h"

/**
* ## Brief
* `not` is a logic inverter. F4 component, e.g. used to invert an enable input (`not0.in = io0.C12`, `fault0.en = not0.out`).
*/

HAL_COMP(not);

HAL_PIN(in);   // *input*, Logic input, true if > 0
HAL_PIN(out);  // *output*, 1 if in <= 0, else 0

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct not_pin_ctx_t *pins = (struct not_pin_ctx_t *)pin_ptr;

  PIN(out) = PIN(in) <= 0.0;
}

const hal_comp_t not_comp_struct = {
    .name      = "not",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .hw_init   = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct not_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
