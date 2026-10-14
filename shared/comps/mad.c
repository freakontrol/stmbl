#include "mad_comp.h"
#include "hal.h"

/**
* ## Brief
* `mad` (multiply add) computes `out = in * mult + add`. F4 component, used e.g. in `conf/spindle_slip_uf.txt` to scale an analog input into a velocity command (`mad0.in = io0.in1`, `ramp0.vel_ext_cmd = mad0.out`).
*
* ## Component Explanation
* 1. **Output** (`rt`):
* - `out = in * mult + add`. All pins default to 0.
*/

HAL_COMP(mad);

HAL_PIN(in);    // *input*, Signal
HAL_PIN(mult);  // *parameter*, Multiplier
HAL_PIN(add);   // *parameter*, Offset

HAL_PIN(out);  // *output*, in * mult + add

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct mad_pin_ctx_t *pins = (struct mad_pin_ctx_t *)pin_ptr;

  PIN(out) = PIN(in) * PIN(mult) + PIN(add);
}

const hal_comp_t mad_comp_struct = {
    .name      = "mad",
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
    .pin_count = sizeof(struct mad_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
