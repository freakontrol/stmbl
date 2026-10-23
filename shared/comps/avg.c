#include "avg_comp.h"
#include "hal.h"
#include "defines.h"

/**
* ## Brief
* `avg` scales and offsets a signal and low pass filters it (first order). F4 component, typically used to turn an analog spindle speed input into a velocity command, e.g. `avg0.mult = 0.1047` (rpm to rad/s), `ramp0.vel_ext_cmd = avg0.out`.
*
* ## Component Explanation
* 1. **Output** (`rt`):
* ```c
* out = LP(in * mult + offset)   // first order low pass at lpf Hz
* ```
* - Until `mult` is set the output is only `offset`.
*/

HAL_COMP(avg);

HAL_PIN(in);      // *input*, Signal
HAL_PIN(out);     // *output*, Filtered in * mult + offset
HAL_PIN(lpf);     // *parameter*, Filter cutoff (Hz), default 100
HAL_PIN(mult);    // *parameter*, Multiplier, no default (0)
HAL_PIN(offset);  // *parameter*, Offset added after mult, default 0

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct avg_ctx_t * ctx = (struct avg_ctx_t *)ctx_ptr;
  struct avg_pin_ctx_t *pins = (struct avg_pin_ctx_t *)pin_ptr;
  PIN(lpf)                   = 100;
  PIN(out)                   = 0.0;
  PIN(offset)                = 0.0;
}


static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct avg_pin_ctx_t *pins = (struct avg_pin_ctx_t *)pin_ptr;
  PIN(out)                   = (PIN(in) * PIN(mult) + PIN(offset)) * LP_HZ(PIN(lpf)) + (1.0 - LP_HZ(PIN(lpf))) * PIN(out);
}

hal_comp_t avg_comp_struct = {
    .name      = "avg",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct avg_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};