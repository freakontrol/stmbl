#include "scale_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `scale` multiplies one signal by `scale` and divides another one by it. F4 component; not used by any config in `conf/`.
*
* ## Component Explanation
* 1. **Output** (`rt`):
* - `out0 = in0 * scale`, `out1 = in1 / MAX(scale, 0.001)`.
* - There is no default, so `scale` is 0 until set.
*/

HAL_COMP(scale);

HAL_PIN(in0);    // *input*, Signal to multiply
HAL_PIN(out0);   // *output*, in0 * scale
HAL_PIN(in1);    // *input*, Signal to divide
HAL_PIN(out1);   // *output*, in1 / scale (scale clamped to >= 0.001)
HAL_PIN(scale);  // *parameter*, Scale factor

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct scale_ctx_t * ctx = (struct scale_ctx_t *)ctx_ptr;
  struct scale_pin_ctx_t *pins = (struct scale_pin_ctx_t *)pin_ptr;

  PIN(out0) = PIN(in0) * PIN(scale);
  PIN(out1) = PIN(in1) / MAX(PIN(scale), 0.001);
}

hal_comp_t scale_comp_struct = {
    .name      = "scale",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct scale_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
