#include "move_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `move` turns two direction bits into a signed value: `out = scale * (fwd - rev)`. F4 component, used in `conf/dual_dc_sserial.txt` to drive a DC motor from smart serial outputs (`move0.fwd = sserial0.out0`, `move0.rev = sserial0.out1`, `hv0.d_cmd = move0.out`).
*
* ## Component Explanation
* 1. **Output** (`rt`):
* - With both bits set the output is 0.
*/

HAL_COMP(move);

HAL_PIN(fwd);    // *input*, Forward, typically 0 or 1
HAL_PIN(rev);    // *input*, Reverse, typically 0 or 1
HAL_PIN(scale);  // *input*, Output magnitude
HAL_PIN(out);    // *output*, scale * (fwd - rev)

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct move_pin_ctx_t *pins = (struct move_pin_ctx_t *)pin_ptr;
  PIN(out)                    = PIN(scale) * PIN(fwd) + PIN(scale) * PIN(rev) * -1;
}

hal_comp_t move_comp_struct = {
    .name      = "move",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct move_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
