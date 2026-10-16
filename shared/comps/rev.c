#include "rev_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `rev` optionally reverses the direction of a position and its derivative. F4 component, normally used on the command path: e.g. `rev0.in = vel_int0.pos_out`, `rev0.rev = conf0.cmd_rev`, `reslimit0.pos_in = rev0.out`, `pid0.vel_ext_cmd = rev0.out_d`, and in the sserial templates `rev0.in_dd = vel_int0.acc_out`, `pid0.acc_ext_cmd = rev0.out_dd` (acceleration feedforward from the host frames).
*
* ## Component Explanation
* 1. **Reversal** (`rt`):
* - If `rev` > 0: `out = minus(0, in)` (negated and wrapped to +-pi), `out_d = -in_d` and `out_dd = -in_dd`.
* - Otherwise the inputs are passed through unchanged.
*/

HAL_COMP(rev);

HAL_PIN(in);      // *input*, Position (rad)
HAL_PIN(out);     // *output*, Position, negated if rev > 0 (rad, +-pi)
HAL_PIN(in_d);    // *input*, Derivative of the position (rad/s)
HAL_PIN(out_d);   // *output*, Derivative, negated if rev > 0 (rad/s)
HAL_PIN(in_dd);   // *input*, Second derivative of the position (rad/s^2)
HAL_PIN(out_dd);  // *output*, Second derivative, negated if rev > 0 (rad/s^2)
HAL_PIN(rev);     // *parameter*, Reverse direction if > 0

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct rev_ctx_t * ctx = (struct rev_ctx_t *)ctx_ptr;
  struct rev_pin_ctx_t *pins = (struct rev_pin_ctx_t *)pin_ptr;

  if(PIN(rev) > 0.0) {
    PIN(out)   = minus(0, PIN(in));
    PIN(out_d)  = -PIN(in_d);
    PIN(out_dd) = -PIN(in_dd);
  } else {
    PIN(out)   = PIN(in);
    PIN(out_d)  = PIN(in_d);
    PIN(out_dd) = PIN(in_dd);
  }
}

hal_comp_t rev_comp_struct = {
    .name      = "rev",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct rev_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
