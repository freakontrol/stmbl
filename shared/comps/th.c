#include "th_comp.h"
#include "hal.h"

/**
* ## Brief
* `th` is a window comparator with an input low pass: `out` is 1 while the filtered input is between `min` and `max`. It runs on the F4 board. In `conf/move.txt` it detects a hard stop for homing: `th0.in = pid0.torque_cmd`, `min = -1`, `max = 1`, `in_lpf = 0.25`, `home0.home_in = th0.out_not`.
*
* ## Component Explanation
* 1. **Filter** (in `rt`): `in_lp += (in - in_lp) * in_lpf` every rt cycle. `in_lpf` is a factor per cycle (0..1), not a frequency.
*
* 2. **Window**: `out = 1`, `out_not = 0` when `min < in_lp < max` (strict), otherwise `out = 0`, `out_not = 1`.
*/

HAL_COMP(th);

HAL_PIN(in);       // *input*, Input
HAL_PIN(in_lpf);   // *parameter*, Low pass factor per rt cycle, 1 = off (default 1)
HAL_PIN(in_lp);    // *output*, Filtered input
HAL_PIN(min);      // *parameter*, Lower window limit
HAL_PIN(max);      // *parameter*, Upper window limit
HAL_PIN(out);      // *output*, 1 while min < in_lp < max
HAL_PIN(out_not);  // *output*, Inverted out

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct th_pin_ctx_t *pins = (struct th_pin_ctx_t *)pin_ptr;
  PIN(in_lpf)               = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct th_pin_ctx_t *pins = (struct th_pin_ctx_t *)pin_ptr;

  PIN(in_lp) += (PIN(in) - PIN(in_lp)) * PIN(in_lpf);

  if(PIN(in_lp) > PIN(min) && PIN(in_lp) < PIN(max)) {
    PIN(out)     = 1;
    PIN(out_not) = 0;
  } else {
    PIN(out)     = 0;
    PIN(out_not) = 1;
  }
}

const hal_comp_t th_comp_struct = {
    .name      = "th",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct th_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
