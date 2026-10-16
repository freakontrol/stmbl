#include "debounce_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `debounce` filters a logic signal with an integrating timer, so short pulses are ignored. F4 component, used in `conf/wobl.txt` (`fault0.en = debounce0.out`).
*
* ## Component Explanation
* 1. **Timer** (`rt`):
* - `timer` counts up by the period while `in` > 0 and down otherwise.
* - When `timer` exceeds `debounce_time`, `out` becomes 1 (and `timer` is clamped to `debounce_time`). When `timer` reaches 0, `out` becomes 0.
* - The delay is the same for both edges; short glitches only move the timer and do not change `out`.
*/

HAL_COMP(debounce);

HAL_PIN(in);             // *input*, Logic input, true if > 0
HAL_PIN(out);            // *output*, Debounced output
HAL_PIN(debounce_time);  // *parameter*, Debounce time (s), default 0
HAL_PIN(timer);          // *output*, Debounce timer (internal state, s)

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct debounce_ctx_t *ctx      = (struct debounce_ctx_t *)ctx_ptr;
  struct debounce_pin_ctx_t *pins = (struct debounce_pin_ctx_t *)pin_ptr;

  if(PIN(in) > 0.0) {
    PIN(timer) += period;
  } else {
    PIN(timer) -= period;
  }

  if(PIN(timer) > PIN(debounce_time)) {
    PIN(timer) = PIN(debounce_time);
    PIN(out)   = 1.0;
  } else if(PIN(timer) <= 0.0) {
    PIN(timer) = 0.0;
    PIN(out)   = 0.0;
  }
}

hal_comp_t debounce_comp_struct = {
    .name      = "debounce",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct debounce_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
