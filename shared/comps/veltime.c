#include "veltime_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `veltime` measures velocity by timing how long it takes until a position changes (period measurement instead of frequency measurement). Useful for low resolution or slowly updating positions. F4 component, runs in `frt`. Used in `conf/template/mpid.txt`: `veltime0.pos = rev0.out`, `mpid0.vel_ext_cmd = veltime0.vel_lp`.
*
* ## Component Explanation
* 1. **Measurement**:
* - `timer` is incremented every frt period. When `pos` differs from `old_pos`, `vel = minus(pos, old_pos) / timer`, `old_pos` is updated and `timer` is reset.
* - The velocity is held until the next change.
*
* 2. **Standstill**:
* - If no change happens within `max_time`, `vel` is set to 0 and `timer` is clamped to `max_time`. The lowest measurable velocity is therefore about one position step per `max_time`.
*/

HAL_COMP(veltime);

HAL_PIN(pos);      // *input*, Position (rad)
HAL_PIN(old_pos);  // *output*, Position at the last change (internal state, rad)
HAL_PIN(timer);    // *output*, Time since the last position change (internal state, s)
HAL_PIN(vel);      // *output*, Velocity (rad/s)
HAL_PIN(vel_lp);   // *output*, Low pass filtered velocity (rad/s)

HAL_PIN(max_time);  // *parameter*, Time without change after which vel is 0 (s), default 0.1
HAL_PIN(lpf)        // *parameter*, Cutoff of the vel_lp filter (Hz), default 100


static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct veltime_ctx_t *ctx      = (struct veltime_ctx_t *)ctx_ptr;
  struct veltime_pin_ctx_t *pins = (struct veltime_pin_ctx_t *)pin_ptr;

  PIN(max_time) = 0.1;

  PIN(lpf) = 100.0;
}


static void frt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct veltime_ctx_t *ctx      = (struct veltime_ctx_t *)ctx_ptr;
  struct veltime_pin_ctx_t *pins = (struct veltime_pin_ctx_t *)pin_ptr;

  PIN(timer) += period;

  if(PIN(pos) != PIN(old_pos)) {
    PIN(vel)     = minus(PIN(pos), PIN(old_pos)) / PIN(timer);
    PIN(old_pos) = PIN(pos);
    PIN(timer)   = 0.0;
  }

  if(PIN(timer) > PIN(max_time)) {
    PIN(timer) = PIN(max_time);
    PIN(vel)   = 0.0;
  }

  PIN(vel_lp) = PIN(vel) * LP_HZ(PIN(lpf)) + PIN(vel_lp) * (1.0 - LP_HZ(PIN(lpf)));
}

hal_comp_t veltime_comp_struct = {
    .name      = "veltime",
    .nrt       = 0,
    .rt        = 0,
    .frt       = frt_func,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct veltime_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};