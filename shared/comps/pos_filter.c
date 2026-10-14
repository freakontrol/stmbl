#include "pos_filter_comp.h"
#include "hal.h"
#include "angle.h"
#include "defines.h"

/**
* ## Brief
* `pos_filter` smooths a position command with a critically damped second order tracking filter and outputs filtered position, velocity and acceleration. F4 component, see `conf/template/pos_filter.txt`: `pos_in = rev0.out`, `vel_in = rev0.out_d`, `reslimit0.pos_in = pos_out`, `pid0.vel_ext_cmd = vel_out`, `pid0.acc_ext_cmd = acc_out`.
*
* ## Component Explanation
* All work is done in `rt`. `bandwidth` must be set (the template uses 1000).
*
* 1. **Filter**:
* ```c
* ki = 2 * MIN(bandwidth, 1 / period / 2);
* kp = ki * ki / 4;
* acc_out  = kp * minus(pos_in, pos_out) + ki * (vel_in - vel_out);
* vel_out += acc_out * period;
* pos_out  = mod(pos_out + vel_out * period);
* ```
* - Both poles are at `bandwidth` (rad/s). `bandwidth` is limited to `1 / (2 * period)`.
*
* 2. **Enable**:
* - With `en` <= 0 `acc_out` is 0.
*/

HAL_COMP(pos_filter);

HAL_PIN(pos_in);  // *input*, Position (rad)
HAL_PIN(vel_in);  // *input*, Velocity (rad/s)

HAL_PIN(pos_out);  // *output*, Filtered position (rad, +-pi)
HAL_PIN(vel_out);  // *output*, Filtered velocity (rad/s)
HAL_PIN(acc_out);  // *output*, Filtered acceleration (rad/s^2)

HAL_PIN(bandwidth);  // *parameter*, Filter bandwidth (rad/s), no default

HAL_PIN(en);  // *input*, Enable filter, pass through if <= 0

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct pos_filter_ctx_t *ctx      = (struct pos_filter_ctx_t *)ctx_ptr;
  struct pos_filter_pin_ctx_t *pins = (struct pos_filter_pin_ctx_t *)pin_ptr;

  float ki = 2.0 * MIN(PIN(bandwidth), 1.0 / period / 2.0);
  float kp = 0.25 * ki * ki;

  float pos_error = minus(PIN(pos_in), PIN(pos_out));
  float vel_error = PIN(vel_in) - PIN(vel_out);


  if(PIN(en) > 0.0) {
    PIN(acc_out) = kp * pos_error + ki * vel_error;
    PIN(vel_out) += period * PIN(acc_out);
    PIN(pos_out) = mod(PIN(pos_out) + period * PIN(vel_out));
  } else {
    PIN(acc_out) = 0;
    PIN(vel_out) = PIN(vel_in);
    PIN(pos_out) = PIN(pos_in);
  }
}

const hal_comp_t pos_filter_comp_struct = {
    .name      = "pos_filter",
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
    .pin_count = sizeof(struct pos_filter_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};