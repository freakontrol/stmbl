#include "veltopos_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `veltopos` integrates a velocity command into an electrical angle, with the velocity limited relative to the measured velocity. Intended for open loop / sensorless style commutation. F4 component; it is not used by any config in `conf/`.
*
* ## Component Explanation
* All work is done in `rt`. There is no `nrt_init`, all pins start at 0.
*
* 1. **Acceleration and velocity limit**:
* - The velocity is clamped to `vel_fb +- max_acc * period`, i.e. it may only be ahead of or behind the measured velocity by one period of `max_acc`. With `max_acc` = 0 the output follows `vel_fb`.
* - If `max_vel` > 0, the result is then limited to +-`max_vel`.
*
* 2. **Integration**:
* - `pos += vel * polecount * period`, wrapped with `mod()`.
*
* {{% hint warning %}}
* Experimental.
* {{% /hint %}}
*/

HAL_COMP(veltopos);

HAL_PIN(vel);        // *input*, Velocity command (rad/s, mechanical)
HAL_PIN(vel_fb);     // *input*, Measured velocity (rad/s, mechanical)
HAL_PIN(max_acc);    // *parameter*, Maximum acceleration relative to vel_fb (rad/s^2)
HAL_PIN(max_vel);    // *parameter*, Maximum velocity (rad/s), 0 = no limit
HAL_PIN(polecount);  // *parameter*, Motor pole pairs
HAL_PIN(pos);        // *output*, Integrated electrical angle (rad, +-pi)


struct veltopos_ctx_t {
  float vel;
};

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct veltopos_ctx_t *ctx      = (struct veltopos_ctx_t *)ctx_ptr;
  struct veltopos_pin_ctx_t *pins = (struct veltopos_pin_ctx_t *)pin_ptr;

  //ctx->vel = CLAMP(PIN(vel), ctx->vel - PIN(max_acc) * period, ctx->vel + PIN(max_acc) * period);
  ctx->vel = CLAMP(PIN(vel), PIN(vel_fb) - PIN(max_acc) * period, PIN(vel_fb) + PIN(max_acc) * period);
  if(PIN(max_vel) > 0.0) {
    ctx->vel = LIMIT(ctx->vel, PIN(max_vel));
  }
  PIN(pos) += ctx->vel * PIN(polecount) * period;
  PIN(pos) = mod(PIN(pos));
}

hal_comp_t veltopos_comp_struct = {
    .name      = "veltopos",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct veltopos_ctx_t),
    .pin_count = sizeof(struct veltopos_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
