#include "home_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `home` performs a homing sequence with a home switch by adding a moving offset to a position, and releases the axis with `en_out` when homing is done. F4 component, used in `conf/move.txt`: `home0.pos_in = fmove0.mpos`, `rev0.in = home0.pos_out`, `home0.home_in = th0.out_not`, `fmove0.en = home0.en_out`.
*
* ## Component Explanation
* All work is done in `rt`. The homing velocity is limited to `|home_vel|` and changes by at most `home_acc`.
*
* 1. **States** (`state` pin):
* - 0, not homed: when `en_in` > 0, reset `offset`, `home_offset` and `vel`, go to 1.
* - 1, search switch: move with `home_vel` until the switch is active (`home_in` > 0, or <= 0 if `home_polarity` <= 0).
* - 2, leave switch: move with `-home_vel` until the switch is inactive again, then `home_offset = offset + home_pos`.
* - 3, go to home position: move `offset` to `home_offset` with a `sqrt(2 * home_acc * distance)` profile; done when closer than 0.01 rad and `|vel|` < 1% of `|home_vel|`.
* - 4, homed: `en_out` = 1, `offset = home_offset`, `vel` = 0.
*
* 2. **Output**:
* - `pos_out = mod(pos_in + mod(offset))`.
*
* 3. **Disable**:
* - With `en_in` <= 0, `en_out` is 0. If `re_home` > 0 the state is reset to 0.
*
* {{% hint warning %}}
* When entering state 4 the velocity is set to 0 without the acceleration limit.
* {{% /hint %}}
*/

HAL_COMP(home);

HAL_PIN(home_vel);  // *parameter*, Homing velocity (rad/s), default 2pi
HAL_PIN(home_acc);  // *parameter*, Homing acceleration (rad/s^2), default 2pi/0.1

HAL_PIN(pos_in);   // *input*, Position (rad)
HAL_PIN(pos_out);  // *output*, Position plus homing offset (rad, +-pi)
HAL_PIN(vel);      // *output*, Homing velocity (rad/s)

HAL_PIN(home_in);        // *input*, Home switch
HAL_PIN(home_polarity);  // *parameter*, > 0: switch active high, <= 0: active low, default 1

HAL_PIN(offset);       // *output*, Current offset added to pos_in (rad)
HAL_PIN(home_offset);  // *output*, Offset at the home position (rad)
HAL_PIN(home_pos);     // *parameter*, Home position relative to the switch edge (rad)

HAL_PIN(state);  // *output*, 0 not homed, 1 search, 2 leave switch, 3 go home, 4 homed

HAL_PIN(en_in);   // *input*, Enable, starts homing
HAL_PIN(en_out);  // *output*, Enable after homing

HAL_PIN(re_home);  // *parameter*, Home again on every enable if > 0, default 1

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct home_pin_ctx_t *pins = (struct home_pin_ctx_t *)pin_ptr;
  PIN(state)                  = 0;
  PIN(re_home)                = 1;
  PIN(home_polarity)          = 1.0;
  PIN(home_vel)               = 2.0 * M_PI;
  PIN(home_acc)               = 2.0 * M_PI / 0.1;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  //struct home_ctx_t *ctx      = (struct home_ctx_t *)ctx_ptr;
  struct home_pin_ctx_t *pins = (struct home_pin_ctx_t *)pin_ptr;

  float vel = 0.0;

  if(PIN(en_in) <= 0.0) {
    if(PIN(re_home) > 0.0) {
      PIN(state) = 0;
    }
    PIN(en_out) = 0;
  }

  switch((int)PIN(state)) {
    case 0:  // not homed
      if(PIN(en_in) > 0.0) {
        PIN(state)       = 1;
        PIN(en_out)      = 0;
        PIN(offset)      = 0.0;
        PIN(home_offset) = 0.0;
        PIN(vel)         = 0.0;
      }
      break;

    case 1:  // search home rising
      if(PIN(home_polarity) > 0.0 && PIN(home_in) > 0.0) {
        PIN(state) = 2;
      } else if(PIN(home_polarity) <= 0.0 && PIN(home_in) <= 0.0) {
        PIN(state) = 2;
      } else {
        vel = PIN(home_vel);
      }
      break;

    case 2:  // search home falling
      if(PIN(home_polarity) <= 0.0 && PIN(home_in) > 0.0) {
        PIN(state)       = 3;
        PIN(home_offset) = PIN(offset) + PIN(home_pos);
      } else if(PIN(home_polarity) > 0.0 && PIN(home_in) <= 0.0) {
        PIN(state)       = 3;
        PIN(home_offset) = PIN(offset) + PIN(home_pos);
      } else {
        vel = -PIN(home_vel);
      }
      break;

    case 3:  // go to home pos
      vel = SIGN(PIN(home_offset) - PIN(offset)) * sqrtf(ABS(PIN(home_offset) - PIN(offset)) * 2.0 * PIN(home_acc));

      if(ABS(PIN(offset) - PIN(home_offset)) < 0.01 && ABS(PIN(vel)) < ABS(PIN(home_vel)) * 0.01) {
        PIN(state) = 4;
      }
      break;

    case 4:  // homed
      PIN(en_out) = 1;
      PIN(offset) = PIN(home_offset);
      PIN(vel)    = 0.0;
      break;
  }

  vel      = LIMIT(vel, ABS(PIN(home_vel)));
  PIN(vel) = CLAMP(vel, PIN(vel) - PIN(home_acc) * period, PIN(vel) + PIN(home_acc) * period);

  PIN(offset) += PIN(vel) * period;

  PIN(pos_out) = mod(PIN(pos_in) + mod(PIN(offset)));
}

hal_comp_t home_comp_struct = {
    .name      = "home",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct home_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
