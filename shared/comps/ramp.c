#include "ramp_comp.h"
#include "commands.h"
#include "hal.h"
#include "defines.h"

/**
* ## Brief
* `ramp` is a velocity ramp generator with separate acceleration and deceleration limits and an automatic enable output, mainly for spindles in V/f mode. F4 component, used e.g. in `conf/haas_spindle*.txt`: `ramp0.vel_ext_cmd = avg0.out`, `uf0.vel_cmd = ramp0.vel_cmd`, `ramp0.max_acc = conf0.max_acc`.
*
* ## Component Explanation
* All work is done in `rt`. `max_vel`, `max_acc` and `max_dec` default to 0 and must be set.
*
* 1. **Command**:
* - `vel_ext_cmd` is limited to +-`max_vel`.
*
* 2. **Ramp**:
* - `vel_cmd` moves towards the target by at most `max_acc * scale * period` when the absolute velocity increases and `max_dec * scale * period` when it decreases.
* - The ramp only starts once `en_timer` >= 0.9 * `en_delay`, giving the drive time to enable.
*
* 3. **Automatic enable**:
* - `en_timer` counts up while `vel_cmd` or the target is above 0.01, and down otherwise, clamped to 0..`en_delay`. `en_out` = 1 while `en_timer` > 0, so the drive stays enabled for `en_delay` after reaching standstill.
*
* 4. **Status**:
* - `at_speed` = 1 when `|target - vel_cmd|` < `max_vel * at_speed_th` and `|vel_cmd|` > 0.01, in both directions.
* - `zero_speed` = 1 when both `vel_cmd` and the target are below 0.01.
*
* {{% hint warning %}}
* When reversing directly, the phase that decelerates through zero uses `max_acc` instead of `max_dec` once the absolute target is larger than the absolute current velocity.
* {{% /hint %}}
*/

HAL_COMP(ramp);

// input
HAL_PIN(vel_ext_cmd);  // *input*, Velocity target (rad/s)

HAL_PIN(en);  // *input*, Enable, target is 0 if <= 0

HAL_PIN(scale);  // *input*, Scale for max_acc and max_dec, default 1

HAL_PIN(max_vel);      // *parameter*, Maximum velocity (rad/s)
HAL_PIN(max_acc);      // *parameter*, Maximum acceleration (rad/s^2)
HAL_PIN(max_dec);      // *parameter*, Maximum deceleration (rad/s^2)
HAL_PIN(at_speed_th);  // *parameter*, at_speed threshold as fraction of max_vel, default 0.01

// output
HAL_PIN(vel_cmd);  // *output*, Ramped velocity command (rad/s)

HAL_PIN(at_speed);    // *output*, 1 = vel_cmd reached the target
HAL_PIN(zero_speed);  // *output*, 1 = vel_cmd and target are 0
HAL_PIN(en_out);      // *output*, Enable for the drive
HAL_PIN(en_timer);    // *output*, Enable timer (internal state, s)
HAL_PIN(en_delay);    // *parameter*, Enable delay before ramping and after stopping (s), default 0.25

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct ramp_ctx_t *ctx      = (struct ramp_ctx_t *)ctx_ptr;
  struct ramp_pin_ctx_t *pins = (struct ramp_pin_ctx_t *)pin_ptr;

  PIN(at_speed_th) = 0.01;
  PIN(en_delay)    = 0.25;
  PIN(scale)       = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct ramp_ctx_t *ctx      = (struct ramp_ctx_t *)ctx_ptr;
  struct ramp_pin_ctx_t *pins = (struct ramp_pin_ctx_t *)pin_ptr;

  float vel_ext_cmd = LIMIT(PIN(vel_ext_cmd), PIN(max_vel));

  if(PIN(en) <= 0.0) {
    vel_ext_cmd = 0.0;
  }

  float vel_error = vel_ext_cmd - PIN(vel_cmd);
  float abs_vel_error = ABS(vel_ext_cmd) - ABS(PIN(vel_cmd));
  float max_acc   = PIN(max_acc) * PIN(scale);
  float max_dec   = PIN(max_dec) * PIN(scale);

  if(PIN(en_timer) >= PIN(en_delay) * 0.9) {
    PIN(vel_cmd) += LIMIT(vel_error, (abs_vel_error > 0.f ? max_acc : max_dec) * period);
  }

  if(ABS(vel_ext_cmd - PIN(vel_cmd)) < PIN(max_vel) * PIN(at_speed_th) && ABS(PIN(vel_cmd)) > 0.01) {
    PIN(at_speed) = 1;
  } else {
    PIN(at_speed) = 0;
  }

  if((ABS(PIN(vel_cmd)) < 0.01) && (ABS(vel_ext_cmd) < 0.01)) {
    PIN(zero_speed) = 1;
  } else {
    PIN(zero_speed) = 0;
  }


  if((ABS(PIN(vel_cmd)) > 0.01) || (ABS(vel_ext_cmd) > 0.01)) {
    PIN(en_timer) = CLAMP(PIN(en_timer) + period, 0, PIN(en_delay));
  } else {
    PIN(en_timer) = CLAMP(PIN(en_timer) - period, 0, PIN(en_delay));
  }
  if(PIN(en_timer) > 0.0) {
    PIN(en_out) = 1;
  } else {
    PIN(en_out) = 0;
  }
}

hal_comp_t ramp_comp_struct = {
    .name      = "ramp",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct ramp_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
