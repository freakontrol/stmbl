#include "mpid_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `mpid` is an alternative cascaded position / velocity controller for the F4 board that tracks the position error incrementally, so the error is not limited to +-pi. It outputs a torque command (Nm). It is loaded by `conf/template/mpid.txt` with `mpid0.pos_ext_cmd = reslimit0.pos_out`, `mpid0.vel_ext_cmd = veltime0.vel_lp`, `mpid0.pos_fb = fb_switch0.pos_fb`, `mpid0.vel_fb = veltime1.vel_lp`, `mpid0.en = fault0.en_pid`, and gains / limits from `conf0` (`pos_p`, `vel_p`, `vel_i`, `j`, `max_vel`, `max_acc`, `max_force`).
*
* ## Component Explanation
* All work is done in `rt`. There is no `nrt_init`, all pins start at 0.
*
* 1. **Incremental position error**:
* - Every cycle the change of the command (`minus(pos_ext_cmd, old_pos_ext_cmd)`, multiplied by `scale`) is added to `pos_error` and the change of the feedback is subtracted. The old values are stored in `old_pos_ext_cmd` / `old_pos_fb`, also while disabled.
* - Because only increments are used, `pos_error` can grow beyond one turn.
*
* 2. **Position loop**:
* - `vel_cmd = LIMIT(pos_error * pos_p, max_vel)`, set to 0 while `abs(pos_error) < min_pos_error` (dead band), then `vel_cmd += vel_ext_cmd`.
*
* 3. **Velocity loop**:
* - `vel_error = vel_cmd - vel_fb`, `acc_cmd = LIMIT(vel_error * vel_p, max_acc)`, `torque_cmd = LIMIT(acc_cmd * j, max_torque)`.
* - Integrator: `torque_sum += vel_error * vel_i * period`, clamped to the headroom `+-max_torque - torque_cmd`.
* - `torque_cmd += torque_ext_cmd + torque_sum`. The final value is not clamped again, so `torque_ext_cmd` can push it beyond `max_torque`.
*
* 4. **Disable**: with `en <= 0` all outputs, the position error and the integrator are 0.
*
* {{% hint warning %}}
* `scale` has no default and is not set by `conf/template/mpid.txt`, so it is 0 and command motion is ignored (only feedback motion enters `pos_error`). Set `mpid0.scale = 1` in the config. There is no saturation output, so `fault0.sat` is not driven in this template.
* {{% /hint %}}
*/

HAL_COMP(mpid);

HAL_PIN(pos_ext_cmd);      // *input*, Position command (rad)
HAL_PIN(pos_fb);           // *input*, Position feedback (rad)
HAL_PIN(old_pos_ext_cmd);  // *output*, Position command of the previous cycle (rad)
HAL_PIN(old_pos_fb);       // *output*, Position feedback of the previous cycle (rad)
HAL_PIN(pos_error);        // *output*, Accumulated position error, not wrapped (rad)

HAL_PIN(vel_ext_cmd);  // *input*, Velocity feedforward command (rad/s)
HAL_PIN(vel_fb);       // *input*, Velocity feedback (rad/s)
HAL_PIN(vel_error);    // *output*, Velocity error (rad/s)
HAL_PIN(vel_cmd);      // *output*, Velocity command (rad/s)

HAL_PIN(acc_cmd);  // *output*, Acceleration command (rad/s^2)

HAL_PIN(torque_ext_cmd);  // *input*, Additional torque command (Nm)
HAL_PIN(torque_sum);      // *output*, Velocity loop integrator (Nm)
HAL_PIN(torque_cmd);      // *output*, Torque command (Nm)

HAL_PIN(min_pos_error);  // *parameter*, Position error dead band (rad)
HAL_PIN(max_vel);        // *parameter*, Velocity limit of the position loop (rad/s)
HAL_PIN(max_acc);        // *parameter*, Acceleration limit (rad/s^2)
HAL_PIN(max_torque);     // *parameter*, Torque limit (Nm)

HAL_PIN(pos_p);  // *parameter*, Position P gain (1/s)
HAL_PIN(vel_i);  // *parameter*, Velocity I gain (Nm/rad)
HAL_PIN(vel_p);  // *parameter*, Velocity P gain (1/s)
HAL_PIN(j);      // *parameter*, Inertia (kgm^2)
HAL_PIN(scale);  // *parameter*, Command scale / gear ratio, no default (0), set to 1

HAL_PIN(en);  // *input*, Enable

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct mpid_ctx_t *ctx      = (struct mpid_ctx_t *)ctx_ptr;
  struct mpid_pin_ctx_t *pins = (struct mpid_pin_ctx_t *)pin_ptr;

  if(PIN(en) > 0.0) {
    PIN(pos_error) += minus(PIN(pos_ext_cmd), PIN(old_pos_ext_cmd)) * PIN(scale);
    PIN(pos_error) -= minus(PIN(pos_fb), PIN(old_pos_fb));

    PIN(vel_cmd) = LIMIT(PIN(pos_error) * PIN(pos_p), PIN(max_vel));

    if(ABS(PIN(pos_error)) < PIN(min_pos_error)) {
      PIN(vel_cmd) = 0.0;
    }

    PIN(vel_cmd) += PIN(vel_ext_cmd);

    PIN(vel_error) = PIN(vel_cmd) - PIN(vel_fb);

    PIN(acc_cmd) = LIMIT(PIN(vel_error) * PIN(vel_p), PIN(max_acc));

    PIN(torque_cmd) = LIMIT(PIN(acc_cmd) * PIN(j), PIN(max_torque));

    PIN(torque_sum) += PIN(vel_error) * PIN(vel_i) * period;
    PIN(torque_sum) = CLAMP(PIN(torque_sum), -PIN(max_torque) - PIN(torque_cmd), PIN(max_torque) - PIN(torque_cmd));

    PIN(torque_cmd) += PIN(torque_ext_cmd);
    PIN(torque_cmd) += PIN(torque_sum);
  } else {
    PIN(pos_error)  = 0.0;
    PIN(vel_cmd)    = 0.0;
    PIN(vel_error)  = 0.0;
    PIN(acc_cmd)    = 0.0;
    PIN(torque_sum) = 0.0;
    PIN(torque_cmd) = 0.0;
  }

  PIN(old_pos_fb)      = PIN(pos_fb);
  PIN(old_pos_ext_cmd) = PIN(pos_ext_cmd);
}

hal_comp_t mpid_comp_struct = {
    .name      = "mpid",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct mpid_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};