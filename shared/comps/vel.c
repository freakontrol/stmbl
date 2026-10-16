#include "vel_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `vel` is a tracking observer (a second order PLL with optional torque feedforward) that estimates velocity and a filtered position from a position signal. It is an F4 component and is loaded several times in the templates: e.g. `vel1.pos_in = fb_switch0.vel_fb` with `pid0.vel_fb = vel1.vel` gives the velocity feedback, `vel2.pos_in = fb_switch0.com_fb` gives a filtered commutation angle for `hv0.pos`, and `vel0.pos_in = rev0.out` differentiates the position command.
*
* ## Component Explanation
* All work is done in `rt`.
*
* 1. **Enable**:
* - While `en` is 0 the internal position estimate is set to `pos_in` and the internal velocity to 0 every period, so the observer starts without a jump when enabled.
*
* 2. **Torque feedforward**:
* - The expected acceleration from the applied torque is `acc = g * torque / j` (`j` is clamped to at least 1e-7).
* - `acc` is low pass filtered at `lp` Hz and only the high frequency part `acc - lp(acc)` is used as feedforward, so a wrong `j` does not cause a steady state error. Set `torque` to 0 (or `g` = 0) to disable it.
*
* 3. **PLL loop**:
* - `pos_error = minus(pos_in, pos_est)` (wrapped to +-pi).
* - The acceleration `acc` = torque feedforward + `w^2 * pos_error` is integrated into the velocity estimate.
* - The position estimate is integrated from the velocity estimate plus `vel_ff * h` plus a damping term `2 * d * w * pos_error`, then wrapped with `mod()`.
* ```c
* acc_ff   = (acc - acc_lp) + pos_error * w * w;
* vel_est += acc_ff * period;
* pos_est += (vel_est + vel_ff * h + 2 * d * w * pos_error) * period;
* ```
* - A `d` of 0.9 is slightly underdamped, 1 is critically damped.
*
* 4. **Outputs**:
* - `vel` = velocity estimate + `vel_ff * h` + `vel_p` x the damping term `2 * d * w * pos_error` (`vel_p` clamped to 0..1). `pos_error` = remaining tracking error after the update.
* - The velocity estimate is the loop's integrator, which trails the speed by `2 * d / w` in steady acceleration (1.8 ms at `w` 1000, `d` 0.9). With `vel_p = 1` the proportional term is added, so `vel` is the derivative of `pos_out` and has no lag, but it passes more encoder noise. Set it for a position axis, not a spindle; `conf/fanuc_a3-3000.txt` sets `vel1.vel_p = 1` (with `vel_int0.vel_ip = 1`).
*/

HAL_COMP(vel);

HAL_PIN(pos_in);     // *input*, Position to track (rad, wrapped)
HAL_PIN(pos_out);    // *output*, Filtered/estimated position (rad, +-pi)
HAL_PIN(vel);        // *output*, Estimated velocity (rad/s)
HAL_PIN(acc);        // *output*, Loop acceleration (rad/s^2)
HAL_PIN(w);          // *parameter*, Loop bandwidth (rad/s), default 1000
HAL_PIN(d);          // *parameter*, Loop damping ratio, default 0.9
HAL_PIN(g);          // *parameter*, Gain of the torque feedforward, default 1
HAL_PIN(h);          // *parameter*, Gain of the velocity feedforward, default 1
HAL_PIN(j);          // *parameter*, Inertia used for the torque feedforward (kgm^2), default 1e-5
HAL_PIN(lp);         // *parameter*, Low pass frequency of the torque feedforward (Hz), default 50
HAL_PIN(torque);     // *input*, Applied torque for the acceleration feedforward (Nm)
HAL_PIN(vel_ff);     // *input*, Velocity feedforward (rad/s)
HAL_PIN(en);         // *input*, Enable, 0 resets the observer to pos_in, default 1
HAL_PIN(pos_error);  // *output*, Tracking error pos_in - pos_out (rad)
HAL_PIN(vel_p);      // *parameter*, Share of the loop's proportional term in vel, 0..1, 0 = integrator only (default), 1 for a position axis

struct vel_ctx_t {
  float last_acc;
  float acc_sum;
  float vel_sum;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct vel_ctx_t * ctx = (struct vel_ctx_t *)ctx_ptr;
  struct vel_pin_ctx_t *pins = (struct vel_pin_ctx_t *)pin_ptr;

  PIN(w)  = 1000.0;
  PIN(d)  = 0.9;
  PIN(g)  = 1.0;
  PIN(h)  = 1.0;
  PIN(j)  = 0.00001;
  PIN(lp) = 50.0;
  PIN(en) = 1.0;
  PIN(vel_p) = 0.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct vel_ctx_t *ctx      = (struct vel_ctx_t *)ctx_ptr;
  struct vel_pin_ctx_t *pins = (struct vel_pin_ctx_t *)pin_ptr;

  if(PIN(en) == 0.0) {
    ctx->vel_sum = PIN(pos_in);
    ctx->acc_sum = 0.0;
  }
  float vel_ff = PIN(vel_ff) * PIN(h);
  ctx->vel_sum += (ctx->acc_sum + vel_ff) * period;  // ff

  float pos_error = minus(PIN(pos_in), ctx->vel_sum);
  float w         = PIN(w);
  float d         = PIN(d);
  float g         = PIN(g);
  float lp        = LP_HZ(PIN(lp));
  float j         = MAX(PIN(j), 0.0000001);
  float acc       = g * PIN(torque) / j;

  ctx->last_acc = acc * lp + (1.0 - lp) * ctx->last_acc;

  float acc_ff = acc - ctx->last_acc;

  acc_ff += pos_error * w * w;

  ctx->acc_sum += acc_ff * period;

  PIN(acc) = acc_ff;

  vel_ff = 2.0 * d * w * pos_error;
  PIN(vel) = ctx->acc_sum + PIN(vel_ff) * PIN(h) + vel_ff * CLAMP(PIN(vel_p), 0.0, 1.0);

  ctx->vel_sum += vel_ff * period;
  ctx->vel_sum = mod(ctx->vel_sum);

  PIN(pos_out)   = ctx->vel_sum;
  PIN(pos_error) = minus(PIN(pos_in), ctx->vel_sum);
}

hal_comp_t vel_comp_struct = {
    .name      = "vel",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct vel_ctx_t),
    .pin_count = sizeof(struct vel_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
