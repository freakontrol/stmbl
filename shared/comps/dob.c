#include "dob_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `dob` is a disturbance (load torque) observer on the torque command. It estimates the torque the rigid body model cannot explain (load, friction, cutting force, cogging) and subtracts it from the torque command, so the drive cancels the disturbance before the velocity integrator has to. `conf/template/filters.txt` loads it (opt-in, `link filters` after `link pmsm`) between `biquad0` and the torque controller, passing through (`k` 0) until set; `conf/fanuc_a3-3000.txt` links it with `k` 1 and the friction feedforward off. The wiring, by hand or as in the template:
* - `dob0.in = pid0.torque_cmd` (the template: `biquad0.out`)
* - `pmsm_ttc0.torque = dob0.out`
* - `dob0.pos = fb_switch0.vel_fb`
* - `vel1.torque = dob0.out` (and `vel2.torque`), so the velocity observers see the compensated torque
* - `dob0.j` set to the total inertia `conf0.j + conf0.j_sys`, as `pid0` uses it (`j_mot + j_sys`)
* - `dob0.en` has to be linked or set: unlinked it is 0 and `dob` only passes `in` through
*
* ## Component Explanation
* All work is done in `rt`.
*
* 1. **Model**:
* - Rigid body: `j * dw/dt = u + d`, so `d = j * dw/dt - u`.
* - `w` is the raw velocity from the position difference, `minus(pos, last pos) / period`. `w` and the previous `w` are mean velocities over the last two steps, so their difference is the acceleration caused by the torque `u` of the step before; `u` is the previous `out` (the compensated command).
* - `d` is low passed at `bw` into `d_est` (`bw` 0 = no filter).
*
* 2. **Compensation**:
* - `out = in - k * d_est`, with `k * d_est` clamped to `+-max` when `max > 0`.
* - `k` 0 only estimates (`d_est` is still valid, `out = in`), `k` 1 is full compensation.
*
* 3. **Enable and start**:
* - While `en <= 0` or `j <= 0` (the default): `out = in`, `d_est = 0`, and the last position, velocity and torque keep following, so enabling (or setting `j`) starts clean.
* - After power up or enabling it waits for valid velocity samples (up to two rt steps) before estimating.
*
* 4. **Results** (commit message): in the simulation on the Y axis the load step error was 2.5 to 3 times lower at `bw` 50 to 100 Hz, 200 Hz oscillated. On the Y axis bench, with the friction feedforward off, `bw` 50 halved the error.
*
* {{% hint warning %}}
* `j` must be set, or dob stays off. With no inertia the model would read the drive's own torque command as disturbance (`d` = `-u`) and `k` would cancel it.
* {{% /hint %}}
*/

HAL_COMP(dob);

HAL_PIN(in);     // *input*, Torque command (Nm), pid0.torque_cmd
HAL_PIN(out);    // *output*, Compensated torque command (Nm)
HAL_PIN(pos);    // *input*, Motor position (rad), fb_switch0.vel_fb
HAL_PIN(j);      // *parameter*, Total inertia (kgm^2, default 0 = dob off)
HAL_PIN(bw);     // *parameter*, Low pass of the estimate (Hz, default 50, 0 = none)
HAL_PIN(k);      // *parameter*, Compensation gain, 0 = estimate only, 1 = full (default 0)
HAL_PIN(max);    // *parameter*, Clamp on the compensation (Nm, default 0 = none)
HAL_PIN(d_est);  // *output*, Estimated disturbance torque (Nm), 0 while disabled
HAL_PIN(en);     // *input*, Enable, 0 = pass-through

struct dob_ctx_t {
  float last_pos;
  float last_w;
  float last_u;
  uint32_t have;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct dob_pin_ctx_t *pins = (struct dob_pin_ctx_t *)pin_ptr;
  PIN(bw)  = 50.0;
  PIN(k)   = 0.0;
  PIN(max) = 0.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct dob_ctx_t *ctx      = (struct dob_ctx_t *)ctx_ptr;
  struct dob_pin_ctx_t *pins = (struct dob_pin_ctx_t *)pin_ptr;

  float w       = ctx->have ? minus(PIN(pos), ctx->last_pos) / period : 0.0;
  ctx->last_pos = PIN(pos);

  if(PIN(en) <= 0.0 || PIN(j) <= 0.0 || ctx->have < 2) {
    PIN(d_est) = 0.0;
    ctx->have  = MIN(ctx->have + 1, 2);
    ctx->last_w = w;
    ctx->last_u = PIN(in);
    PIN(out)    = PIN(in);
    if(PIN(en) <= 0.0 || PIN(j) <= 0.0) {
      ctx->have = ctx->have ? 1 : 0;
    }
    return;
  }

  // w and last_w are mean velocities over the last two steps: their
  // difference is the acceleration caused by the torque of the step before
  float d     = PIN(j) * (w - ctx->last_w) / period - ctx->last_u;
  PIN(d_est) += (d - PIN(d_est)) * LP_HZ(PIN(bw));
  ctx->last_w = w;

  float c = PIN(k) * PIN(d_est);
  if(PIN(max) > 0.0) {
    c = CLAMP(c, -PIN(max), PIN(max));
  }
  PIN(out)    = PIN(in) - c;
  ctx->last_u = PIN(out);
}

hal_comp_t dob_comp_struct = {
    .name      = "dob",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct dob_ctx_t),
    .pin_count = sizeof(struct dob_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
