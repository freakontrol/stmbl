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
* - `dob0.vel = vel1.vel`, the velocity observer pid0 uses
* - `vel1.torque = dob0.out` (and `vel2.torque`), so the velocity observers see the compensated torque
* - `dob0.j` set to the total inertia `conf0.j + conf0.j_sys`, as `pid0` uses it (`j_mot + j_sys`)
* - `dob0.en` has to be linked or set: unlinked it is 0 and `dob` only passes `in` through
*
* ## Component Explanation
* All work is done in `rt`.
*
* 1. **Model**:
* - Rigid body: `j * dw/dt = u + d`, so `d = j * dw/dt - u`.
* - `w` is the observer velocity `vel` (vel1, which already filters the encoder at its loop bandwidth); `(w - last w) / period` is the acceleration caused by the torque `u` of the step before, where `u` is the previous `out` (the compensated command). An earlier version differenced the raw position twice: with an 8000 count encoder that left about 0.4 Nm rms on `d_est` at `bw` 50 (upstream review), the observer velocity about a fifth of it.
* - `d` is low passed at `bw` into `d_est` (`bw` 0 = no filter). Default 20 Hz (upstream review): on the Y plant 50 Hz cuts the load step error by about a third (4.5 against 6.7 mrad for a 2 Nm step) at 2.5 times the estimate noise; set it per axis.
*
* 2. **Compensation**:
* - `out = in - k * d_est`, with `k * d_est` clamped to `+-max` when `max > 0`. The template links `max` to the torque limit `conf0.max_force`, so the compensation can never exceed what the drive may apply; a smaller value bounds the load share dob takes.
* - `k` 0 only estimates (`d_est` is still valid, `out = in`), `k` 1 is full compensation.
*
* 3. **Enable and start**:
* - While `en <= 0` or `j <= 0` (the default): `out = in`, `d_est = 0`, and the last position, velocity and torque keep following, so enabling (or setting `j`) starts clean.
* - After power up or enabling it waits one rt step for a previous velocity sample before estimating.
*
* 4. **Standstill hold** (`hold`, linked to `pid0.hold` with `pid0.hold_en` 2):
* - With stiction the estimate freezes at whatever the last move left (on a vertical axis gravity plus or minus the friction, so the standing current depends on the direction of the last move).
* - While `hold` is 1 (pid0 at rest, position error inside its hold window) the estimate stops and the compensation slews at `hold_rate` so that `out` moves to `hold_o` (the moving offset `conf0.o`, about 0 on a horizontal axis). `d_est` follows the compensation, so the estimate goes on from there when the hold ends. If the axis creeps out of the window the hold ends at once.
* - `max` must leave room for the slew: on Z (o 4.59, f 2.7) `max` 4 stopped it short, 8 did not.
*
* 5. **Results** (commit message): in the simulation on the Y axis the load step error was 2.5 to 3 times lower at `bw` 50 to 100 Hz, 200 Hz oscillated. On the Y axis bench, with the friction feedforward off, `bw` 50 halved the error.
*
* {{% hint warning %}}
* `j` must be set, or dob stays off. With no inertia the model would read the drive's own torque command as disturbance (`d` = `-u`) and `k` would cancel it.
* {{% /hint %}}
*/

HAL_COMP(dob);

HAL_PIN(in);     // *input*, Torque command (Nm), pid0.torque_cmd
HAL_PIN(out);    // *output*, Compensated torque command (Nm)
HAL_PIN(vel);    // *input*, Observer velocity (rad/s), vel1.vel
HAL_PIN(j);      // *parameter*, Total inertia (kgm^2, default 0 = dob off)
HAL_PIN(bw);     // *parameter*, Low pass of the estimate (Hz, default 20, 0 = none)
HAL_PIN(k);      // *parameter*, Compensation gain, 0 = estimate only, 1 = full (default 0)
HAL_PIN(max);    // *parameter*, Clamp on the compensation (Nm, default 0 = none), conf0.max_force
HAL_PIN(d_est);  // *output*, Estimated disturbance torque (Nm), 0 while disabled
HAL_PIN(en);     // *input*, Enable, 0 = pass-through
HAL_PIN(hold);       // *input*, Standstill hold, pid0.hold (with pid0.hold_en 2)
HAL_PIN(hold_o);     // *parameter*, Torque the hold moves out to (Nm), conf0.o
HAL_PIN(hold_rate);  // *parameter*, Slew rate of out to hold_o (Nm/s), default 20

struct dob_ctx_t {
  float last_w;
  float last_u;
  uint32_t have;
  float c;  // compensation applied, k * d_est clamped, or slewed while holding
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct dob_pin_ctx_t *pins = (struct dob_pin_ctx_t *)pin_ptr;
  PIN(bw)        = 20.0;
  PIN(k)         = 0.0;
  PIN(max)       = 0.0;
  PIN(hold_rate) = 20.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct dob_ctx_t *ctx      = (struct dob_ctx_t *)ctx_ptr;
  struct dob_pin_ctx_t *pins = (struct dob_pin_ctx_t *)pin_ptr;

  float w = PIN(vel);

  if(PIN(en) <= 0.0 || PIN(j) <= 0.0 || !ctx->have) {
    PIN(d_est)  = 0.0;
    ctx->have   = PIN(en) > 0.0 && PIN(j) > 0.0;
    ctx->last_w = w;
    ctx->last_u = PIN(in);
    PIN(out)    = PIN(in);
    ctx->c      = 0.0;
    return;
  }

  // the change of the observer velocity over the step is the acceleration
  // caused by the torque of the step before
  float d     = PIN(j) * (w - ctx->last_w) / period - ctx->last_u;
  ctx->last_w = w;

  float c;
  if(PIN(hold) > 0.0 && PIN(k) > 0.0) {
    // standstill hold: the estimate stops, the compensation slews so that
    // out moves to hold_o; d_est follows it so leaving the hold is smooth
    float err  = PIN(in) - ctx->c - PIN(hold_o);
    c          = ctx->c + CLAMP(err, -PIN(hold_rate) * period, PIN(hold_rate) * period);
    PIN(d_est) = c / PIN(k);
  } else {
    PIN(d_est) += (d - PIN(d_est)) * LP_HZ(PIN(bw));
    c = PIN(k) * PIN(d_est);
  }
  if(PIN(max) > 0.0) {
    c = CLAMP(c, -PIN(max), PIN(max));
  }
  ctx->c      = c;
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
