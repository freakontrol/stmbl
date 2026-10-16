#include "dob_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

// Disturbance (load torque) observer on the torque command.
// The rigid-body model j * dw/dt = u + d gives the torque the model cannot
// explain: d = j * dw/dt - u (load, friction, cutting force, cogging).
// It is low passed at bw Hz and k * d is subtracted from the torque command,
// so the drive cancels the disturbance before the velocity integrator has to.
//   dob0.in = pid0.torque_cmd
//   pmsm_ttc0.torque = dob0.out
//   dob0.pos = fb_switch0.vel_fb
//   dob0.j = conf0.j_sys + conf0.j (total inertia, as pid0 uses it)
//   vel1.torque = dob0.out (and vel2.torque)
// k 0 = estimate only (d_est still valid), 1 = full compensation.
// max clamps the compensation (Nm, 0 = none).
// j <= 0 (the default) keeps it off: with no inertia the model reads the
// drive's own torque command as disturbance and k would cancel it.

HAL_COMP(dob);

HAL_PIN(in);
HAL_PIN(out);
HAL_PIN(pos);
HAL_PIN(j);
HAL_PIN(bw);  // (Hz)
HAL_PIN(k);
HAL_PIN(max);
HAL_PIN(d_est);  // (Nm)
HAL_PIN(en);

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
