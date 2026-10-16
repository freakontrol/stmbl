#include "biquad_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

// Second-order filter for a command signal, e.g. a notch on the torque command
// at a mechanical resonance:
//   biquad0.in = pid0.torque_cmd
//   pmsm_ttc0.torque = biquad0.out
//   biquad0.rt_prio = 8.5 (after pid0, before the torque controller)
// type 0: pass-through (default), 1: notch, 2: low pass.
// notch: gain depth at f0 (0 = full notch, 0.1 = -20 dB), width f0 / q.
// low pass: corner f0, q 0.707 = Butterworth.
// Coefficients from the bilinear transform (RBJ cookbook) at the rt period,
// recomputed when f0, q, depth or type change. f0 is held below 0.45 * rt rate.

HAL_COMP(biquad);

HAL_PIN(in);
HAL_PIN(out);
HAL_PIN(type);
HAL_PIN(f0);     // centre / corner frequency (Hz)
HAL_PIN(q);
HAL_PIN(depth);  // notch gain at f0

struct biquad_ctx_t {
  float b0, b1, b2, a1, a2;
  float z1, z2;
  float type, f0, q, depth, period;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct biquad_ctx_t *ctx      = (struct biquad_ctx_t *)ctx_ptr;
  struct biquad_pin_ctx_t *pins = (struct biquad_pin_ctx_t *)pin_ptr;

  PIN(type)  = 0.0;
  PIN(f0)    = 20.0;
  PIN(q)     = 2.0;
  PIN(depth) = 0.0;
  ctx->type  = -1.0;  // forces a coefficient update on the first rt call
}

static void coeffs(struct biquad_ctx_t *ctx, float in, float period) {
  float type  = ctx->type;
  float f0    = CLAMP(ctx->f0, 1.0, 0.45 / period);
  float q     = MAX(ctx->q, 0.1);
  float w0    = 2.0 * M_PI * f0 * period;
  float cw    = cosf(w0);
  float alpha = sinf(w0) / (2.0 * q);
  float a0    = 1.0 + alpha;

  if(type == 1.0) {
    float d = MAX(ctx->depth, 0.0);
    ctx->b0 = (1.0 + alpha * d) / a0;
    ctx->b1 = -2.0 * cw / a0;
    ctx->b2 = (1.0 - alpha * d) / a0;
  } else if(type == 2.0) {
    ctx->b0 = (1.0 - cw) / 2.0 / a0;
    ctx->b1 = (1.0 - cw) / a0;
    ctx->b2 = ctx->b0;
  } else {
    ctx->b0 = 1.0;
    ctx->b1 = 0.0;
    ctx->b2 = 0.0;
    ctx->a1 = 0.0;
    ctx->a2 = 0.0;
    ctx->z1 = 0.0;
    ctx->z2 = 0.0;
    return;
  }
  ctx->a1 = -2.0 * cw / a0;
  ctx->a2 = (1.0 - alpha) / a0;

  // start from the steady state for the current input (both types pass DC)
  float y = in;
  ctx->z1 = y - ctx->b0 * in;
  ctx->z2 = ctx->b2 * in - ctx->a2 * y;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct biquad_ctx_t *ctx      = (struct biquad_ctx_t *)ctx_ptr;
  struct biquad_pin_ctx_t *pins = (struct biquad_pin_ctx_t *)pin_ptr;

  float in = PIN(in);
  if(PIN(type) != ctx->type || PIN(f0) != ctx->f0 || PIN(q) != ctx->q || PIN(depth) != ctx->depth || period != ctx->period) {
    ctx->type   = PIN(type);
    ctx->f0     = PIN(f0);
    ctx->q      = PIN(q);
    ctx->depth  = PIN(depth);
    ctx->period = period;
    coeffs(ctx, in, period);
  }

  // direct form II transposed
  float y = ctx->b0 * in + ctx->z1;
  ctx->z1 = ctx->b1 * in - ctx->a1 * y + ctx->z2;
  ctx->z2 = ctx->b2 * in - ctx->a2 * y;
  if(!isfinite(y)) {  // NaN in: reset instead of latching
    y       = 0.0;
    ctx->z1 = 0.0;
    ctx->z2 = 0.0;
  }
  PIN(out) = y;
}

hal_comp_t biquad_comp_struct = {
    .name      = "biquad",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct biquad_ctx_t),
    .pin_count = sizeof(struct biquad_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
