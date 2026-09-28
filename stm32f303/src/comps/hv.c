#include "hv_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "tim.h"
#include "f3hw.h"
#include "common.h"

HAL_COMP(hv);

// dead time compensation, see rt_func. Its sign comes from the commanded
// phase currents, never the measured ones: keyed on measured current the
// compensation drives the current it reads and runs away.
HAL_PIN(drop_k);      // compensation, fraction of the ideal PWM_DEADTIME volts
HAL_PIN(drop_band);   // latched sign flips outside +-drop_band [A]
HAL_PIN(drop_knee);   // >0: curve knee on the commanded current [A], 0 = latched sign
HAL_PIN(cmd_mode);    // compensation in current mode only
HAL_PIN(phase_mode);  // compensation in 120 deg 3ph mode only
HAL_PIN(d_cmd);       // command from the f4
HAL_PIN(q_cmd);
HAL_PIN(si);  // dq0's sin and cos of the same angle
HAL_PIN(co);
HAL_PIN(iu);  // reference phase currents, outputs
HAL_PIN(iv);
HAL_PIN(iw);

//U V W input in Volt
HAL_PIN(u);
HAL_PIN(v);
HAL_PIN(w);

//dclink in, to scale pwm
HAL_PIN(udc);

//TODO: half bridge enable in
HAL_PIN(enu);
HAL_PIN(env);
HAL_PIN(enw);

HAL_PIN(min_on);   // min on time [s]
HAL_PIN(min_off);  // min off time [s]

HAL_PIN(arr);

struct hv_ctx_t {
  int32_t pwm_res;
  int8_t drop_su;
  int8_t drop_sv;
  int8_t drop_sw;
};

// Schmitt trigger sign: holds inside +-band, so it has no incremental gain
// to excite. The price is a stale sign for the time a zero crossing takes to
// cross the band.
static float drop_sign(int8_t *s, float i, float band) {
  if(i > band) {
    *s = 1;
  } else if(i < -band) {
    *s = -1;
  }
  return (float)(*s);
}

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx      = (struct hv_ctx_t *)ctx_ptr;
  struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;

  ctx->drop_su = 0;
  ctx->drop_sv = 0;
  ctx->drop_sw = 0;

  PIN(enu)       = 1.0;
  PIN(env)       = 1.0;
  PIN(enw)       = 1.0;
  PIN(min_on)    = 0.000003;
  PIN(min_off)   = 0.000003;
  PIN(arr)       = PWM_RES;
  PIN(drop_k)    = 0;
  PIN(drop_band) = 0.5;
  PIN(drop_knee) = 0.0;
}

// the latches survive a stop; start without a stale sign
static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx = (struct hv_ctx_t *)ctx_ptr;

  ctx->drop_su = 0;
  ctx->drop_sv = 0;
  ctx->drop_sw = 0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct hv_ctx_t *ctx      = (struct hv_ctx_t *)ctx_ptr;
  struct hv_pin_ctx_t *pins = (struct hv_pin_ctx_t *)pin_ptr;

  ctx->pwm_res = (int32_t)CLAMP(PIN(arr), PWM_RES * 0.9, PWM_RES * 1.1);
  TIM8->ARR    = ctx->pwm_res;

  float udc = MAX(PIN(udc), 0.1);

  // Dead time compensation. While a phase current keeps its sign the bridge
  // loses the dead time's share of the pwm period times the link voltage.
  // Both are timer ticks, so only their ratio is needed, taken against the
  // live pwm_res that ls.c walks to phase lock. drop_k scales that ideal loss
  // (the switches' own delays eat part of the dead time).
  //
  // Current mode only: in volt mode there is no commanded current to take the
  // sign from, and the measured one would close the loop again.
  float dt_drop = PIN(drop_k) * (float)PWM_DEADTIME_TICKS / (2.0 * (float)ctx->pwm_res) * udc;
  if(PIN(cmd_mode) == VOLT_MODE || PIN(phase_mode) != PHASE_120_3PH) {
    dt_drop = 0.0;
  }
  // a mistyped drop_k costs a distorted waveform, not a bridge
  dt_drop = CLAMP(dt_drop, 0.0, udc * 0.1);

  // reference phase currents, inverse park and clarke of the command
  float a = PIN(d_cmd) * PIN(co) - PIN(q_cmd) * PIN(si);
  float b = PIN(d_cmd) * PIN(si) + PIN(q_cmd) * PIN(co);
  PIN(iu) = a;
  PIN(iv) = -a / 2.0 + b / 2.0 * M_SQRT3;
  PIN(iw) = -a / 2.0 - b / 2.0 * M_SQRT3;

  float uu, uv, uw;
  if(PIN(drop_knee) > 0.0) {
    // k(i) = 1 - 1 / (1 + |i| / knee)^2, the shape of the real loss: it falls
    // away at low current, where the ripple carries the phase through zero
    // inside the pwm period, and settles above. No latch to stick on an
    // axis drawing less than drop_band.
    float kk = 1.0 / MAX(PIN(drop_knee), 0.02);
    float au = 1.0 + ABS(PIN(iu)) * kk, av = 1.0 + ABS(PIN(iv)) * kk, aw = 1.0 + ABS(PIN(iw)) * kk;
    float ku = 1.0 - 1.0 / (au * au), kv = 1.0 - 1.0 / (av * av), kw = 1.0 - 1.0 / (aw * aw);
    uu       = PIN(u) + dt_drop * (PIN(iu) >= 0.0 ? ku : -ku);
    uv       = PIN(v) + dt_drop * (PIN(iv) >= 0.0 ? kv : -kv);
    uw       = PIN(w) + dt_drop * (PIN(iw) >= 0.0 ? kw : -kw);
  } else {
    float band = MAX(PIN(drop_band), 0.05);
    uu         = PIN(u) + dt_drop * drop_sign(&(ctx->drop_su), PIN(iu), band);
    uv         = PIN(v) + dt_drop * drop_sign(&(ctx->drop_sv), PIN(iv), band);
    uw         = PIN(w) + dt_drop * drop_sign(&(ctx->drop_sw), PIN(iw), band);
  }
  //convert voltages to PWM output compare values
  int32_t u = (int32_t)(CLAMP(uu, 0.0, udc) / udc * (float)(ctx->pwm_res));
  int32_t v = (int32_t)(CLAMP(uv, 0.0, udc) / udc * (float)(ctx->pwm_res));
  int32_t w = (int32_t)(CLAMP(uw, 0.0, udc) / udc * (float)(ctx->pwm_res));
  // center aligned: one compare unit is 2 timer ticks whatever ARR is
  int32_t min_on  = (int32_t)(PWM_TIM_CLK / 2.0 * PIN(min_on) + 0.5);
  int32_t min_off = (int32_t)(PWM_TIM_CLK / 2.0 * PIN(min_off) + 0.5);

  // a phase spread wider than min_on/min_off leave room for cannot be
  // fixed by a common mode shift: scale it down around its center
  int32_t max_range = ctx->pwm_res - min_on - min_off;
  int32_t range      = MAX3(u, v, w) - MIN3(u, v, w);
  if(max_range > 0 && range > max_range) {
    int32_t center = (MAX3(u, v, w) + MIN3(u, v, w)) / 2;
    u              = center + (int32_t)(((int64_t)(u - center) * max_range) / range);
    v              = center + (int32_t)(((int64_t)(v - center) * max_range) / range);
    w              = center + (int32_t)(((int64_t)(w - center) * max_range) / range);
  }

  // both checks on the uncorrected values, so one shift cannot undo the other
  int32_t u0 = u, v0 = v, w0 = w;

  if((u0 > 0 && u0 < min_on) || (v0 > 0 && v0 < min_on) || (w0 > 0 && w0 < min_on)) {
    u += min_on;
    v += min_on;
    w += min_on;
  }

  if((u0 > ctx->pwm_res - min_off) || (v0 > ctx->pwm_res - min_off) || (w0 > ctx->pwm_res - min_off)) {
    u -= min_off;
    v -= min_off;
    w -= min_off;
  }

  // per phase floor for what the shifts could not fix
  if(u > 0 && u < min_on) u = min_on;
  if(v > 0 && v < min_on) v = min_on;
  if(w > 0 && w < min_on) w = min_on;

  u = CLAMP(u, 0, ctx->pwm_res - min_off);
  v = CLAMP(v, 0, ctx->pwm_res - min_off);
  w = CLAMP(w, 0, ctx->pwm_res - min_off);

#ifdef PWM_INVERT
  PWM_U = ctx->pwm_res - u;
  PWM_V = ctx->pwm_res - v;
  PWM_W = ctx->pwm_res - w;
#else
  PWM_U = u;
  PWM_V = v;
  PWM_W = w;
#endif
}

hal_comp_t hv_comp_struct = {
    .name      = "hv",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct hv_ctx_t),
    .pin_count = sizeof(struct hv_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
