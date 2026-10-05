#include "hv_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "periph.h"
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
// Volt mode (vf, the spindle): the reference is the measured current vector,
// low passed in the dq frame with time constant drop_vlp [s], so the sign
// follows the fundamental and not the ripple. drop_volt > 0 turns it on.
HAL_PIN(drop_volt);
HAL_PIN(drop_vlp);
HAL_PIN(id_fb);
HAL_PIN(iq_fb);

//U V W input in Volt: idq0's phase voltages, before the common mode. The
//dead-time compensation is added to them and the space vector offset is
//taken after that, so it sees the compensated phases (svm0 is not used).
HAL_PIN(u);
HAL_PIN(v);
HAL_PIN(w);

//dclink in, to scale pwm
HAL_PIN(udc);

//TODO: half bridge enable in
HAL_PIN(enu);
HAL_PIN(env);
HAL_PIN(enw);

HAL_PIN(min_on);    // min on time [s], floored at dead time + 0.5 us
HAL_PIN(min_off);   // min off time [s], floored at dead time + 0.5 us
HAL_PIN(duty_max);  // usable duty after min_on/min_off, for ls0.pwm_volt

HAL_PIN(arr);

HAL_PIN(sbrake);  // io0.sbrake_on: all compares 0, so the low sides carry the phases

struct hv_ctx_t {
  int32_t pwm_res;
  float id_lp;
  float iq_lp;
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
  PIN(drop_volt) = 0.0;
  PIN(drop_vlp)  = 0.003;
  ctx->id_lp     = 0.0;
  ctx->iq_lp     = 0.0;
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
  LL_TIM_SetAutoReload(TIM8, ctx->pwm_res);

  float udc     = MAX(PIN(udc), 0.1);
  float udc_inv = 1.0 / udc;                   // one division, used three times below
  float res_inv = 1.0 / (float)ctx->pwm_res;  // and this one twice

  // Dead time compensation. While a phase current keeps its sign the bridge
  // loses the dead time's share of the pwm period times the link voltage.
  // Both are timer ticks, so only their ratio is needed, taken against the
  // live pwm_res that ls.c walks to phase lock. drop_k scales that ideal loss
  // (the switches' own delays eat part of the dead time).
  //
  // In volt mode there is no commanded current to take the sign from. The
  // instantaneous measured one would close a loop and run away; its low
  // passed fundamental does not (drop_volt, off by default).
  float dt_drop = PIN(drop_k) * (float)PWM_DEADTIME_TICKS * 0.5 * res_inv * udc;
  float d_ref   = PIN(d_cmd);
  float q_ref   = PIN(q_cmd);
  if(PIN(cmd_mode) == VOLT_MODE) {
    float k = CLAMP(period / MAX(PIN(drop_vlp), period), 0.0, 1.0);
    ctx->id_lp += (PIN(id_fb) - ctx->id_lp) * k;
    ctx->iq_lp += (PIN(iq_fb) - ctx->iq_lp) * k;
    d_ref = ctx->id_lp;
    q_ref = ctx->iq_lp;
    if(PIN(drop_volt) <= 0.0) {
      dt_drop = 0.0;
    }
  }
  if(PIN(phase_mode) != PHASE_120_3PH) {
    dt_drop = 0.0;
  }
  // a mistyped drop_k costs a distorted waveform, not a bridge
  dt_drop = CLAMP(dt_drop, 0.0, udc * 0.1);

  // reference phase currents, inverse park and clarke of the command
  float a = d_ref * PIN(co) - q_ref * PIN(si);
  float b = d_ref * PIN(si) + q_ref * PIN(co);
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
  // space vector common mode (midpoint) on the compensated phases, so near
  // full modulation the compensation is not cut by the clamp below
  float off = (MIN3(uu, uv, uw) + MAX3(uu, uv, uw)) / 2.0 - udc / 2.0;
  uu -= off;
  uv -= off;
  uw -= off;

  //convert voltages to PWM output compare values
  int32_t u = (int32_t)(CLAMP(uu, 0.0, udc) * udc_inv * (float)(ctx->pwm_res));
  int32_t v = (int32_t)(CLAMP(uv, 0.0, udc) * udc_inv * (float)(ctx->pwm_res));
  int32_t w = (int32_t)(CLAMP(uw, 0.0, udc) * udc_inv * (float)(ctx->pwm_res));
  //convert on and off times to PWM output compare values.
  //TIM8 is center aligned, so a PWM period spans 2*ARR timer ticks and one
  //compare unit is worth 2 ticks of on time -- the on time resolution is a
  //constant PWM_TIM_CLK/2, independent of ARR (which ls.c varies to phase
  //lock the loop to the master).
  //
  //Neither may go below the dead time plus half a microsecond: a high side
  //pulse shorter than the dead time never turns on at all (the driver's
  //interlock eats it), and a low side pulse that short leaves nothing for the
  //current sample, so a smaller setting would not buy resolution, only a dead
  //band at the ends of the range.
  float t_min     = (float)PWM_DEADTIME_TICKS / PWM_TIM_CLK + 0.5e-6;
  int32_t min_on  = (int32_t)(PWM_TIM_CLK / 2.0 * MAX(PIN(min_on), t_min) + 0.5);
  int32_t min_off = (int32_t)(PWM_TIM_CLK / 2.0 * MAX(PIN(min_off), t_min) + 0.5);

  //what is left for the line to line voltage once both ends are reserved.
  //ls.c scales pwm_volt by it, so curpid's ceiling matches what this clamp
  //will actually pass rather than a fixed 95%.
  PIN(duty_max) = (float)MAX(ctx->pwm_res - min_on - min_off, 0) * res_inv;

  // a phase spread wider than min_on/min_off leave room for cannot be
  // fixed by a common mode shift: scale it down around its center
  int32_t max_range = ctx->pwm_res - min_on - min_off;
  int32_t range      = MAX3(u, v, w) - MIN3(u, v, w);
  if(max_range > 0 && range > max_range) {
    int32_t center = (MAX3(u, v, w) + MIN3(u, v, w)) / 2;
    float k        = (float)max_range / (float)range;  // no int64 division from flash
    u              = center + (int32_t)((float)(u - center) * k);
    v              = center + (int32_t)((float)(v - center) * k);
    w              = center + (int32_t)((float)(w - center) * k);
  }

  // Common-mode shift by the deficit, not by the whole limit. A phase sitting
  // 20 units inside min_on used to drag all three up by the full min_on (216
  // units, 4.5% of the link): the line voltages survive a common-mode move,
  // but the neutral jumped by that much every time a phase crossed the
  // boundary, and with it the common-mode current through the motor's
  // capacitance. Moving by just what the offending phase lacks puts it exactly
  // on the limit, and the spread scaling above guarantees the far phase still
  // clears the other end.
  //
  // Both deficits come from the pre-shift values, so one shift cannot trigger
  // the other; after the scaling they cannot both be nonzero anyway. A phase
  // at exactly 0 is fully off and legal on its own, but once any phase needs
  // the min_on shift it comes along, to min_on, since the shift is common mode.
  int32_t lo  = MIN3(u, v, w);
  int32_t hi  = MAX3(u, v, w);
  int32_t up  = 0;
  int32_t dn  = 0;

  if((u > 0 && u < min_on) || (v > 0 && v < min_on) || (w > 0 && w < min_on)) {
    up = min_on - lo;
  }

  if(hi > ctx->pwm_res - min_off) {
    dn = hi - (ctx->pwm_res - min_off);
  }

  u += up - dn;
  v += up - dn;
  w += up - dn;

  // final per-phase safety net: guarantees every phase individually clears
  // both min_on and min_off even when a single common-mode shift above
  // can't satisfy both boundaries at once (e.g. one phase near 0 and
  // another near full scale in the same cycle -- a double violation)
  if(u > 0 && u < min_on) u = min_on;
  if(v > 0 && v < min_on) v = min_on;
  if(w > 0 && w < min_on) w = min_on;

  u = CLAMP(u, 0, ctx->pwm_res - min_off);
  v = CLAMP(v, 0, ctx->pwm_res - min_off);
  w = CLAMP(w, 0, ctx->pwm_res - min_off);

  if(PIN(sbrake) > 0.0) {
    u = 0;
    v = 0;
    w = 0;
  }

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
