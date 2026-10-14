#include "hv_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "periph.h"
#include "f3hw.h"
#include "common.h"

/**
* ## Brief
* `hv` on the F3 (HV board) side: it turns the three phase voltages from `idq0` into TIM8 compare values that drive the power stage. It adds a dead time compensation, then the space vector (midpoint) common mode offset, keeps every pulse above a minimum on and off time, follows the PWM period that `ls0` sets to phase lock the board to the F4, and holds all low sides on while `io0` short-circuit brakes. It is loaded by `stm32f303/src/main.c` as `hv0` (rt_prio 6, after `idq0`) and is not configured by the user directly: its settings (`drop_k`, `drop_knee`, `arr`) come from the F4 through `ls0`. This page is the F3 component and is published as `hv_f3`. The F4-side counterpart, which configures the HV board and sends it the commands, is the `hv` component.
*
* ## Component Explanation
*
* 1. **Wiring on the F3** (fixed in main.c):
* - `hv0.u/v/w = idq0.u/v/w` (phase voltages centred on 0; `svm0` is no longer loaded, `hv` takes the offset itself), `hv0.udc = io0.udc_duty` (the fast link voltage), `hv0.arr = ls0.arr`, `hv0.drop_k = ls0.drop_k`, `hv0.drop_knee = ls0.drop_knee`, `hv0.sbrake = io0.sbrake_on`, and `ls0.duty_max = hv0.duty_max`.
* - For the dead time compensation: `hv0.d_cmd/q_cmd = ls0.d_cmd/q_cmd`, `hv0.si/co = idq0.si_out/co_out` (sin/cos of the voltage angle `angle0.pos_v`), `hv0.id_fb/iq_fb = dq0.d/q`, `hv0.cmd_mode = ls0.cmd_mode`, `hv0.phase_mode = ls0.phase_mode`.
* - `drop_band`, `drop_volt`, `drop_vlp`, `min_on` and `min_off` are not wired and not F4 config words: they keep their nrt_init defaults unless changed from the F3 terminal.
* - All three half bridges always switch.
*
* 2. **PWM period** (rt):
* - `arr` is written to the TIM8 auto reload register every tick. `ls0` moves it by a few counts once per F4 packet to lock the PWM to the F4's packets. nrt_init sets `arr` = `PWM_RES`.
* - TIM8 runs centre aligned at 144 MHz (`PWM_TIM_CLK`), so one PWM period is 2 * ARR ticks. The PWM (and F3 rt) frequency `PWM_FREQ` is a build option (`stm32f303/Makefile`, 10, 15 or 20 kHz, default 15 kHz; the F4 reads it back as `hv0.pwm_freq`), and `PWM_RES = PWM_TIM_CLK / (2 * PWM_FREQ)` is 7200, 4800 or 3600.
*
* 3. **Dead time compensation** (rt):
* - While a phase current keeps its sign the bridge loses the dead time's share of the PWM period times the link voltage. The size of the correction in volts (`udc` has a floor of 0.1 V):
* ```c
* dt_drop = drop_k * PWM_DEADTIME_TICKS / (2 * pwm_res) * udc;
* ```
* - `PWM_DEADTIME` is the BDTR.DTG code 196, which decodes to 288 ticks (2.0 us), so `drop_k` = 1 is 3 % of `udc` at 15 kHz (2 % at 10 kHz, 4 % at 20 kHz). A `drop_k` below 1 accounts for the part of the dead time the switches' own delays eat. The result is clamped to 0..10 % of `udc`, so a mistyped `drop_k` distorts the waveform but cannot harm the bridge.
* - Its sign per phase comes from a reference current, never the instantaneous measured one (keyed on that, the compensation drives the current it reads and runs away). The reference phase currents `iu`, `iv`, `iw` are built from a d/q reference with `si`/`co` (inverse Park and 120 deg inverse Clarke), every tick, also when the compensation is off. `si`/`co` belong to the voltage angle, the angle at which this tick's voltage is applied on average (see `angle0.pos_v` and `ls0.v_lead`).
* - Voltage mode (`cmd_mode` = 0, e.g. the spindle's V/f): there is no current command, so the reference is the measured `id_fb`/`iq_fb`, low pass filtered in the d/q frame with the time constant `drop_vlp` (floor one period), so the sign follows the fundamental and not the ripple. The compensation is only applied here when `drop_volt` > 0 (only its sign is used).
* - `drop_knee` = 0 (default): latched sign. Per phase a Schmitt trigger flips to +1 above `drop_band` and to -1 below `-drop_band` and holds its value in between. The sign starts at 0, and rt_start resets it to 0, so nothing is added until a phase has carried current once. The price is a sign that is stale while a zero crossing passes through the band, and an axis that draws less than `drop_band` never flips it at all.
* - `drop_knee` > 0 (A): curve instead of latch. Each phase gets `dt_drop * k(i)` with the sign of its reference current, where
* ```c
* k(i) = 1 - 1 / (1 + |i| / drop_knee)^2;
* ```
* - This follows the real loss, which fades out at low current (where the ripple carries the phase through zero inside the PWM period) and settles above a couple of amps. It cannot stick like the latch. `drop_knee` has a floor of 0.02 A.
*
* 4. **Space vector offset** (rt):
* - The compensation is added to `u`, `v`, `w` first, then the common mode is chosen on the compensated phases: `off = (min + max) / 2 - udc / 2`, subtracted from all three, so the midpoint of the highest and lowest phase sits at `udc / 2`. Because the offset sees the compensation, near full modulation the clamp below no longer cuts it.
* - Each phase is then converted to a compare value, `CLAMP(x, 0, udc) / udc * pwm_res`.
*
* 5. **Minimum on and off time** (rt):
* - `min_on` and `min_off` are converted to compare units at `PWM_TIM_CLK / 2` (one compare unit is 2 timer ticks in centre aligned mode, whatever ARR is), 216 units at 3 us. Both have a floor of the dead time plus 0.5 us (2.5 us): a shorter high side pulse never turns on through the driver interlock, and a shorter low side pulse leaves nothing for the current sample.
* - `duty_max = (pwm_res - min_on - min_off) / pwm_res` is output every tick (0.91 at the defaults and 15 kHz, 0.94 at 10 kHz, 0.88 at 20 kHz). `ls0` scales `pwm_volt` by it, so `curpid0`'s voltage limit matches what this stage passes.
* - If the spread between the highest and lowest phase is larger than `pwm_res - min_on - min_off`, all three are scaled down around their centre (a float scale).
* - Then, judged on the values before any shift: if any phase is above 0 but below `min_on`, all three are moved up by what the lowest phase lacks (`min_on - min`); if the highest phase is above `pwm_res - min_off`, all three are moved down by its excess. The shift is only the deficit, so the neutral moves no more than needed; line to line voltages are unchanged.
* - Finally each phase between 0 and `min_on` is raised to `min_on`, and every phase is clamped to 0..`pwm_res - min_off`. A phase at exactly 0 (low side on for the whole period) is allowed.
*
* 6. **Short-circuit braking** (rt):
* - While `sbrake` > 0 (`io0.sbrake_on`) all three compares are forced to 0, so the low side switches of all phases are on and the back EMF drives current round the windings. `io0` switches the bridge on and off to chop that current; `hv` only provides the compare values.
*
* 7. **Output**:
* - The values go to `PWM_U/V/W` (TIM8 CCR3/CCR2/CCR1), inverted when the board defines `PWM_INVERT`.
*
* {{% hint warning %}}
* - `hv` has no enable or fault handling of its own. The bridge is switched on and off by `io0` (TIM8 MOE and the driver enable pin) and, on an overcurrent comparator break, by the hardware break plus the TIM8 break interrupt in main.c, so `hv` keeps writing compare values even while the bridge is off.
* - The voltage mode compensation (`drop_volt`) is new and off by default; it has not been tried on the spindle yet.
* - The compensation is added on top of the voltage `curpid0` was limited to, so near full modulation the spread can exceed `pwm_res - min_on - min_off` again and the spread scaling cuts it by a few percent.
* - `ls0.pwm_volt` is computed from the slow `io0.udc`, while this component divides by the fast `io0.udc_duty`, so during a fast link voltage change the two limits differ briefly.
* {{% /hint %}}
*/

HAL_COMP(hv);

// dead time compensation, see rt_func. Its sign comes from the commanded
// phase currents, never the measured ones: keyed on measured current the
// compensation drives the current it reads and runs away.
HAL_PIN(drop_k);      // *input*, Dead time compensation as a fraction of the ideal dead time drop, 0 = off, from ls0.drop_k, default 0
HAL_PIN(drop_band);   // *parameter*, Latched sign flips outside +-drop_band (A), min 0.05, not wired, default 0.5
HAL_PIN(drop_knee);   // *input*, > 0: knee of the compensation curve on the reference current (A), 0 = latched sign, from ls0.drop_knee
HAL_PIN(cmd_mode);    // *input*, Command mode from ls0, 0 = voltage (reference from id_fb/iq_fb), else current (reference from d_cmd/q_cmd)
HAL_PIN(phase_mode);  // *input*, Phase mode from ls0, the compensation runs only in 120 deg 3ph mode (2)
HAL_PIN(d_cmd);       // *input*, D-axis current command from the F4 (A), reference for the compensation in current mode
HAL_PIN(q_cmd);       // *input*, Q-axis current command from the F4 (A), reference for the compensation in current mode
HAL_PIN(si);          // *input*, Sine of the voltage angle, from idq0.si_out
HAL_PIN(co);          // *input*, Cosine of the voltage angle, from idq0.co_out
HAL_PIN(iu);          // *output*, Reference U phase current for the compensation sign (A)
HAL_PIN(iv);          // *output*, Reference V phase current for the compensation sign (A)
HAL_PIN(iw);          // *output*, Reference W phase current for the compensation sign (A)
// Volt mode (vf, the spindle): the reference is the measured current vector,
// low passed in the dq frame with time constant drop_vlp [s], so the sign
// follows the fundamental and not the ripple. drop_volt > 0 turns it on.
HAL_PIN(drop_volt);  // *parameter*, > 0 turns the compensation on in voltage mode, not wired, default 0
HAL_PIN(drop_vlp);   // *parameter*, Low pass time constant of the voltage mode reference current (s), not wired, default 0.003
HAL_PIN(id_fb);      // *input*, Measured d-axis current (A), from dq0.d, voltage mode reference
HAL_PIN(iq_fb);      // *input*, Measured q-axis current (A), from dq0.q, voltage mode reference

//U V W input in Volt: idq0's phase voltages, before the common mode. The
//dead-time compensation is added to them and the space vector offset is
//taken after that, so it sees the compensated phases (svm is not built here).
HAL_PIN(u);  // *input*, U phase voltage centred on 0 (V), from idq0.u
HAL_PIN(v);  // *input*, V phase voltage centred on 0 (V), from idq0.v
HAL_PIN(w);  // *input*, W phase voltage centred on 0 (V), from idq0.w

//dclink in, to scale pwm
HAL_PIN(udc);  // *input*, DC link voltage (V), from io0.udc_duty, scales the PWM, min 0.1

HAL_PIN(min_on);    // *parameter*, Minimum on time (s), floored at dead time + 0.5 us, default 3 us
HAL_PIN(min_off);   // *parameter*, Minimum off time (s), floored at dead time + 0.5 us, default 3 us
HAL_PIN(duty_max);  // *output*, Usable duty after min_on and min_off (0..1), to ls0.duty_max

HAL_PIN(arr);  // *input*, PWM timer reload value from ls0.arr, clamped to 90..110 % of PWM_RES (4800 at 15 kHz)

HAL_PIN(sbrake);  // *input*, Short-circuit braking from io0.sbrake_on, > 0 forces all compares to 0 (all low sides on)

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
  //will actually pass.
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

  // Common-mode shift by the deficit, not by the whole limit. Shifting by the
  // whole limit, a phase sitting 20 units inside min_on would drag all three
  // up by the full min_on (216 units, 4.5% of the link): the line voltages
  // survive a common-mode move, but the neutral would jump by that much every
  // time a phase crossed the boundary, and with it the common-mode current
  // through the motor's capacitance. Moving by just what the offending phase lacks puts it exactly
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
