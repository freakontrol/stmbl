#include "flux_obs_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `flux_obs` is a hybrid rotor flux observer for an induction motor on an encoder (review item B5). It measures the rotor flux angle and magnitude from voltage and current, without tr, and can trim the current model's slip so that the field orientation follows the measured flux at speed. Below its crossover it falls back to the current model (`acim_flux`), so at low speed nothing changes.
*
* **Status (14 Oct):** shadow (`pos_err`, `psi`) and the slip trim are bench-checked on the spindle, cold and warm, without load; untested under a cutting load.
*
* `conf/template/flux_obs.txt` loads it after `acim_foc` (`link flux_obs`) in shadow mode, at `rt_prio` 9.4, with `en = fault0.en_out`, `r = conf0.r`, `l = conf0.l`, `adv = hv0.adv`, `id`/`iq`/`ud`/`uq = hv0.*_fb`, `pos_ref = angle0.pos`, `vel_ref = angle0.vel`, `psi_cm = acim_flux0.psi`, `slip_in = acim_flux0.slip` and `slip_max = acim_flux0.slip_max`. `flux_obs0.pos_err` then shows how far the current model's orientation is off at speed (tr). To trim the slip with it: `flux_obs0.kp_s = 20`, `flux_obs0.ki_s = 2` and `angle0.slip = flux_obs0.slip`; `flux_obs0.k` is then true / modelled slip.
*
* ## Component Explanation
* All work is done in `rt`; `rt_start` clears the observer and sets `k` to 1.
*
* 1. **Model** (Jansen / Lorenz, stationary frame):
* - With `l = sigma*Ls` and `psi = lmr * i_mr` (the rotor flux seen from the stator, as `acim_flux0.psi`):
* - Voltage model: `d(psi_s)/dt = u - r i`.
* - Current model: `psi_s_cm = psi_cm e^(j pos_ref) + l i`.
* - Hybrid: `d(psi_s)/dt = u - r i + kp (psi_s_cm - psi_s) + ki int(psi_s_cm - psi_s)`, with `kp = 1.4 wc`, `ki = wc^2`.
* - Rotor flux: `psi_r = psi_s - l i`; `psi` is its magnitude, `pos` its angle.
* - The estimate follows the current model below about `wc` and the voltage model above it, where only `r` and `l` matter and r's share falls with speed. The current model term holds the voltage integrator's drift (offsets, r error).
* - The resistance used is `r + r_w * abs(vel_ref)`.
* - While `en <= 0` the stator flux is set to the current model's and the integral cleared.
*
* 2. **Frames**:
* - Like `obs`: the feedback (`hv0.id_fb` ...) is in the frame `pos_ref + vel_ref * adv` (`angle0.pos`, `hv0.adv`). Run `flux_obs` before `angle0` (lower `rt_prio`), so `pos_ref` is the angle the feedback was taken with.
* - The voltages are the mean over the rt period before the sample, which the frame turned through, so they sit `u_adv` off the current's frame: half an rt period (`u_adv` 0 = `-period / 2`, -100 us at 5 kHz).
* - Do not calibrate `u_adv` on idle holds: on the spindle (13 Oct) -30 us zeroed `pos_err` there, but that hid the current model's slip error and left 0.17 rad while braking. At -100 us the rotor resistance seen by the observer (`slip * psi / iq`) is 0.121 to 0.128 in run-up, hold and both brakes, at -30 us 0.09 to 0.15.
* - `pos_err = pos` minus that frame: how far the current model's orientation is off, 0 below the crossover (and 0 while `psi` is below 1e-4).
*
* 3. **Slip trim** (off by default, shadow):
* - `slip = slip_in * k + kp_s * pos_err`.
* - `kp_s` pulls the frame onto the measured flux. The pull is bounded to `0.5 * abs(slip_in) + slip_min`, so a wrong angle (voltage limit, sensor fault, transient) cannot drive the frame off by more (it was open: +-63 rad/s at `kp_s` 20).
* - `ki_s` moves `k`, the ratio of true to modelled slip (tr / true tr), so `pos_err` goes to 0 in steady state and a load step finds the right slip at once. Flux ahead of the frame (`pos_err > 0`) means the true slip is larger. `k` stays within 0.5 to 2.
* - Both act only while the gate is open (`trim` = 1): `en > 0`, `abs(vel_ref) > vel_min`, `abs(psi_cm) > 1e-3` and `psi > psi_min * abs(psi_cm)`. `k` moves only while also `ki_s > 0` and `abs(slip_in) > slip_min`.
* - With `acc_max > 0`, `k` also holds while `abs(acc)` (the `vel_ref` derivative, 5 Hz low pass) is above it. With `u_adv` right `k` is the same in run-up, hold and brake (spindle 13 Oct: within 0.01 in field weakening), so the gate is only for a doubtful `u_adv`.
* - `k` starts at 1 when the drive starts (`rt_start`) and otherwise keeps its value while the gates are shut or `en` is 0, so `slip = slip_in * k` still applies there: `k` tracks the rotor temperature, which changes slowly, so a new enable starts from the last learnt ratio.
* - The trimmed slip is clamped to `+-slip_max` when `slip_max > 0`, as `acim_flux` clamps its own.
* - `kp_s` 0 and `ki_s` 0: `slip = slip_in` (with `k` at 1), the observer only reports. Wire `angle0.slip = flux_obs0.slip` to use it.
*
* 4. **Results**:
* - Spindle bench (13 Oct, unloaded): shadow (`pos_err`, `psi`) checked.
* - Spindle bench (14 Oct, no load, template): shadow within 0.03 rad cold. Trim `kp_s` 20, `ki_s` 2 follows the rotor temperature (`k` 1.0 cold, 1.07 to 1.11 after 20 min at 2352 rpm) and holds `pos_err` within 0.02 rad.
*
* {{% hint warning %}}
* The slip trim is not tested under a cutting load yet.
* {{% /hint %}}
*/

HAL_COMP(flux_obs);

HAL_PIN(r);         // *parameter*, Stator resistance (ohm, default 0.2), conf0.r
HAL_PIN(r_w);       // *parameter*, Resistance growing with speed, d loss (ohm per rad/s electrical, default 0), as obs0.r_w
HAL_PIN(l);         // *parameter*, Leakage inductance sigma*Ls (H, default 0.001), conf0.l
HAL_PIN(wc);        // *parameter*, Crossover from the current to the voltage model (rad/s electrical, default 20)
HAL_PIN(adv);       // *parameter*, Feedback frame lead over pos_ref (s), hv0.adv
HAL_PIN(u_adv);     // *parameter*, Voltage frame lead over the current's (s, default 0 = -half the rt period)
HAL_PIN(kp_s);      // *parameter*, Frame pull onto the measured flux (1/s, default 0 = off)
HAL_PIN(ki_s);      // *parameter*, Rate of the slip ratio k (1/(rad s), default 0 = off)
HAL_PIN(vel_min);   // *parameter*, Trim only above this abs(vel_ref) (rad/s electrical, default 150)
HAL_PIN(psi_min);   // *parameter*, Trim only while psi > psi_min * abs(psi_cm) (default 0.5)
HAL_PIN(slip_min);  // *parameter*, k moves only above this abs(slip_in), also widens the kp_s bound (rad/s electrical, default 2)
HAL_PIN(acc_max);   // *parameter*, k moves only below this abs(acc) (rad/s^2 electrical, default 0 = no gate)
HAL_PIN(slip_max);  // *input*, Clamp on the trimmed slip (rad/s electrical, 0 = none), acim_flux0.slip_max

HAL_PIN(en);        // *input*, Enable, 0 = flux estimate follows the current model and trim off (k is kept), fault0.en_out
HAL_PIN(id);        // *input*, d current feedback (A), hv0.id_fb
HAL_PIN(iq);        // *input*, q current feedback (A), hv0.iq_fb
HAL_PIN(ud);        // *input*, d voltage feedback (V), hv0.ud_fb
HAL_PIN(uq);        // *input*, q voltage feedback (V), hv0.uq_fb
HAL_PIN(pos_ref);   // *input*, Angle the feedback frame was at (rad electrical), angle0.pos
HAL_PIN(vel_ref);   // *input*, Frame velocity (rad/s electrical), angle0.vel
HAL_PIN(psi_cm);    // *input*, Current model rotor flux (V s), acim_flux0.psi
HAL_PIN(slip_in);   // *input*, Current model slip (rad/s electrical), acim_flux0.slip

HAL_PIN(psi);       // *output*, Measured rotor flux magnitude (V s)
HAL_PIN(pos);       // *output*, Measured rotor flux angle (rad electrical)
HAL_PIN(pos_err);   // *output*, pos minus the feedback frame (rad)
HAL_PIN(k);         // *output*, True / modelled slip, the trim's integral (1 at start, 0.5 to 2)
HAL_PIN(slip);      // *output*, Trimmed slip (rad/s electrical), to angle0.slip
HAL_PIN(trim);      // *output*, 1 = trim gates open
HAL_PIN(acc);       // *output*, vel_ref derivative, 5 Hz low pass (rad/s^2 electrical)

struct flux_obs_ctx_t {
  float psa, psb;  // stator flux, stationary frame
  float ia, ib;    // its PI integral
  float w;         // last vel_ref
  float acc;       // its derivative, low passed
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct flux_obs_pin_ctx_t *pins = (struct flux_obs_pin_ctx_t *)pin_ptr;

  PIN(r)        = 0.2;
  PIN(l)        = 0.001;
  PIN(wc)       = 20.0;
  PIN(vel_min)  = 150.0;
  PIN(psi_min)  = 0.5;
  PIN(slip_min) = 2.0;
  PIN(u_adv)    = 0.0;
  PIN(acc_max)  = 0.0;
}

static void rt_start(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct flux_obs_ctx_t *ctx      = (struct flux_obs_ctx_t *)ctx_ptr;
  struct flux_obs_pin_ctx_t *pins = (struct flux_obs_pin_ctx_t *)pin_ptr;

  ctx->psa = ctx->psb = 0.0;
  ctx->ia = ctx->ib = 0.0;
  ctx->w   = PIN(vel_ref);
  ctx->acc = 0.0;
  PIN(k)            = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct flux_obs_ctx_t *ctx      = (struct flux_obs_ctx_t *)ctx_ptr;
  struct flux_obs_pin_ctx_t *pins = (struct flux_obs_pin_ctx_t *)pin_ptr;

  ctx->acc += ((PIN(vel_ref) - ctx->w) / period - ctx->acc) * CLAMP(2.0 * M_PI * 5.0 * period, 0.0, 1.0);
  ctx->w = PIN(vel_ref);

  float frame = mod(PIN(pos_ref) + PIN(vel_ref) * PIN(adv));
  float sn, cs;
  sincos_fast(frame, &sn, &cs);
  float ia = PIN(id) * cs - PIN(iq) * sn;
  float ib = PIN(id) * sn + PIN(iq) * cs;
  // ud/uq are the mean over the rt period before the sample, which the
  // frame turned through: half a period behind the current's frame
  float su, cu;
  float u_adv = PIN(u_adv) != 0.0 ? PIN(u_adv) : -0.5 * period;  // -100 us at 5 kHz
  sincos_fast(mod(frame + PIN(vel_ref) * u_adv), &su, &cu);
  float ua = PIN(ud) * cu - PIN(uq) * su;
  float ub = PIN(ud) * su + PIN(uq) * cu;

  float l   = MAX(PIN(l), 0.0);
  float r   = MAX(PIN(r) + PIN(r_w) * ABS(PIN(vel_ref)), 0.0);
  float pcm = PIN(psi_cm);
  // current model stator flux
  float ca = pcm * cs + l * ia;
  float cb = pcm * sn + l * ib;

  if(PIN(en) > 0.0) {
    float wc = MAX(PIN(wc), 0.0);
    float ea = ca - ctx->psa;
    float eb = cb - ctx->psb;
    ctx->ia += ea * period;
    ctx->ib += eb * period;
    ctx->psa += (ua - r * ia + 1.4 * wc * ea + wc * wc * ctx->ia) * period;
    ctx->psb += (ub - r * ib + 1.4 * wc * eb + wc * wc * ctx->ib) * period;
  } else {
    ctx->psa = ca;
    ctx->psb = cb;
    ctx->ia = ctx->ib = 0.0;
  }

  float pra = ctx->psa - l * ia;
  float prb = ctx->psb - l * ib;
  float psi = sqrtf(pra * pra + prb * prb);
  float pos = atan2f(prb, pra);
  float err = psi > 1e-4 ? minus(pos, frame) : 0.0;

  float slip_in = PIN(slip_in);
  float k       = PIN(k);
  int gate      = PIN(en) > 0.0 && ABS(PIN(vel_ref)) > PIN(vel_min) && ABS(pcm) > 1e-3 &&
             psi > PIN(psi_min) * ABS(pcm);
  float slip = slip_in * k;
  if(gate) {
    // the pull is bounded by the slip it corrects: a wrong angle (voltage
    // limit, sensor fault, transient) can not drive the frame off by more
    // than half the model's slip plus slip_min (it was open: +-63 rad/s at
    // kp_s 20)
    slip += LIMIT(PIN(kp_s) * err, 0.5 * ABS(slip_in) + MAX(PIN(slip_min), 0.0));
    if(PIN(ki_s) > 0.0 && ABS(slip_in) > PIN(slip_min) &&
       (PIN(acc_max) <= 0.0 || ABS(ctx->acc) < PIN(acc_max))) {
      // flux ahead of the frame (err > 0): the true slip is larger
      k += PIN(ki_s) * err * (slip_in > 0.0 ? 1.0 : -1.0) * period;
      k = CLAMP(k, 0.5, 2.0);
    }
  }

  if(PIN(slip_max) > 0.0) {
    slip = LIMIT(slip, PIN(slip_max));  // as acim_flux clamps its own
  }

  PIN(psi)     = psi;
  PIN(pos)     = pos;
  PIN(pos_err) = err;
  PIN(k)       = k;
  PIN(slip)    = slip;
  PIN(trim)    = gate ? 1.0 : 0.0;
  PIN(acc)     = ctx->acc;
}

hal_comp_t flux_obs_comp_struct = {
    .name      = "flux_obs",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = rt_start,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct flux_obs_ctx_t),
    .pin_count = sizeof(struct flux_obs_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
