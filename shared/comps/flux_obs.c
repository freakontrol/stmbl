#include "flux_obs_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* The `flux_obs` component is a hybrid rotor flux observer for an induction
* motor on an encoder (review item B5). It measures the rotor flux angle and
* magnitude from voltage and current, without tr, and trims the current
* model's slip so that the field orientation follows the measured flux at
* speed. Below its crossover it falls back to the current model (acim_flux),
* so at low speed nothing changes.
*
* **Status (14 Oct):** shadow (pos_err, psi) and the slip trim are
* bench-checked on the spindle, cold and warm, without load; untested under a
* cutting load. conf/template/flux_obs.txt loads it in shadow (link
* flux_obs); its comment gives the trim settings.
*
* ## Model (Jansen / Lorenz, stationary frame)
* With l = sigma*Ls and psi = lmr * i_mr (the rotor flux seen from the
* stator, as acim_flux0.psi):
*
*     voltage model:  d(psi_s)/dt = u - r i
*     current model:  psi_s_cm    = psi_cm e^(j pos_ref) + l i
*     hybrid:         d(psi_s)/dt = u - r i + kp (psi_s_cm - psi_s) + ki int(psi_s_cm - psi_s)
*     rotor flux:     psi_r       = psi_s - l i
*
* kp = 1.4 wc, ki = wc^2: the estimate follows the current model below about
* `wc` [rad/s electrical] and the voltage model above it, where only r and l
* matter and r's share falls with speed. The voltage integrator's drift
* (offsets, r error) is held by the current model term.
*
* ## Frames
* Like obs: the feedback (hv0.id_fb ...) is in the frame pos_ref + vel_ref *
* adv (angle0.pos, hv0.adv). Run flux_obs before angle0 (lower rt_prio), so
* pos_ref is the angle the feedback was taken with. The voltages are the
* mean over the period before the sample, so they sit `u_adv` off the
* current's frame, half an rt period (u_adv 0 = -period / 2, -100 us at
* 5 kHz). Do not calibrate it
* on idle holds: on the spindle (13 Oct) -30 us zeroed pos_err there, but
* that hid the current model's slip error and left 0.17 rad while braking;
* at -100 us the rotor resistance seen by the observer (slip * psi / iq) is
* 0.121..0.128 in run-up, hold and both brakes, at -30 us 0.09..0.15. `pos_err` = flux angle
* minus that frame: how far the current model's orientation is off, 0 below
* the crossover.
*
* ## Slip trim, off by default (shadow)
*
*     slip = slip_in * k + kp_s * pos_err
*
* `kp_s` [1/s] pulls the frame onto the measured flux; `ki_s` [1/(rad s)]
* moves k, the ratio of true to modelled slip (tr / true tr), so pos_err goes
* to 0 in steady state and a load step finds the right slip at once. Both act
* only while |vel_ref| > `vel_min` and the measured flux is above `psi_min`
* times psi_cm; k moves only while |slip_in| > `slip_min`, and stays within
* 0.5 to 2. kp_s 0 and ki_s 0 (default): slip = slip_in, the observer only
* reports. Wire angle0.slip = flux_obs0.slip to use it.
*
* With `acc_max` > 0, k also holds while |acceleration| (vel_ref, 5 Hz low
* pass, `acc`) is above it [rad/s^2 electrical]; 0 (default) = no gate. With
* u_adv right k is the same in run-up, hold and brake (spindle 13 Oct: within
* 0.01 in field weakening), so the gate is only for a doubtful u_adv.
*
* k starts at 1 when the drive starts (rt_start) and otherwise keeps its value
* while the gates are shut or en is 0: it tracks the rotor temperature, which
* changes slowly, so a new enable starts from the last learnt ratio.
*/

HAL_COMP(flux_obs);

HAL_PIN(r);         // *parameter*, stator resistance [ohm], conf0.r
HAL_PIN(r_w);       // *parameter*, d loss growing with speed [ohm per rad/s electrical], as obs0.r_w
HAL_PIN(l);         // *parameter*, leakage sigma*Ls [H], conf0.l
HAL_PIN(wc);        // *parameter*, crossover current to voltage model [rad/s electrical]
HAL_PIN(adv);       // *parameter*, feedback frame lead over pos_ref [s], hv0.adv
HAL_PIN(u_adv);     // *parameter*, voltage frame lead over the current's [s], 0 = -half the rt period
HAL_PIN(kp_s);      // *parameter*, frame pull onto the measured flux [1/s], 0 = off
HAL_PIN(ki_s);      // *parameter*, slip ratio rate [1/(rad s)], 0 = off
HAL_PIN(vel_min);   // *parameter*, trim only above this |vel_ref| [rad/s electrical]
HAL_PIN(psi_min);   // *parameter*, trim only while psi > psi_min * psi_cm
HAL_PIN(slip_min);  // *parameter*, k moves only above this |slip_in| [rad/s electrical]
HAL_PIN(acc_max);   // *parameter*, k moves only below this |acc| [rad/s^2 electrical], 0 = no gate
HAL_PIN(slip_max);  // *input*, clamp on the trimmed slip [rad/s electrical], acim_flux0.slip_max, 0 = none

HAL_PIN(en);        // *input*, 0 = flux estimate follows the current model (k is kept)
HAL_PIN(id);        // *input*, hv0.id_fb
HAL_PIN(iq);        // *input*, hv0.iq_fb
HAL_PIN(ud);        // *input*, hv0.ud_fb
HAL_PIN(uq);        // *input*, hv0.uq_fb
HAL_PIN(pos_ref);   // *input*, angle the feedback frame was at, angle0.pos
HAL_PIN(vel_ref);   // *input*, angle0.vel
HAL_PIN(psi_cm);    // *input*, current model rotor flux, acim_flux0.psi
HAL_PIN(slip_in);   // *input*, current model slip, acim_flux0.slip

HAL_PIN(psi);       // *output*, measured rotor flux magnitude [V s]
HAL_PIN(pos);       // *output*, measured rotor flux angle [rad electrical]
HAL_PIN(pos_err);   // *output*, pos minus the feedback frame [rad]
HAL_PIN(k);         // *output*, true / modelled slip, the trim's integral
HAL_PIN(slip);      // *output*, trimmed slip [rad/s electrical], to angle0.slip
HAL_PIN(trim);      // *output*, 1 = trim gates open
HAL_PIN(acc);       // *output*, vel_ref derivative, 5 Hz low pass [rad/s^2 electrical]

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
