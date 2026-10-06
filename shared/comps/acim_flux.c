#include "acim_flux_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"

/**
* ## Brief
* The `acim_flux` component models the rotor flux of an induction motor in the
* rotor flux frame and gives the slip that keeps that frame oriented.
*
* ## Model
* With i_mr the magnetizing current behind the rotor flux (psi_r = Lm * i_mr):
*
*     tr * d(i_mr)/dt = id - i_mr
*     slip            = iq / (tr * i_mr)             [rad/s electrical]
*     torque          = 3/2 * pp * lmr * i_mr * iq    lmr = Lm^2 / Lr
*
* `tr` (Lr/Rr) and `lmr` come from idacim, at rated flux (i_mr = `i_n`).
* Below rated flux (field weakening) the iron desaturates and both grow:
*
*     x       = (i_n - |i_mr|) / (i_n - i_knee), clamped 0..1
*     tr_act  = tr_est * (1 + tr_sat * x)
*     lmr_act = lmr * (1 + lmr_sat * x)
*
* With `i_knee` > 0 both stop growing below i_knee and stay at their
* plateau (1 + sat); i_knee 0 is a straight line down to zero flux.
*
* At low induction the iron's permeability falls again (the foot of the B-H
* curve): on the spindle the secant lmr was flat from 9 to 19 A and 11% lower
* at 6.7 A (idacim sweep, 6 Oct). With `i_dip` > 0 both fall below it:
*
*     y       = (i_dip - |i_mr|) / i_dip, clamped 0..1
*     tr_act  = ... * (1 - lmr_dip * y)
*     lmr_act = ... * (1 - lmr_dip * y)
*
* tr is Lr / Rr and Lr moves with Lm, so the same factor applies to both.
* i_dip 0 (default) or lmr_dip 0 leaves it out.
*
* The model, slip, psi and torque run on tr_act and lmr_act. tr changes about
* twice as much as the secant lmr, so the two gains are separate; 0 (default)
* or i_n 0 keeps both constant. `id`/`iq` are normally the measured
* currents, so a current limit in curpid scales both and the slip stays right.
*
* `psi` = lmr * i_mr is the rotor flux seen from the stator. Sent as hv0.psi,
* with hv0.l = hv0.lq = sigma*Ls (leakage), it turns curpid's PMSM decoupling
* psi_d = l * id + psi into the induction motor's stator flux, so the F3 needs
* no ACIM code.
*
* `en` = 0 clears the flux, so every enable starts from a demagnetized rotor.
*
* ## tr adaptation (reactive power MRAS), off by default
* tr changes with rotor temperature. With `tr_ki` > 0 the model runs on
* `tr_est`, which integrates the reactive power residual
*
*     Q   = uq id - ud iq
*     Q_m = w (id (l id + psi) + l iq^2) - r_w |w| id iq
*     tr_est -= tr_ki * (Q - Q_m) * sign(w) * period
*
* Q does not depend on r, so neither does the estimate. A positive residual
* (times sign(w)) means tr_est is too high; the plant gain is a few thousand VA per second of
* tr, so tr_ki 0.0001 settles in a few seconds. It adapts only while
* |iq| > `tr_iq_min` and |w| > `tr_vel_min` (slip must be large enough to
* see) and, with `tr_flux_min` > 0, |i_mr| >= tr_flux_min * i_n (in deep
* field weakening the residual is dominated by model errors, not tr; 0.95
* keeps it to the base speed range), holds otherwise, and
* stays within 0.5 to 2 times `tr`. With tr_ki 0, tr_est = tr.
*
* ## tr from an encoder in sensorless mode, off by default
* In sl_acim the frame comes from the voltage observer and the MRAS sees no
* tr. With an encoder, sl_seq0.slip_err = obs0.vel_m - encoder speed is the
* true slip less the model's (mechanical rad/s). With `tr_ks` > 0
*
*     tr_est -= tr_ks * tr_est * slip_err / slip_model * period
*
* so tr_ks is a rate [1/s] on the relative slip error (clamped to +-50 %),
* under the same iq and speed gates as the MRAS. Both adaptations also need
* a steady speed when `tr_acc_max` > 0: |acceleration| (5 Hz low pass)
* below tr_acc_max * max_acc (conf0.max_acc, about 0.3); on a ramp the slip
* error and the reactive residual carry the inertia and observer lag, not tr.
*
* ## lmr trim from the q voltage, off by default
* In steady state the q voltage is uq = r iq + w (l id + lmr i_mr). With
* `lmr_ki` > 0 the relative residual of that, (uq - r iq - w (l id + lmr
* i_mr)) / (w lmr i_mr), trims lmr at lmr_ki [1/s], while |w| > tr_vel_min,
* |i_mr| > 20 % of i_n (or 1 A) and the flux is settled (|id - i_mr| < 5 %
* of i_mr). It needs `r` (conf0.r) and stays within 0.7 to 1.4 times `lmr`.
* `lmr_est` is the trimmed rated value; saturation still applies on top.
*/

HAL_COMP(acim_flux);

HAL_PIN(tr);         // *parameter*, rotor time constant Lr/Rr [s]
HAL_PIN(lmr);        // *parameter*, rotor side magnetizing inductance Lm^2/Lr [H]
HAL_PIN(polecount);  // *parameter*, pole pairs
HAL_PIN(i_min);      // *parameter*, i_mr floor for the slip division [A]
HAL_PIN(slip_max);   // *parameter*, slip clamp [rad/s electrical]

HAL_PIN(en);         // *input*, 0 = rotor demagnetized
HAL_PIN(id);         // *input*, d current [A]
HAL_PIN(iq);         // *input*, q current [A]

HAL_PIN(i_mr);       // *output*, magnetizing current [A]
HAL_PIN(psi);        // *output*, lmr * i_mr [V s], to hv0.psi
HAL_PIN(slip);       // *output*, slip [rad/s electrical]
HAL_PIN(torque);     // *output*, estimated torque [Nm]

HAL_PIN(tr_ki);      // *parameter*, tr adaptation gain [1/(VA s)], 0 = off
HAL_PIN(tr_iq_min);  // *parameter*, adapt only above this |iq| [A]
HAL_PIN(tr_vel_min); // *parameter*, adapt only above this |vel| [rad/s electrical]
HAL_PIN(l);          // *parameter*, leakage sigma*Ls [H], conf0.l
HAL_PIN(r_w);        // *parameter*, speed dependent d loss [ohm per rad/s electrical], as obs0.r_w
HAL_PIN(ud);         // *input*, hv0.ud_fb
HAL_PIN(uq);         // *input*, hv0.uq_fb
HAL_PIN(vel);        // *input*, synchronous speed [rad/s electrical], angle0.vel
HAL_PIN(tr_est);     // *output*, tr at rated flux, tr or adapted
HAL_PIN(i_n);        // *parameter*, rated magnetizing current [A], acim_foc0.id_n, 0 = no saturation
HAL_PIN(tr_sat);     // *parameter*, tr growth at zero flux, tr * (1 + tr_sat * x)
HAL_PIN(lmr_sat);    // *parameter*, lmr growth at zero flux, lmr * (1 + lmr_sat * x)
HAL_PIN(tr_act);     // *output*, the tr the model runs on
HAL_PIN(lmr_act);    // *output*, the lmr the model runs on, to acim_foc0.lmr
HAL_PIN(q_res);      // *output*, reactive power residual Q - Q_m [VA]
HAL_PIN(i_knee);     // *parameter*, tr and lmr flat below this i_mr [A], 0 = no knee
HAL_PIN(tr_flux_min); // *parameter*, adapt only while |i_mr| >= this fraction of i_n, 0 = no limit
HAL_PIN(tr_acc_max); // *parameter*, adapt only below this fraction of max_acc, 0 = no limit
HAL_PIN(max_acc);    // *parameter*, conf0.max_acc [rad/s^2 mech]
HAL_PIN(acc);        // *output*, synchronous acceleration, 5 Hz low pass [rad/s^2 mech]
HAL_PIN(tr_ks);      // *parameter*, tr rate from the encoder slip error [1/s], 0 = off
HAL_PIN(slip_err);   // *input*, true minus model slip [rad/s mech], sl_seq0.slip_err
HAL_PIN(lmr_ki);     // *parameter*, lmr trim rate from the q voltage [1/s], 0 = off
HAL_PIN(r);          // *parameter*, stator resistance [ohm], conf0.r, for the lmr trim
HAL_PIN(lmr_est);    // *output*, lmr at rated flux, lmr or trimmed
HAL_PIN(u_res);      // *output*, q voltage residual [V]
HAL_PIN(i_dip);      // *parameter*, lmr and tr fall below this i_mr [A], 0 = no dip; keep below i_knee
HAL_PIN(lmr_dip);    // *parameter*, fraction lmr and tr lose at zero flux, (1 - lmr_dip * y)

struct acim_flux_ctx_t {
  float w;    // last synchronous speed
  float acc;  // its derivative, low passed
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct acim_flux_pin_ctx_t *pins = (struct acim_flux_pin_ctx_t *)pin_ptr;

  PIN(tr)        = 0.06;
  PIN(lmr)       = 0.0165;
  PIN(polecount) = 2.0;
  PIN(i_min)     = 0.5;
  PIN(slip_max)  = 100.0;
  PIN(tr_iq_min) = 3.0;
  PIN(tr_vel_min) = 100.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct acim_flux_ctx_t *ctx      = (struct acim_flux_ctx_t *)ctx_ptr;
  struct acim_flux_pin_ctx_t *pins = (struct acim_flux_pin_ctx_t *)pin_ptr;

  float tr_n = MAX(PIN(tr), 0.001);
  float i_mr = PIN(i_mr);

  // desaturation below rated flux
  float i_k = CLAMP(PIN(i_knee), 0.0, 0.9 * PIN(i_n));
  float x   = PIN(i_n) > 0.0 ? CLAMP((PIN(i_n) - ABS(i_mr)) / (PIN(i_n) - i_k), 0.0, 1.0) : 0.0;
  float id  = PIN(id);
  float iq  = PIN(iq);
  float w   = PIN(vel);
  float l   = MAX(PIN(l), 0.0);
  float pp  = MAX(PIN(polecount), 1.0);
  ctx->acc += ((w - ctx->w) / period / pp - ctx->acc) * CLAMP(2.0 * M_PI * 5.0 * period, 0.0, 1.0);
  ctx->w   = w;
  PIN(acc) = ctx->acc;
  // low induction dip, on lmr and tr alike
  float y   = PIN(i_dip) > 0.0 ? CLAMP((PIN(i_dip) - ABS(i_mr)) / PIN(i_dip), 0.0, 1.0) : 0.0;
  float k_d = MAX(1.0 - PIN(lmr_dip) * y, 0.1);
  float k_l = MAX(1.0 + PIN(lmr_sat) * x, 0.1) * k_d;

  float lmr_n = MAX(PIN(lmr), 0.0);
  float lmr_r = lmr_n;  // rated lmr, trimmed
  PIN(u_res)  = 0.0;
  if(PIN(lmr_ki) > 0.0 && lmr_n > 0.0) {
    lmr_r       = PIN(lmr_est) > 0.0 ? PIN(lmr_est) : lmr_n;
    float psi_w = w * lmr_r * k_l * i_mr;
    PIN(u_res)  = PIN(uq) - PIN(r) * iq - w * l * id - psi_w;
    float i_on  = MAX(0.2 * PIN(i_n), 1.0);
    if(PIN(en) > 0.0 && ABS(w) > PIN(tr_vel_min) && ABS(i_mr) > i_on &&
       ABS(id - i_mr) < 0.05 * ABS(i_mr) && ABS(psi_w) > 1.0) {
      lmr_r += PIN(lmr_ki) * CLAMP(PIN(u_res) / psi_w, -0.3, 0.3) * lmr_r * period;
    }
    lmr_r = CLAMP(lmr_r, 0.7 * lmr_n, 1.4 * lmr_n);
  }
  PIN(lmr_est) = lmr_r;
  float lmr  = lmr_r * k_l;
  float k_tr = MAX(1.0 + PIN(tr_sat) * x, 0.1) * k_d;

  float tr = tr_n;
  if(PIN(tr_ki) > 0.0 || PIN(tr_ks) > 0.0) {
    tr = PIN(tr_est) > 0.0 ? PIN(tr_est) : tr_n;

    int gate = PIN(en) > 0.0 && ABS(iq) > PIN(tr_iq_min) && ABS(w) > PIN(tr_vel_min) &&
               (PIN(tr_flux_min) <= 0.0 || ABS(i_mr) >= PIN(tr_flux_min) * PIN(i_n)) &&
               (PIN(tr_acc_max) <= 0.0 || ABS(ctx->acc) < PIN(tr_acc_max) * MAX(PIN(max_acc), 1.0));
    if(PIN(tr_ki) > 0.0) {
      float q   = PIN(uq) * id - PIN(ud) * iq;
      float q_m = w * (id * (l * id + lmr * i_mr) + l * iq * iq) - PIN(r_w) * ABS(w) * id * iq;
      PIN(q_res) = q - q_m;
      if(gate) {
        // Q and Q_m both follow the direction of rotation; the detuning term
        // goes with iq^2, so only the sign of w matters
        // tr_est is the rated flux value, so the step is scaled back from tr_act
        tr -= PIN(tr_ki) * PIN(q_res) * (w > 0.0 ? 1.0 : -1.0) * period / k_tr;
      }
    } else {
      PIN(q_res) = 0.0;
    }
    // encoder slip error: more true slip than modelled means tr is too high
    float s_m = PIN(slip) / pp;  // model slip [rad/s mech], last tick
    if(PIN(tr_ks) > 0.0 && gate && ABS(s_m) > 0.1) {
      tr -= PIN(tr_ks) * CLAMP(PIN(slip_err) / s_m, -0.5, 0.5) * tr * period;
    }
    tr = CLAMP(tr, 0.5 * tr_n, 2.0 * tr_n);
  } else {
    PIN(q_res) = 0.0;
  }
  PIN(tr_est) = tr;
  tr *= k_tr;

  if(PIN(en) > 0.0) {
    // backward Euler step of the first order lag, stable for any period
    i_mr += (PIN(id) - i_mr) * period / (tr + period);
  } else {
    i_mr = 0.0;
  }

  // the sign of i_mr follows id; divide by its magnitude with a floor, so a
  // demagnetized rotor asks for a bounded slip instead of an infinite one
  float i_div = MAX(ABS(i_mr), MAX(PIN(i_min), 0.01));
  float slip  = PIN(iq) / (tr * i_div);
  if(i_mr < 0.0) {
    slip = -slip;
  }
  if(PIN(slip_max) > 0.0) {
    slip = LIMIT(slip, PIN(slip_max));
  }

  PIN(i_mr)   = i_mr;
  PIN(tr_act)  = tr;
  PIN(lmr_act) = lmr;
  PIN(psi)    = lmr * i_mr;
  PIN(slip)   = slip;
  PIN(torque) = 1.5 * MAX(PIN(polecount), 1.0) * lmr * i_mr * PIN(iq);
}

hal_comp_t acim_flux_comp_struct = {
    .name      = "acim_flux",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct acim_flux_ctx_t),
    .pin_count = sizeof(struct acim_flux_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
