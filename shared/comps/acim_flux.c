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
* `tr` (Lr/Rr) and `lmr` come from idacim. `id`/`iq` are normally the measured
* currents, so a current limit in curpid scales both and the slip stays right.
*
* `psi` = lmr * i_mr is the rotor flux seen from the stator. Sent as hv0.psi,
* with hv0.l = hv0.lq = sigma*Ls (leakage), it turns curpid's PMSM decoupling
* psi_d = l * id + psi into the induction motor's stator flux, so the F3 needs
* no ACIM code.
*
* `en` = 0 clears the flux, so every enable starts from a demagnetized rotor.
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

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct acim_flux_pin_ctx_t *pins = (struct acim_flux_pin_ctx_t *)pin_ptr;

  PIN(tr)        = 0.06;
  PIN(lmr)       = 0.0165;
  PIN(polecount) = 2.0;
  PIN(i_min)     = 0.5;
  PIN(slip_max)  = 100.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct acim_flux_pin_ctx_t *pins = (struct acim_flux_pin_ctx_t *)pin_ptr;

  float tr   = MAX(PIN(tr), 0.001);
  float lmr  = MAX(PIN(lmr), 0.0);
  float i_mr = PIN(i_mr);

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
    .ctx_size  = 0,
    .pin_count = sizeof(struct acim_flux_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
