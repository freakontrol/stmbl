#include "acim_foc_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"

/**
* ## Brief
* The `acim_foc` component turns a torque command into d and q current
* commands for an induction motor in the rotor flux frame, using the flux
* model of acim_flux.
*
* ## Behaviour
* - d: `id_n * scale` (scale from acim_fw, 1 below the corner), reached at
*   `id_rate` A/s from 0 at every enable, so the bridge never steps the full
*   magnetizing current into one leg.
* - Torque is held at 0 (`ready` = 0, t_max = 0) until i_mr has reached 90% of
*   the d command once; after that `ready` stays 1 until the next disable.
* - q: torque / (3/2 * pp * lmr * i_mr), limited so |i| stays within max_cur
*   with d taking priority.
* - Flux boost (optional, `k_boost` > 0): while the flux is below the
*   ramped d command, d gets k_boost * (d - i_mr) on top, so i_mr rises in
*   about tr / (1 + k_boost) instead of tr: faster torque after field
*   weakening and a shorter magnetizing at enable. Only upward; going into
*   field weakening is not sped up, so the acim_fw loop keeps its gain. The
*   boost only takes current q does not need: |d| <= sqrt(max_cur^2 - q^2),
*   and with `boost_max` > 0 never more than boost_max * id_n on top of d
*   (0.3 keeps the enable peak near the current limit).
*   k_boost 0 = off.
* - t_max: torque the current limit allows at the present flux, capped by
*   acim_fw's t_lim when that is nonzero. Link pid0.max_torque and
*   pid0.neg_min_torque to it.
*/

HAL_COMP(acim_foc);

HAL_PIN(en);         // *input*, fault0.en_out
HAL_PIN(torque);     // *input*, torque command [Nm]
HAL_PIN(scale);      // *input*, flux fraction from acim_fw, 1 = rated
HAL_PIN(t_lim);      // *input*, torque limit from acim_fw [Nm], 0 = none
HAL_PIN(i_mr);       // *input*, magnetizing current from acim_flux [A]

HAL_PIN(id_n);       // *parameter*, rated magnetizing current [A peak]
HAL_PIN(id_rate);    // *parameter*, d current ramp [A/s]
HAL_PIN(max_cur);    // *parameter*, current limit [A peak]
HAL_PIN(lmr);        // *parameter*, Lm^2/Lr [H]
HAL_PIN(polecount);  // *parameter*, pole pairs
HAL_PIN(k_boost);    // *parameter*, flux boost gain, 0 = off
HAL_PIN(boost_max);  // *parameter*, largest boost as a fraction of id_n, 0 = only the current limit

HAL_PIN(d_cmd);      // *output*, to hv0.d_cmd
HAL_PIN(q_cmd);      // *output*, to hv0.q_cmd
HAL_PIN(t_max);      // *output*, available torque [Nm]
HAL_PIN(t_min);      // *output*, -t_max
HAL_PIN(ready);      // *output*, 1 = magnetized, torque allowed
HAL_PIN(boost);      // *output*, d current added by the flux boost [A]

struct acim_foc_ctx_t {
  float id;  // ramped d command, without the boost
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct acim_foc_pin_ctx_t *pins = (struct acim_foc_pin_ctx_t *)pin_ptr;

  PIN(scale)     = 1.0;
  PIN(id_n)      = 10.0;
  PIN(id_rate)   = 200.0;
  PIN(max_cur)   = 10.0;
  PIN(lmr)       = 0.0165;
  PIN(polecount) = 2.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct acim_foc_ctx_t *ctx      = (struct acim_foc_ctx_t *)ctx_ptr;
  struct acim_foc_pin_ctx_t *pins = (struct acim_foc_pin_ctx_t *)pin_ptr;

  float max_cur = MAX(PIN(max_cur), 0.1);
  float id_tgt  = CLAMP(PIN(id_n) * CLAMP(PIN(scale), 0.0, 1.0), 0.0, max_cur);
  float id      = ctx->id;
  float ready   = PIN(ready);

  if(PIN(en) > 0.0) {
    float step = MAX(PIN(id_rate), 1.0) * period;
    id += LIMIT(id_tgt - id, step);
    if(PIN(i_mr) >= 0.9 * id_tgt && id_tgt > 0.0) {
      ready = 1.0;
    }
  } else {
    id    = 0.0;
    ready = 0.0;
  }

  float iq_max = sqrtf(MAX(max_cur * max_cur - id * id, 0.0));
  float i_k    = MAX(PIN(i_mr), 0.1 * MAX(PIN(id_n), 0.1));
  float k      = 1.5 * MAX(PIN(polecount), 1.0) * MAX(PIN(lmr), 0.0001) * i_k;
  float t_max  = k * iq_max;
  if(PIN(t_lim) > 0.0) {
    t_max = MIN(t_max, PIN(t_lim));
  }

  float iq = 0.0;
  if(ready > 0.0) {
    iq = LIMIT(PIN(torque) / k, iq_max);
  } else {
    t_max = 0.0;
  }

  // flux boost: up to the current q leaves free
  float boost = 0.0;
  if(PIN(en) > 0.0 && PIN(k_boost) > 0.0 && id > PIN(i_mr)) {
    float id_max = sqrtf(MAX(max_cur * max_cur - iq * iq, 0.0));
    boost        = CLAMP(PIN(k_boost) * (id - PIN(i_mr)), 0.0, MAX(id_max - id, 0.0));
    if(PIN(boost_max) > 0.0) {
      boost = MIN(boost, PIN(boost_max) * MAX(PIN(id_n), 0.0));
    }
  }

  ctx->id    = id;
  PIN(boost) = boost;
  PIN(d_cmd) = id + boost;
  PIN(q_cmd) = iq;
  PIN(t_max) = t_max;
  PIN(t_min) = -t_max;
  PIN(ready) = ready;
}

hal_comp_t acim_foc_comp_struct = {
    .name      = "acim_foc",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct acim_foc_ctx_t),
    .pin_count = sizeof(struct acim_foc_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
