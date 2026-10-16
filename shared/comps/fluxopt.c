#include "fluxopt_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `fluxopt` reduces the flux of an induction motor at light load (`acim_foc`) to cut the copper loss. `conf/template/acim_foc.txt` loads it between the field weakening and the field oriented control with `fluxopt0.torque = pid0.torque_cmd`, `fluxopt0.scale_in = acim_fw0.scale`, `fluxopt0.id_n = acim_foc0.id_n`, `fluxopt0.lmr = acim_flux0.lmr_act`, `fluxopt0.polecount = conf0.polecount` and `acim_foc0.scale = fluxopt0.scale`. It is off there (`min` 1); `fluxopt0.min = 0.5` in a config turns it on.
*
* ## Component Explanation
* All work is done in `rt`.
*
* 1. **Flux for a torque**:
* - With `k = 1.5 * polecount * lmr` the torque is `T = k * id * iq`. The copper loss for a torque is lowest with `id = iq`, i.e. `id = sqrt(abs(T) / k)`.
* - Target scale: `sqrt(abs(torque) / k) / id_n`, clamped to `MAX(min, 0.05)` .. `scale_in`.
* - With `min >= 1`, `k <= 0` or `id_n <= 0` the target is `scale_in` (off).
*
* 2. **Rise and fall**:
* - Rising flux follows the torque at once (the rotor flux lags by tr anyway).
* - Falling flux is low passed at `fall_hz` (0 = at once), so a pause between cuts does not demagnetize the motor.
* - `scale` never exceeds `scale_in`, so field weakening always wins.
*
* 3. **Results** (spindle bench, 12 Oct): idle copper loss 111 W down to 28 W. Start torque 9.8 Nm instead of 13.9 Nm until the flux is back.
*
* {{% hint warning %}}
* Lower flux at idle means less torque on a sudden load until the rotor flux has built up again (tr).
* {{% /hint %}}
*/

HAL_COMP(fluxopt);

HAL_PIN(torque);     // *input*, Torque command (Nm), pid0.torque_cmd
HAL_PIN(scale_in);   // *input*, Field weakening scale, upper limit of scale (default 1), acim_fw0.scale
HAL_PIN(scale);      // *output*, Flux scale, to acim_foc0.scale
HAL_PIN(min);        // *parameter*, Lowest flux scale, 1 = off (default 1, at least 0.05)
HAL_PIN(id_n);       // *input*, Rated magnetizing current (A), acim_foc0.id_n
HAL_PIN(lmr);        // *input*, Magnetizing inductance lmr (H), acim_flux0.lmr_act
HAL_PIN(polecount);  // *input*, Pole pairs, conf0.polecount
HAL_PIN(fall_hz);    // *parameter*, Low pass of falling flux (Hz, default 1, 0 = no filter)

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct fluxopt_pin_ctx_t *pins = (struct fluxopt_pin_ctx_t *)pin_ptr;
  PIN(min)      = 1.0;
  PIN(fall_hz)  = 1.0;
  PIN(scale_in) = 1.0;
  PIN(scale)    = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct fluxopt_pin_ctx_t *pins = (struct fluxopt_pin_ctx_t *)pin_ptr;

  float k   = 1.5 * PIN(polecount) * PIN(lmr);
  float top = PIN(scale_in);
  float s   = top;
  if(k > 0.0 && PIN(id_n) > 0.0 && PIN(min) < 1.0) {
    s = CLAMP(sqrtf(ABS(PIN(torque)) / k) / PIN(id_n), MAX(PIN(min), 0.05), top);
  }
  if(s < PIN(scale)) {
    s = PIN(scale) + (s - PIN(scale)) * LP_HZ(PIN(fall_hz));
  }
  PIN(scale) = MIN(s, top);
}

hal_comp_t fluxopt_comp_struct = {
    .name      = "fluxopt",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct fluxopt_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
