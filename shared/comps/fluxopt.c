#include "fluxopt_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

// Light-load flux reduction for an induction motor (acim_foc).
// Copper loss for a torque is lowest with id = iq, i.e. id = sqrt(|T| / k),
// k = 1.5 * polecount * lmr. scale = id / id_n, at least min, at most the
// field weakening scale (scale_in, acim_fw0.scale). Rising flux follows the
// torque at once (the rotor flux lags by tr anyway), falling flux is slewed
// down at fall_hz so a pause between cuts does not demagnetize the motor.
//   fluxopt0.torque = pid0.torque_cmd
//   fluxopt0.scale_in = acim_fw0.scale
//   acim_foc0.scale = fluxopt0.scale
// min 1 = off (scale = scale_in).

HAL_COMP(fluxopt);

HAL_PIN(torque);
HAL_PIN(scale_in);
HAL_PIN(scale);
HAL_PIN(min);
HAL_PIN(id_n);
HAL_PIN(lmr);
HAL_PIN(polecount);
HAL_PIN(fall_hz);

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
