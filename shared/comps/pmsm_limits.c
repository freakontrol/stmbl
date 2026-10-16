#include "pmsm_limits_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `pmsm_limits` estimates the current and torque a PMSM can reach with the available AC voltage: absolute limits, limits at the present operating point, and the limits reachable within the next control period. It is compiled into the F4 firmware; the outputs were meant to feed the torque limits of `pid` and the current controller.
*
* ## Component Explanation
* All work is done in `rt`. Defaults (`nrt_init`): `ld = 0.001`. The inputs are clamped to `lq >= 0.0001`, `psi >= 0.01`, `r >= 0.001`, and `polecount` is truncated to an integer >= 1. With `volt = ac_volt`:
*
* 1. **Absolute limits** (standstill, no back EMF):
* - `abs_max_torque = 3/2 * polecount * psi * abs_max_cur`.
*
* 2. **Limits at the present velocity**:
* - `max_cur = (volt - indq) / r`, `min_cur = (-volt - indq) / r`, and `max_torque` / `min_torque` = `3/2 * polecount * psi * cur`.
* - `indq` is the induced voltage and must be supplied from outside (the TODO in the code says `vel * (psi + id * ld)`).
*
* 3. **Limits for the next period**:
* ```c
* next_max_cur = iq + (volt - r * iq - indq) / lq * period * 2 / 3;
* next_min_cur = iq + (-volt - r * iq - indq) / lq * period * 2 / 3;
* ```
* - Only `iq` is used; `id` and `ld` are ignored (TODO in the code).
*
* {{% hint warning %}}
* Not loaded by any template or config in `conf/`. The factor 2/3 in the `next_*` values is not explained in the code; `id`, `ld` and the induced voltage are not handled internally (TODOs).
* {{% /hint %}}
*/

HAL_COMP(pmsm_limits);

// motor values
HAL_PIN(psi);        // *parameter*, Flux linkage (Vs, default 0.01)
HAL_PIN(r);          // *parameter*, Phase resistance (Ohm, default 1)
HAL_PIN(ld);         // *parameter*, d-axis inductance (H), not used
HAL_PIN(lq);         // *parameter*, q-axis inductance (H, default 0.001)
HAL_PIN(polecount);  // *parameter*, Pole pairs (default 1)

// sys limit
HAL_PIN(ac_volt);  // *input*, Available AC voltage (V)

// next min max out -> pid, curpid
HAL_PIN(next_max_cur);     // *output*, Max. current reachable in the next period (A)
HAL_PIN(next_max_torque);  // *output*, Max. torque reachable in the next period (Nm)
HAL_PIN(next_min_cur);     // *output*, Min. current reachable in the next period (A)
HAL_PIN(next_min_torque);  // *output*, Min. torque reachable in the next period (Nm)

// min max out @ current vel
HAL_PIN(max_cur);     // *output*, Max. steady state current at the present velocity (A)
HAL_PIN(max_torque);  // *output*, Max. steady state torque at the present velocity (Nm)
HAL_PIN(min_cur);     // *output*, Min. steady state current at the present velocity (A)
HAL_PIN(min_torque);  // *output*, Min. steady state torque at the present velocity (Nm)

// abs max out
HAL_PIN(abs_max_cur);     // *output*, Current at standstill, ac_volt / r (A)
HAL_PIN(abs_max_torque);  // *output*, Torque at abs_max_cur (Nm)
HAL_PIN(abs_max_vel);     // *output*, No load velocity, ac_volt / psi / polecount (rad/s)

// pmsm feedback
HAL_PIN(iq);    // *input*, Measured q-axis current (A)
HAL_PIN(indq);  // *input*, Induced voltage on the q-axis (V)

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct pmsm_limits_ctx_t * ctx = (struct pmsm_limits_ctx_t *)ctx_ptr;
  struct pmsm_limits_pin_ctx_t *pins = (struct pmsm_limits_pin_ctx_t *)pin_ptr;

  PIN(psi)       = 0.01;
  PIN(r)         = 1.0;
  PIN(ld)        = 0.001;
  PIN(lq)        = 0.001;
  PIN(polecount) = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct pmsm_limits_ctx_t * ctx = (struct pmsm_limits_ctx_t *)ctx_ptr;
  struct pmsm_limits_pin_ctx_t *pins = (struct pmsm_limits_pin_ctx_t *)pin_ptr;

  float p = (int)MAX(PIN(polecount), 1.0);
  //float ld = MAX(PIN(ld), 0.0001);
  float lq  = MAX(PIN(lq), 0.0001);
  float psi = MAX(PIN(psi), 0.01);
  float r   = MAX(PIN(r), 0.001);
  float iq  = PIN(iq);  // TODO id

  float indq = PIN(indq);  // TODO vel * (psi + id ld)

  float volt           = PIN(ac_volt);
  float abs_max_cur    = volt / r;
  float abs_max_vel    = volt / psi / p;
  float abs_max_torque = 3.0 / 2.0 * p * psi * abs_max_cur;

  float next_max_cur    = iq + (volt - r * iq - indq) / lq * period * 2.0 / 3.0;
  float next_min_cur    = iq + (-volt - r * iq - indq) / lq * period * 2.0 / 3.0;
  float next_max_torque = 3.0 / 2.0 * p * (psi * next_max_cur);
  float next_min_torque = 3.0 / 2.0 * p * (psi * next_min_cur);

  float max_cur    = (volt - indq) / r;
  float min_cur    = (-volt - indq) / r;
  float max_torque = 3.0 / 2.0 * p * (psi * max_cur);
  float min_torque = 3.0 / 2.0 * p * (psi * min_cur);


  PIN(abs_max_cur)     = abs_max_cur;
  PIN(abs_max_vel)     = abs_max_vel;
  PIN(abs_max_torque)  = abs_max_torque;
  PIN(next_max_cur)    = next_max_cur;
  PIN(next_min_cur)    = next_min_cur;
  PIN(next_max_torque) = next_max_torque;
  PIN(next_min_torque) = next_min_torque;
  PIN(max_cur)         = max_cur;
  PIN(min_cur)         = min_cur;
  PIN(max_torque)      = max_torque;
  PIN(min_torque)      = min_torque;
}

hal_comp_t pmsm_limits_comp_struct = {
    .name      = "pmsm_limits",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct pmsm_limits_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
