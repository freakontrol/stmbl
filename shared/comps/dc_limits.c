#include "dc_limits_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `dc_limits` is the brushed DC motor version of `pmsm_limits`: it estimates the armature current and torque that can be reached with the available voltage, at standstill, at the present velocity and within the next control period. It is compiled into the F4 firmware.
*
* ## Component Explanation
* All work is done in `rt`. Inputs are clamped to `lq >= 0.0001`, `psi >= 0.01`, `r >= 0.001`. The back EMF is `indq = vel * psi` and `volt = ac_volt`.
*
* 1. **Absolute limits**: `abs_max_cur = volt / r`, `abs_max_vel = volt / psi`, `abs_max_torque = psi * abs_max_cur`.
*
* 2. **Limits at the present velocity**: `max_cur = (volt - vel * psi) / r`, `min_cur = (-volt - vel * psi) / r`, torques = `psi * cur`.
*
* 3. **Limits for the next period**:
* ```c
* next_max_cur = iq + (volt - r * iq - indq) / lq * period * 2 / 3;
* ```
* - and the same with `-volt` for `next_min_cur`; torques = `psi * cur`. `lq` is used as the armature inductance.
*
* {{% hint warning %}}
* Not loaded by any template or config in `conf/`. The factor 2/3 in the `next_*` values is copied from `pmsm_limits` and has no meaning for a DC motor.
* {{% /hint %}}
*/

HAL_COMP(dc_limits);

// motor values
HAL_PIN(psi);  // *parameter*, Torque / back EMF constant (Nm/A, default 0.01)
HAL_PIN(r);    // *parameter*, Armature resistance (Ohm, default 1)
HAL_PIN(ld);   // *parameter*, Not used
HAL_PIN(lq);   // *parameter*, Armature inductance (H, default 0.001)

// sys limit
HAL_PIN(ac_volt);  // *input*, Available voltage (V)

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
HAL_PIN(abs_max_vel);     // *output*, No load velocity, ac_volt / psi (rad/s)

// dc feedback
HAL_PIN(iq);   // *input*, Measured armature current (A)
HAL_PIN(vel);  // *input*, Motor velocity (rad/s)

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct dc_limits_ctx_t * ctx = (struct dc_limits_ctx_t *)ctx_ptr;
  struct dc_limits_pin_ctx_t *pins = (struct dc_limits_pin_ctx_t *)pin_ptr;

  PIN(psi) = 0.01;
  PIN(r)   = 1.0;
  PIN(ld)  = 0.001;
  PIN(lq)  = 0.001;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct dc_limits_ctx_t * ctx = (struct dc_limits_ctx_t *)ctx_ptr;
  struct dc_limits_pin_ctx_t *pins = (struct dc_limits_pin_ctx_t *)pin_ptr;

  float lq  = MAX(PIN(lq), 0.0001);
  float psi = MAX(PIN(psi), 0.01);
  float r   = MAX(PIN(r), 0.001);

  float iq  = PIN(iq);
  float vel = PIN(vel);

  float indq = vel * psi;

  float volt           = PIN(ac_volt);
  float abs_max_cur    = volt / r;
  float abs_max_vel    = volt / psi;
  float abs_max_torque = psi * abs_max_cur;

  float next_max_cur    = iq + (volt - r * iq - indq) / lq * period * 2.0 / 3.0;
  float next_min_cur    = iq + (-volt - r * iq - indq) / lq * period * 2.0 / 3.0;
  float next_max_torque = psi * next_max_cur;
  float next_min_torque = psi * next_min_cur;

  float max_cur    = (volt - vel * psi) / r;
  float min_cur    = (-volt - vel * psi) / r;
  float max_torque = psi * max_cur;
  float min_torque = psi * min_cur;


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

hal_comp_t dc_limits_comp_struct = {
    .name      = "dc_limits",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct dc_limits_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
