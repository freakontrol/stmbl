#include "psi_comp.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `psi` measures the flux linkage (back EMF constant) of a motor that is turned from outside while the power stage is not driving it. It runs on the F4 board. The working wiring is in `conf/experimental/psi.txt`: `psi0.u/v/w = hv0.u_fb/v_fb/w_fb`, `psi0.dc_volt = hv0.dc_volt`, `psi0.vel = vel1.vel`, `psi0.polecount = conf0.polecount`, with `psi0.psi` and `psi0.max_psi` shown on the scope.
*
* ## Component Explanation
* All work is done in `rt`.
*
* 1. **Procedure**:
* - Set `conf0.polecount`, disable the controller (the template sets `net0.enable = 0` and releases the brake with `io0.brake = 1`) and turn the motor by hand or with another drive as fast as possible.
* - Read the result from `max_psi`.
*
* 2. **Calculation**:
* - Electrical frequency: `f = abs(vel) / (2 pi) * polecount` (Hz).
* - Voltage: the peak to peak spread of the three phase voltages `max(u,v,w) - min(u,v,w)`, limited to `dc_volt`, divided by sqrt(3).
* - Only when `f > 1` Hz: `psi = u / (2 pi f)`.
*
* 3. **Peak hold**:
* - Whenever `f` is higher than the stored maximum frequency, `max_psi` is updated to the present `psi`.
* - The stored maximum frequency decays by a factor 0.999999 per rt cycle.
*
* {{% hint warning %}}
* The voltage calculation has a "TODO: fix" in the code. There is no `psi` template any more; for the flux linkage use `id_pmsm` (or `emf0` on the F3).
* {{% /hint %}}
*/

HAL_COMP(psi);

HAL_PIN(vel);        // *input*, Mechanical velocity (rad/s)
HAL_PIN(dc_volt);    // *input*, DC link voltage (V)
HAL_PIN(u);          // *input*, Phase U voltage (V)
HAL_PIN(v);          // *input*, Phase V voltage (V)
HAL_PIN(w);          // *input*, Phase W voltage (V)
HAL_PIN(polecount);  // *parameter*, Pole pairs
HAL_PIN(psi);        // *output*, Measured flux linkage (Vs), updated above 1 Hz electrical
HAL_PIN(max_psi);    // *output*, Flux linkage measured at the highest speed (Vs)

struct psi_ctx_t {
  float max_f;
};


static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct psi_ctx_t *ctx      = (struct psi_ctx_t *)ctx_ptr;
  struct psi_pin_ctx_t *pins = (struct psi_pin_ctx_t *)pin_ptr;

  float f  = ABS(PIN(vel)) / 2.0 * M_1_PI * PIN(polecount);
  float u  = PIN(dc_volt);
  float u2 = MAX3(PIN(u), PIN(v), PIN(w)) - MIN3(PIN(u), PIN(v), PIN(w));
  u        = MIN(u, u2) * M_SQRT1_3;  // TODO: fix
  if(f > 1.0) {
    PIN(psi) = u / f / 2.0 * M_1_PI;
    if(ctx->max_f < f) {
      ctx->max_f   = f;
      PIN(max_psi) = PIN(psi);
    }
  }
  ctx->max_f *= 0.999999;
}

const hal_comp_t psi_comp_struct = {
    .name      = "psi",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .hw_init   = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct psi_ctx_t),
    .pin_count = sizeof(struct psi_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
