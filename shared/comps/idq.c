#include "idq_comp.h"
#include "commands.h"
#include "common.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `idq` is the inverse of `dq`: it turns d/q values (normally the voltages from the current controller) at the rotor angle `pos` into the alpha/beta frame (inverse Park) and then into three phase values (inverse Clarke). It runs on the F3 (HV board) as `idq0`, loaded by `stm32f303/src/main.c` (rt_prio 4): `idq0.d/q = curpid0.ud/uq`, `idq0.pos = ls0.pos_v` (the voltage angle), `idq0.mode = ls0.phase_mode`, `ext_sc` = 0, `u`/`v`/`w` go to `hv0.u/v/w` and `si_out`/`co_out` to `hv0.si/co`. The old `conf/template/linkv3.txt` also loads it on the F4.
*
* ## Component Explanation
*
* 1. **Inverse Park transform** (rt):
* - The electrical angle is `pos * polecount` (`polecount` is truncated to an integer). Its sine and cosine are computed with `sincos_fast` (which takes any angle, no wrap needed).
* - With `ext_sc` > 0 the sine and cosine are taken from the `si`/`co` pins instead; `pos` and `polecount` are then ignored, and the caller must make sure `si`/`co` belong to the right angle. The F3 used this to share `dq0`'s sin/cos, but now sets `ext_sc` = 0: `idq0` runs at `ls0.pos_v`, the angle at which the voltage is applied on average (1.5 PWM periods after the current sample), while `dq0` keeps the sample angle `ls0.pos`.
* - The sine and cosine used this tick are output on `si_out`/`co_out`, so `hv0` can build its dead time reference currents on the same angle.
* ```c
* a = d * cos - q * sin;
* b = d * sin + q * cos;
* ```
*
* 2. **Inverse Clarke transform** (rt), selected by `mode`:
* - 0 (90 deg 3 phase): `u = a`, `v = 0`, `w = b`.
* - 2 (120 deg 3 phase): `u = a`, `v = -a/2 + b * sqrt(3)/2`, `w = -a/2 - b * sqrt(3)/2`. The phase amplitude equals the vector length.
* - 3 (180 deg 2 phase): `u = b/2`, `v = 0`, `w = -b/2`.
* - 4 (180 deg 3 phase): `u = b/2`, `v = a`, `w = -b/2`.
* - Any other mode (including 1, 90 deg 4 phase) gives 0.
* - The outputs are centred on 0. On the F3, `hv0` adds the dead time compensation and the space vector offset that puts them into 0..udc (formerly `svm0` did the offset).
*/

HAL_COMP(idq);

HAL_PIN(mode);  // *input*, Phase mode, 0 = 90 deg 3ph, 2 = 120 deg 3ph, 3 = 180 deg 2ph, 4 = 180 deg 3ph, others give 0

//d,q inputs
HAL_PIN(d);  // *input*, D-axis value, e.g. voltage (V)
HAL_PIN(q);  // *input*, Q-axis value, e.g. voltage (V)

//rotor position
HAL_PIN(pos);        // *input*, Rotor angle (rad), ls0.pos_v on the F3
HAL_PIN(polecount);  // *parameter*, Pole pairs, pos is multiplied by it, min 1, default 0 (used as 1)

// sin/cos of the same angle from elsewhere, used instead of sincos_fast when
// ext_sc > 0 (no longer on the f3: idq0 runs at ls0.pos_v, dq0 at ls0.pos)
HAL_PIN(si);      // *input*, Sine of the electrical angle, used when ext_sc > 0, not wired on the F3
HAL_PIN(co);      // *input*, Cosine of the electrical angle, used when ext_sc > 0, not wired on the F3
HAL_PIN(ext_sc);  // *parameter*, > 0 = use si/co instead of computing them from pos, default 0 (set to 0 on the F3)
// the sin/cos this tick used, for hv0's dead-time reference currents
HAL_PIN(si_out);  // *output*, Sine used this tick, to hv0.si on the F3
HAL_PIN(co_out);  // *output*, Cosine used this tick, to hv0.co on the F3

//a,b output
HAL_PIN(a);  // *output*, Alpha component
HAL_PIN(b);  // *output*, Beta component

//U V W output
HAL_PIN(u);  // *output*, U phase value
HAL_PIN(v);  // *output*, V phase value
HAL_PIN(w);  // *output*, W phase value

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct idq_ctx_t * ctx = (struct idq_ctx_t *)ctx_ptr;
  struct idq_pin_ctx_t *pins = (struct idq_pin_ctx_t *)pin_ptr;

  float d = PIN(d);
  float q = PIN(q);

  float p   = (int)MAX(PIN(polecount), 1.0);
  float pos = PIN(pos) * p;

  float si = PIN(si);
  float co = PIN(co);
  if(PIN(ext_sc) <= 0.0) {
    sincos_fast(pos, &si, &co);
  }
  PIN(si_out) = si;
  PIN(co_out) = co;

  //inverse park transformation
  float a = d * co - q * si;
  float b = d * si + q * co;

  //inverse clarke transformation
  float u, v, w;

  switch((int)PIN(mode)) {
    case PHASE_90_3PH:  // 90°
      u = a;
      v = 0.0;
      w = b;
      break;

    case PHASE_120_3PH:  // 120°
      u = a;
      v = -a / 2.0 + b / 2.0 * M_SQRT3;
      w = -a / 2.0 - b / 2.0 * M_SQRT3;
      break;

    case PHASE_180_2PH:  // 180°
      u = b / 2.0;
      v = 0.0;
      w = -b / 2.0;
      break;

    case PHASE_180_3PH:  // 180°
      u = b / 2.0;
      v = a;
      w = -b / 2.0;
      break;

    default:
      u = 0.0;
      v = 0.0;
      w = 0.0;
  }


  PIN(a) = a;
  PIN(b) = b;

  PIN(u) = u;
  PIN(v) = v;
  PIN(w) = w;
}

hal_comp_t idq_comp_struct = {
    .name      = "idq",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct idq_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
