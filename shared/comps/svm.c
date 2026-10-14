#include "svm_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `svm` adds a common mode offset to three phase voltages centred on 0, so that they fit into the 0..`udc` range of the half bridges. The choice of offset sets the modulation: sine, space vector (the default), flat bottom or flat top. It is built into both the F3 (HV board) and the F4 firmware, but `stm32f303/src/main.c` no longer loads it: on the F3, `hv0` now takes `idq0`'s phase voltages directly, adds the dead time compensation and then applies the same space vector (midpoint) offset itself, so the offset sees the compensated phases. No template in `conf/` loads `svm` either, so it is only there to be loaded by hand.
*
* ## Component Explanation
*
* 1. **Offset** (rt), selected by `mode`, the outputs are `su = u - offset` and so on:
* - 0, sine: `offset = (u + v + w) / 3 - udc / 2`, the phases swing around `udc / 2`.
* - 1, space vector: `offset = (min + max) / 2 - udc / 2`. The mid point between the highest and lowest phase sits at `udc / 2`, which gives about 15 % more line to line voltage than sine.
* - 2, flat bottom: `offset = min`, the lowest phase is at 0 (its low side stays on).
* - 3, flat top: `offset = max - udc`, the highest phase is at `udc`.
* - Any other value works as 0 (sine).
* - The outputs are not clamped. If the input asks for more than `udc` allows they leave 0..`udc`, and the PWM stage (`hv`) clamps them.
*
* 2. **Commutation mode** (rt, `cmode`):
* - 0 (default): all half bridges enabled, `enu`/`env`/`enw` = 1.
* - 1, block: the enable of the middle phase (the one between the other two, in either order) is set to 0.
*
* {{% hint warning %}}
* The enable outputs are not used anywhere: the F3's `hv` ignores its own enable inputs.
* {{% /hint %}}
*/

HAL_COMP(svm);

//U V W inputs
HAL_PIN(u);  // *input*, U phase voltage, centred on 0 (V)
HAL_PIN(v);  // *input*, V phase voltage, centred on 0 (V)
HAL_PIN(w);  // *input*, W phase voltage, centred on 0 (V)

//dclink input
HAL_PIN(udc);  // *input*, DC link voltage (V)

//U V W outputs
HAL_PIN(su);  // *output*, U phase voltage with offset, 0..udc when within range (V)
HAL_PIN(sv);  // *output*, V phase voltage with offset, 0..udc when within range (V)
HAL_PIN(sw);  // *output*, W phase voltage with offset, 0..udc when within range (V)

//commutation mode
HAL_PIN(cmode);  // *parameter*, Commutation mode, 0 = sine, 1 = block

//modulation mode
HAL_PIN(mode);  // *parameter*, Modulation, 0 = sine, 1 = space vector, 2 = flat bottom, 3 = flat top, default 1

//half bridge enable out
HAL_PIN(enu);  // *output*, U half bridge enable, 0 in block mode when u is the middle phase
HAL_PIN(env);  // *output*, V half bridge enable, 0 in block mode when v is the middle phase
HAL_PIN(enw);  // *output*, W half bridge enable, 0 in block mode when w is the middle phase

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct svm_ctx_t * ctx = (struct svm_ctx_t *)ctx_ptr;
  struct svm_pin_ctx_t *pins = (struct svm_pin_ctx_t *)pin_ptr;

  PIN(mode) = 1.0;
  PIN(enu)  = 1.0;
  PIN(env)  = 1.0;
  PIN(enw)  = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct svm_ctx_t * ctx = (struct svm_ctx_t *)ctx_ptr;
  struct svm_pin_ctx_t *pins = (struct svm_pin_ctx_t *)pin_ptr;

  float offset = 0;
  float udc    = PIN(udc);

  float u = PIN(u);
  float v = PIN(v);
  float w = PIN(w);

  switch((int)PIN(mode)) {
    default:
    case 0:  // sine modulation
      offset = (u + v + w) / 3.0 - udc / 2.0;
      break;

    case 1:  // space vector modulation
      offset = (MIN3(u, v, w) + MAX3(u, v, w)) / 2.0 - udc / 2.0;
      break;

    case 2:  // flat bottom space vector modulation
      offset = MIN3(u, v, w);
      break;

    case 3:  // flat top space vector modulation
      offset = MAX3(u, v, w) - udc;
      break;
  }

  PIN(enu) = 1.0;
  PIN(env) = 1.0;
  PIN(enw) = 1.0;

  switch((int)PIN(cmode)) {
    case 1:  // block
      // the middle phase is off, in either order of the other two
      if((u > v && u < w) || (u < v && u > w)) {
        PIN(enu) = 0.0;
      }
      if((v > u && v < w) || (v < u && v > w)) {
        PIN(env) = 0.0;
      }
      if((w > u && w < v) || (w < u && w > v)) {
        PIN(enw) = 0.0;
      }
      break;

    default:  // sine
      break;
  }

  PIN(su) = u - offset;
  PIN(sv) = v - offset;
  PIN(sw) = w - offset;
}

hal_comp_t svm_comp_struct = {
    .name      = "svm",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = 0,
    .pin_count = sizeof(struct svm_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
