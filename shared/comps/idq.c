#include "idq_comp.h"
#include "commands.h"
#include "common.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

HAL_COMP(idq);

HAL_PIN(mode);

//d,q inputs
HAL_PIN(d);
HAL_PIN(q);

//rotor position
HAL_PIN(pos);
HAL_PIN(polecount);

// sin/cos of the same angle from elsewhere (dq0 on the f3, which transforms
// the currents with it a moment earlier), used instead of a second
// sincos_fast when ext_sc > 0
HAL_PIN(si);
HAL_PIN(co);
HAL_PIN(ext_sc);

//a,b output
HAL_PIN(a);
HAL_PIN(b);

//U V W output
HAL_PIN(u);
HAL_PIN(v);
HAL_PIN(w);

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
