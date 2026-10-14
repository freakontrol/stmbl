#include "idx_home_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `idx_home` implements index homing for LinuxCNC via smart serial: when LinuxCNC requests an index, it switches the feedback from the incremental motor position to the absolute motor position at the next zero crossing. F4 component, loaded by `conf/template/sserial.txt`: `idx_home0.fb = fb_switch0.mot_fb_no_offset`, `idx_home0.fb_abs = fb_switch0.mot_abs_fb_no_offset`, `idx_home0.index_en = sserial0.index_out`, `sserial0.index_clear = idx_home0.index_clear`, `linrev0.fb_in = idx_home0.pos_out`.
*
* ## Component Explanation
* All work is done in `rt`. There is no `nrt_init`.
*
* 1. **Index detection**:
* - While `index_en` > 0 and `mot_state` is 3 (absolute), the quadrant of `fb_abs` is tracked. A crossing between quadrant 1 and 4 (through 0 rad) acts as the index: `index_clear` is set to 1 and the component latches into the homed state. Each new request (rising edge of `index_en`) starts the quadrant tracking fresh, so a quadrant left over from an earlier request does not count as a crossing.
* - If the index was requested while the feedback was not yet absolute, the index is accepted immediately once `mot_state` becomes 3. This only applies to that request; later ones wait for the zero crossing.
* - `index_clear` goes back to 0 when `index_en` is released.
*
* {{% hint warning %}}
* The homed state is never reset: after the first index `pos_out` stays `fb_abs`, later index requests only set `index_clear` again.
* {{% /hint %}}
*/

HAL_COMP(idx_home);

HAL_PIN(mot_state);    // *input*, Motor feedback state: 0 = disabled, 1 = inc, 2 = start abs, 3 = abs
HAL_PIN(fb);           // *input*, Incremental motor position (rad)
HAL_PIN(fb_abs);       // *input*, Absolute motor position (rad)
HAL_PIN(index_en);     // *input*, Index request from LinuxCNC
HAL_PIN(index_clear);  // *output*, 1 = index found
HAL_PIN(pos_out);      // *output*, fb before, fb_abs after the index (rad)

struct idx_home_ctx_t {
  int state;
  int lastq;
  int waitabs;
  int last_en;
};

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct idx_home_ctx_t *ctx      = (struct idx_home_ctx_t *)ctx_ptr;
  struct idx_home_pin_ctx_t *pins = (struct idx_home_pin_ctx_t *)pin_ptr;

  uint8_t q = 0;
  int en    = PIN(index_en) > 0;

  if(en && !ctx->last_en) {  // new request: nothing carried over from an old one
    ctx->lastq   = 0;
    ctx->waitabs = 0;
  }
  ctx->last_en = en;

  if(en) {
    if(PIN(mot_state) == 3) {
      q = quadrant(PIN(fb_abs));
      if(((q == 1 && ctx->lastq == 4) || (q == 4 && ctx->lastq == 1)) || ctx->waitabs == 1) {
        PIN(index_clear) = 1;
        ctx->state       = 1;
        ctx->waitabs     = 0;  // only this request; later ones wait for the zero crossing
      }
      ctx->lastq = q;
    } else {  //index requested, but index not seen
      ctx->waitabs = 1;
    }
  }

  if(!(PIN(index_en) > 0) && ctx->state == 1) {
    PIN(index_clear) = 0;
  }

  if(ctx->state == 1) {
    PIN(pos_out) = PIN(fb_abs);
  } else {
    PIN(pos_out) = PIN(fb);
  }
}

hal_comp_t idx_home_comp_struct = {
    .name      = "idx_home",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct idx_home_ctx_t),
    .pin_count = sizeof(struct idx_home_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
