#include "linrev_comp.h"
//Calculate motor angle from position in machine units

/*

      |
  Q2  |  Q1
      |
-------------
      |
  Q3  |  Q4
      |

*/


#include "hal.h"
#include "angle.h"
#include "defines.h"
#include <math.h>

HAL_COMP(linrev);

HAL_PIN(scale);

HAL_PIN(cmd_in);
HAL_PIN(cmd_out);

HAL_PIN(cmd_d_in);
HAL_PIN(cmd_d_out);

HAL_PIN(fb_in);
HAL_PIN(fb_out);
HAL_PIN(fb_d_in);
HAL_PIN(fb_d_out);

HAL_PIN(rev_clear);
HAL_PIN(rev);

HAL_PIN(abs_en);
HAL_PIN(abs_rev);
HAL_PIN(abs_pos);    // angle of the abs_rev source, same tick as abs_rev (e.g. encf0.pos)
HAL_PIN(abs_state);  // state of the abs_rev source, 3 = absolute (fb_switch mot_state convention)

HAL_PIN(pos_offset);

struct linrev_ctx_t {
  int lastq;    //last quadrant
  int32_t rev;  //current multiturn
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct linrev_ctx_t *ctx      = (struct linrev_ctx_t *)ctx_ptr;
  struct linrev_pin_ctx_t *pins = (struct linrev_pin_ctx_t *)pin_ptr;

  PIN(pos_offset) = 0;
  PIN(abs_state)  = 3;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct linrev_ctx_t *ctx      = (struct linrev_ctx_t *)ctx_ptr;
  struct linrev_pin_ctx_t *pins = (struct linrev_pin_ctx_t *)pin_ptr;

  float scale = PIN(scale);
  if(ABS(scale) > 0.01) {
    PIN(cmd_out)   = mod((PIN(cmd_in) / scale) * 2.0 * M_PI);
    PIN(cmd_d_out) = PIN(cmd_d_in) / scale * 2.0 * M_PI;
  }

  int q = quadrant(PIN(fb_in));

  // Incremental: count the wraps of fb_in through +-pi.
  // Absolute (abs_en, abs_state 3): rev is the source's turn count, taken
  // every tick. fb_in can lag abs_rev by a tick or two (in the sserial
  // template fb_in comes through fb_switch and idx_home, which run after
  // linrev), so rev is aligned to fb_in: the whole number of turns that
  // puts fb_in + rev nearest the source's position abs_rev + abs_pos. That
  // is exact while the lag is under half a turn of motion, and follows an
  // encoder re-referencing at its index at once. abs_rev must step where
  // abs_pos wraps at +-pi (encf does). With abs_pos unlinked (0) this is
  // rev = abs_rev, fb_in = -pi included.
  if(PIN(abs_en) > 0 && PIN(abs_state) == 3.0) {
    float d   = PIN(abs_pos) - PIN(fb_in);
    ctx->rev = (int32_t)PIN(abs_rev) + (d > M_PI) - (d < -M_PI);
  } else {
    if(q == 3 && ctx->lastq == 2) {
      ctx->rev++;
    }
    if(q == 2 && ctx->lastq == 3) {
      ctx->rev--;
    }
  }

  ctx->lastq = q;

  if(PIN(rev_clear) > 0) {
    ctx->rev = 0;
  }
  PIN(rev)      = ctx->rev;
  PIN(fb_out)   = ((PIN(fb_in) + ctx->rev * M_PI * 2.0) * scale) / (2.0 * M_PI) + PIN(pos_offset) * scale;
  PIN(fb_d_out) = (PIN(fb_d_in) * scale) / (2.0 * M_PI);
}

const hal_comp_t linrev_comp_struct = {
    .name      = "linrev",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .hw_init   = 0,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct linrev_ctx_t),
    .pin_count = sizeof(struct linrev_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
