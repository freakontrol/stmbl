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
HAL_PIN(abs_neg);    // 1: fb_in counts against the source (link conf0.mot_fb_rev)

HAL_PIN(pos_offset);

struct linrev_ctx_t {
  int lastq;             //last quadrant
  int32_t rev;           //current multiturn
  int32_t last_abs_rev;  //abs_rev (sign applied) of the last absolute tick
  int last_abs_ok;       //last tick was absolute
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

  // rev counts the wraps of fb_in through +-pi.
  if(q == 3 && ctx->lastq == 2) {
    ctx->rev++;
  }
  if(q == 2 && ctx->lastq == 3) {
    ctx->rev--;
  }
  ctx->lastq = q;

  // Absolute (abs_en, abs_state 3): rev is taken from the source's turn
  // count once, when the source becomes absolute or its count jumps by more
  // than one turn (an encoder re-referencing at its index), and counted from
  // fb_in's wraps after that. The take aligns rev to fb_in: the whole number
  // of turns that puts fb_in + rev nearest the source's position abs_rev +
  // abs_pos, so fb_in may lag the source by a tick or two (in the sserial
  // template it comes through fb_switch and idx_home, which run after
  // linrev). Taken every tick instead, rev flipped by a turn while fb_in
  // and abs_pos differed by about half a turn (abs_pos = encf0.pos with
  // pos_offset near 32768 against fb_in = encf0.abs_pos after index
  // homing), and with abs_neg it was wrong outright. abs_rev must step
  // where abs_pos wraps at +-pi (encf does). Link abs_state: left at its
  // default 3, the take happens on the first tick, before the source may
  // have a valid count. rev_clear (index homing) zeroes rev for good.
  int abs_ok = PIN(abs_en) > 0 && PIN(abs_state) == 3.0;
  if(abs_ok) {
    int32_t ar = (int32_t)PIN(abs_rev);
    float ap   = PIN(abs_pos);
    if(PIN(abs_neg) > 0) {
      // -(ap + 2 pi ar) = -ap + 2 pi (-ar), -ap still within +-pi
      ar = -ar;
      ap = -ap;
    }
    int32_t dr = ar - ctx->last_abs_rev;
    if(!ctx->last_abs_ok || dr > 1 || dr < -1) {
      float d  = ap - PIN(fb_in);
      ctx->rev = ar + (d > M_PI) - (d < -M_PI);
    }
    ctx->last_abs_rev = ar;
  }
  ctx->last_abs_ok = abs_ok;

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
