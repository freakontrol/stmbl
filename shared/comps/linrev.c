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

HAL_PIN(pos_offset);

static uint32_t abs_state_counter;

struct linrev_ctx_t {
  int lastq;    //last quadrant
  int32_t rev;  //current multiturn
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  // struct linrev_ctx_t *ctx      = (struct linrev_ctx_t *)ctx_ptr;
  struct linrev_pin_ctx_t *pins = (struct linrev_pin_ctx_t *)pin_ptr;

  abs_state_counter = 0;
  PIN(pos_offset) = 0;
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

  if (PIN(abs_en) > 0) {
    if (q != 0 && q == ctx->lastq && abs_state_counter != 1) {
      ctx->rev = PIN(abs_rev);
      abs_state_counter = 1;
    }
  }

  if(q != 0 && q == 3 && ctx->lastq == 2) {
    if(PIN(abs_en) > 0){
      ctx->rev = PIN(abs_rev);
    } else {
      ctx->rev++;
    }
  }

  if(q != 0 && q == 2 && ctx->lastq == 3) {
    if(PIN(abs_en) > 0){
      ctx->rev = PIN(abs_rev);
    } else {
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
