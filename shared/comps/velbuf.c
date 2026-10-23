#include "velbuf_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `velbuf` computes a velocity from a position signal by taking the difference over an adaptive time window from a ring buffer. This gives usable velocities from coarse or slowly changing positions. F4 component, see `conf/template/velbuf.txt` (velocity of the position command `rev0.out`, used as `pid0.vel_ext_cmd`). It needs both an `rt_prio` and an `frt_prio`: the frt part fills the buffer, without an `frt_prio` it never runs.
*
* ## Component Explanation
* 1. **Sampling (frt)**:
* - Every `frt` period `pos` is written into a ring buffer of 20 samples.
*
* 2. **Velocity (rt)**:
* - Starting from the oldest sample, the component searches for the newest sample that still differs from the latest position by more than `min_pos_diff`. The window is therefore as short as possible, but long enough to see at least `min_pos_diff` of movement, and at most 19 frt periods (the oldest and newest of the 20 samples).
* - `diff_pos = minus(pos_newest, pos_old)`, `diff_time` = window length (s), `vel = diff_pos / diff_time`, or 0 while `diff_time` is 0 (until the frt part has run).
* - `vel_lp` is `vel` low pass filtered at `lpf` Hz (rt period).
*/

HAL_COMP(velbuf);

HAL_PIN(pos);           // *input*, Position (rad)
HAL_PIN(vel);           // *output*, Velocity (rad/s)
HAL_PIN(vel_lp);        // *output*, Low pass filtered velocity (rad/s)
HAL_PIN(min_pos_diff);  // *parameter*, Minimum position change for the window (rad), default 0.01

HAL_PIN(lpf)  // *parameter*, Cutoff of the vel_lp filter (Hz), default 1000

HAL_PIN(diff_time);  // *output*, Length of the used window (s)
HAL_PIN(diff_pos);   // *output*, Position change over the window (rad)

#define velbuf_size 20

struct velbuf_ctx_t {
  float buf[velbuf_size];
  uint32_t ptr;
  float frt_period;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct velbuf_ctx_t *ctx      = (struct velbuf_ctx_t *)ctx_ptr;
  struct velbuf_pin_ctx_t *pins = (struct velbuf_pin_ctx_t *)pin_ptr;

  PIN(min_pos_diff) = 0.01;

  for(int i = 0; i < velbuf_size; i++) {
    ctx->buf[i] = 0;
  }

  PIN(lpf) = 1000.0;
}


static void frt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct velbuf_ctx_t *ctx      = (struct velbuf_ctx_t *)ctx_ptr;
  struct velbuf_pin_ctx_t *pins = (struct velbuf_pin_ctx_t *)pin_ptr;

  ctx->buf[ctx->ptr++] = PIN(pos);
  ctx->ptr %= velbuf_size;
  ctx->frt_period = period;
}


static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct velbuf_ctx_t *ctx      = (struct velbuf_ctx_t *)ctx_ptr;
  struct velbuf_pin_ctx_t *pins = (struct velbuf_pin_ctx_t *)pin_ptr;

  int p     = ctx->ptr;
  float pos = ctx->buf[(p - 1 + velbuf_size) % velbuf_size];

  float old_pos   = ctx->buf[p];
  float diff_time = ctx->frt_period * (velbuf_size - 1);

  float min_diff = PIN(min_pos_diff);

  for(int i = 0; i < velbuf_size; i++) {
    if(ABS(minus(pos, ctx->buf[(p + i) % velbuf_size])) > min_diff) {
      old_pos   = ctx->buf[(p + i) % velbuf_size];
      diff_time = (velbuf_size - i - 1) * ctx->frt_period;
    } else {
      break;
    }
  }

  float diff_pos = minus(pos, old_pos);
  PIN(diff_pos)  = diff_pos;
  PIN(diff_time) = diff_time;
  PIN(vel)       = diff_time > 0.0 ? diff_pos / diff_time : 0.0;  // 0 until the frt part has run

  PIN(vel_lp) = PIN(vel) * LP_HZ(PIN(lpf)) + PIN(vel_lp) * (1.0 - LP_HZ(PIN(lpf)));
}

hal_comp_t velbuf_comp_struct = {
    .name      = "velbuf",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = frt_func,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct velbuf_ctx_t),
    .pin_count = sizeof(struct velbuf_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};