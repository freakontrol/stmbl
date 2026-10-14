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

/**
* ## Brief
* `linrev` converts between machine units (e.g. mm from LinuxCNC) and the motor angle, and counts motor revolutions to turn the wrapped motor angle back into an unwrapped machine position. With a multiturn absolute encoder it can take the revolution count from the encoder instead. F4 component, loaded by `conf/template/sserial.txt`: `linrev0.cmd_in = sserial0.pos_cmd`, `linrev0.fb_in = idx_home0.pos_out`, `sserial0.pos_fb = linrev0.fb_out`, `linrev0.scale = sserial0.scale`, `linrev0.rev_clear = sserial0.index_clear`, `linrev0.abs_neg = conf0.mot_fb_rev`.
*
* ## Component Explanation
* All work is done in `rt`. In the sserial template `idx_home0` (rt_prio 2.05) runs before `linrev0` (2.1) and `sserial0`'s rt (2.3, its frt at 2.0) after it, so `fb_in` is from this tick and `fb_out` goes out in the same tick; `fb_switch0` (rt_prio 5) runs after `linrev0`, so its output reaches `fb_in` one tick late.
*
* 1. **Command path**:
* - If `|scale|` > 0.01: `cmd_out = mod(cmd_in / scale * 2pi)` and `cmd_d_out = cmd_d_in / scale * 2pi`. Otherwise the outputs keep their last value.
*
* 2. **Revolution counting**:
* - The quadrant of `fb_in` is tracked every tick. A step from quadrant 2 to 3 (crossing +pi to -pi) increments the revolution counter, a step from 3 to 2 decrements it. This runs in both modes.
*
* 3. **Absolute multiturn** (`abs_en` > 0 and `abs_state` = 3):
* - The counter is taken from the source's turn count once: when the source becomes absolute, or when its count jumps by more than one turn (an encoder re-referencing at its index). After that it counts `fb_in`'s wraps as in 2. The take aligns it to `fb_in`:
* ```c
* d   = abs_pos - fb_in;
* rev = abs_rev + (d > pi) - (d < -pi);
* ```
* - This is the whole number of turns that puts `fb_in + rev * 2pi` nearest the source position `abs_pos + abs_rev * 2pi`, so `fb_in` may lag the source by a tick or two. `abs_rev` must step exactly where `abs_pos` wraps at +-pi (`encf` does). With `abs_pos` unlinked (0) this is `rev = abs_rev`.
* - `abs_neg` > 0 negates `abs_rev` and `abs_pos` first, for an `fb_in` that counts against the source (link `conf0.mot_fb_rev`).
* - Link `abs_state`: left at its default 3, the take happens on the first tick, before the source may have a valid count. No shipped config enables it.
*
* 4. **Clear and output**:
* - `rev_clear` > 0 sets the counter to 0 for good (typically linked to `sserial0.index_clear` for index homing); it is not taken from `abs_rev` again until the source's count jumps.
* ```c
* fb_out   = (fb_in + rev * 2pi) * scale / 2pi + pos_offset * scale;
* fb_d_out = fb_d_in * scale / 2pi;
* ```
*/

HAL_COMP(linrev);

HAL_PIN(scale);  // *parameter*, Machine units per motor revolution

HAL_PIN(cmd_in);   // *input*, Position command (machine units)
HAL_PIN(cmd_out);  // *output*, Motor angle command (rad, +-pi)

HAL_PIN(cmd_d_in);   // *input*, Velocity command (machine units/s)
HAL_PIN(cmd_d_out);  // *output*, Motor velocity command (rad/s)

HAL_PIN(fb_in);     // *input*, Motor angle feedback (rad, +-pi)
HAL_PIN(fb_out);    // *output*, Position feedback (machine units)
HAL_PIN(fb_d_in);   // *input*, Motor velocity feedback (rad/s)
HAL_PIN(fb_d_out);  // *output*, Velocity feedback (machine units/s)

HAL_PIN(rev_clear);  // *input*, Clear the revolution counter if > 0
HAL_PIN(rev);        // *output*, Revolution counter

HAL_PIN(abs_en);     // *parameter*, Take the revolution count from abs_rev if > 0
HAL_PIN(abs_rev);    // *input*, Turn count of a multiturn absolute encoder (e.g. encf0.turns)
HAL_PIN(abs_pos);    // *input*, Angle of the abs_rev source on the same tick (rad, +-pi, e.g. encf0.pos)
HAL_PIN(abs_state);  // *input*, State of the abs_rev source, 3 = absolute (e.g. encf0.state), default 3
HAL_PIN(abs_neg);    // *parameter*, 1 = fb_in counts against the source (link conf0.mot_fb_rev)

HAL_PIN(pos_offset);  // *parameter*, Feedback offset (motor revolutions), default 0

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
  // linrev). Taken every tick instead, rev would flip by a turn while fb_in
  // and abs_pos differ by about half a turn (abs_pos = encf0.pos with
  // pos_offset near 32768 against fb_in = encf0.abs_pos after index
  // homing), and with abs_neg it would be wrong outright. abs_rev must step
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
