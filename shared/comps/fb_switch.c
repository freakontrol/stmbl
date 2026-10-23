#include "fb_switch_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `fb_switch` combines up to three feedback sources (motor, commutation and joint feedback) into the position feedback for the position loop, the position used for velocity estimation and the commutation angle for the current loop. If no absolute source is available it performs autophasing. F4 component, loaded by the `pid`/`mpid` templates; e.g. `fb_switch0.mot_pos = enc_fb0.ipos`, `fb_switch0.com_state = uvw0.state`, `pid0.pos_fb = fb_switch0.pos_fb`, `vel1.pos_in = fb_switch0.vel_fb`, `fault0.fb_ready = fb_switch0.state`, `hv0.d_cmd = fb_switch0.id`.
*
* ## Component Explanation
* All work is done in `rt`. The `*_state` inputs use the feedback state convention: 0 = disabled/not ready, 1 = incremental, 2 = start absolute (absolute position known, but not yet precise), 3 = absolute.
*
* 1. **Direction and raw outputs**:
* - If `mot_rev`, `com_rev` or `joint_rev` > 0, the position, absolute position and offset of that source are negated.
* - Link other components to `mot_state_fb` rather than to the input `mot_state`: a link typed at runtime to an input pin that is itself linked reads that pin's own value (usually 0), not its source. `id_pmsm` uses it to warn when the motor feedback is not absolute.
*
* 2. **Position and velocity feedback**:
* - `pos_fb` is taken from the first source in state 1 or 3 in the order joint, mot, com, plus a command offset, wrapped with `mod()`.
* - `vel_fb` is `mot_pos` (or `com_pos` if the motor feedback is not ready), without offset; it is meant to be differentiated by a `vel` instance.
* - While disabled (`en` <= 0) and `mot_state` > 0, the command offsets are set to `minus(cmd_pos, source_pos)` so that `pos_fb` equals `cmd_pos` and enabling causes no jump. With `track_fb` > 0 this is only done once after boot (via `offset_first_enable`), so the feedback keeps its absolute reference and the command has to follow the feedback.
*
* 3. **Commutation source selection** (while `en` > 0):
* - `current_com_pos` is reset to 10 on disable.
* - A better source is taken whenever its state reaches 2 or 3, even while enabled (priority mot > com > joint). E.g. hall sensors (`uvw`) are used first and the motor encoder takes over once it is absolute.
* - On switching to com or joint, an offset between that source and the motor position is stored. While that source is in state 2, `com_fb` is computed from `mot_pos` plus this offset (tracking); in state 3 directly from the absolute position:
* ```c
* com_fb = mod((com_abs_pos + com_offset) * polecount / com_polecount);
* com_fb = mod(mot_pos * polecount / mot_polecount + offset);  // tracking
* ```
* - With the motor source, `state` stays 0 until `mot_state` is 3.
* - Encoders with a commutation track (e.g. `encf`, see the `fanuc_fb0` template: `com_abs_pos = encf0.com_pos`, `com_state` = 3, `mot_state = encf0.state`) commutate from `com_abs_pos + com_offset` while `mot_state` is below 2, e.g. after a battery loss before the first index crossing, and switch to the motor feedback once it becomes absolute. `com_offset` is linked to `conf0.com_fb_offset` in the `pid`/`mpid` templates; `id_pmsm` measures it.
*
* 4. **Autophasing** (no source in state >= 2 when enabled):
* - The start position is stored and `com_fb` is held at that angle while `id` ramps from 0 to `phase_cur` within `phase_time / 4`.
* - The offset is integrated with `phase_gain` from the rotor movement, pulling the field so that the rotor returns to its start position.
* - After `phase_time` `current_com_pos` becomes 4 and `com_fb = mod(mot_pos * polecount / mot_polecount + offset)`. `offset` shows the result. The motor must be free to move a little during phasing.
* - `state` is 0 during phasing and 1 afterwards.
*
* {{% hint warning %}}
* The `joint_fb` output is never written. For joint commutation both the offset and the absolute path (joint state 3) use `polecount / mot_joint_ratio`, so `mot_joint_ratio` must not be 0 (division by zero).
* {{% /hint %}}
*/

HAL_COMP(fb_switch);

HAL_PIN(polecount);            // *parameter*, Motor pole pairs
HAL_PIN(track_fb);             // *parameter*, > 0: align pos_fb to cmd_pos only once after boot
HAL_PIN(offset_first_enable);  // *input/output*, 1 until the first command offset was computed, default 1

HAL_PIN(pos_fb);    // *output*, Position feedback (rad, +-pi)
HAL_PIN(vel_fb);    // *output*, Position for velocity estimation, no offset (rad)
HAL_PIN(com_fb);    // *output*, Commutation angle (electrical rad, +-pi)
HAL_PIN(joint_fb);  // *output*, Never written by the component (unused)
HAL_PIN(state);     // *output*, 0 = disabled or not ready, 1 = enabled and commutation ready

HAL_PIN(cmd_pos);  // *input*, Position command, used to align pos_fb while disabled (rad)

HAL_PIN(mot_pos);               // *input*, Motor feedback position (rad)
HAL_PIN(mot_abs_pos);           // *input*, Motor feedback absolute position (rad)
HAL_PIN(mot_polecount);         // *parameter*, Pole pairs of the motor feedback
HAL_PIN(mot_offset);            // *parameter*, Commutation offset of the motor feedback (rad)
HAL_PIN(mot_state);             // *input*, 0 = disabled, 1 = inc, 2 = start abs, 3 = abs
HAL_PIN(mot_state_fb);          // *output*, Copy of mot_state for other components to link to (e.g. idpmsm0.mot_state)
HAL_PIN(mot_rev);               // *parameter*, Reverse the motor feedback if > 0
HAL_PIN(mot_fb_no_offset);      // *output*, Motor position after reversal (rad)
HAL_PIN(mot_abs_fb_no_offset);  // *output*, Motor absolute position after reversal (rad)

HAL_PIN(plot_fb_pos);  // *output*, Feedback position for plotting (rad)

HAL_PIN(com_pos);           // *input*, Commutation feedback position (rad)
HAL_PIN(com_abs_pos);       // *input*, Commutation feedback absolute position (rad)
HAL_PIN(com_polecount);     // *parameter*, Pole pairs of the commutation feedback
HAL_PIN(com_offset);        // *parameter*, Commutation offset of the com feedback (rad), linked to conf0.com_fb_offset
HAL_PIN(com_state);         // *input*, 0 = disabled, 1 = inc, 2 = start abs, 3 = abs
HAL_PIN(com_rev);           // *parameter*, Reverse the commutation feedback if > 0
HAL_PIN(com_fb_no_offset);  // *output*, Com position after reversal (rad)

HAL_PIN(joint_pos);           // *input*, Joint feedback position (rad)
HAL_PIN(joint_abs_pos);       // *input*, Joint feedback absolute position (rad)
HAL_PIN(joint_offset);        // *parameter*, Commutation offset of the joint feedback (rad)
HAL_PIN(joint_state);         // *input*, 0 = disabled, 1 = inc, 2 = start abs, 3 = abs
HAL_PIN(joint_rev);           // *parameter*, Reverse the joint feedback if > 0
HAL_PIN(joint_fb_no_offset);  // *output*, Joint position after reversal (rad)

HAL_PIN(mot_joint_ratio);  // *parameter*, Motor to joint ratio for joint commutation, default 1

HAL_PIN(phase_time);  // *parameter*, Autophasing time (s), min 0.1, default 1
HAL_PIN(phase_cur);   // *parameter*, Autophasing d current (A peak), default 1
HAL_PIN(phase_gain);  // *parameter*, Autophasing offset integrator gain, default 100
HAL_PIN(offset);      // *output*, Current commutation offset (electrical rad)
HAL_PIN(id);          // *output*, D current command during autophasing (A peak)

HAL_PIN(current_com_pos);  // *output*, Commutation source: 1 mot, 2 com, 3 joint, 4 autophased, 5 phasing, 10 none

HAL_PIN(en);  // *input*, Enable

struct fb_switch_ctx_t {
  int32_t current_com_pos;
  float phase_start_pos;
  float cmd_com_offset;
  float cmd_mot_offset;
  float cmd_joint_offset;
  float com_offset;
  float phase_timer;
  int32_t phase_state;
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct fb_switch_ctx_t *ctx      = (struct fb_switch_ctx_t *)ctx_ptr;
  struct fb_switch_pin_ctx_t *pins = (struct fb_switch_pin_ctx_t *)pin_ptr;

  ctx->current_com_pos     = 10;
  ctx->cmd_mot_offset      = 0.0;
  ctx->com_offset          = 0.0;
  PIN(phase_cur)           = 1.0;
  PIN(phase_time)          = 1.0;
  PIN(phase_gain)          = 100.0;
  PIN(offset_first_enable) = 1.0;
  PIN(mot_joint_ratio)     = 1.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct fb_switch_ctx_t *ctx      = (struct fb_switch_ctx_t *)ctx_ptr;
  struct fb_switch_pin_ctx_t *pins = (struct fb_switch_pin_ctx_t *)pin_ptr;

  float mot_pos   = PIN(mot_pos);
  float com_pos   = PIN(com_pos);
  float joint_pos = PIN(joint_pos);

  float mot_abs_pos   = PIN(mot_abs_pos);
  float com_abs_pos   = PIN(com_abs_pos);
  float joint_abs_pos = PIN(joint_abs_pos);

  float mot_offset   = PIN(mot_offset);
  float com_offset   = PIN(com_offset);
  float joint_offset = PIN(joint_offset);


  if(PIN(mot_rev) > 0.0) {
    mot_pos *= -1.0;
    mot_abs_pos *= -1.0;
    mot_offset *= -1.0;
  }

  if(PIN(com_rev) > 0.0) {
    com_pos *= -1.0;
    com_abs_pos *= -1.0;
    com_offset *= -1.0;
  }

  if(PIN(joint_rev) > 0.0) {
    joint_pos *= -1.0;
    joint_abs_pos *= -1.0;
    joint_offset *= -1.0;
  }

  PIN(com_fb_no_offset)     = com_pos;
  PIN(mot_fb_no_offset)     = mot_pos;
  PIN(mot_abs_fb_no_offset) = mot_abs_pos;
  PIN(mot_state_fb)         = PIN(mot_state);
  PIN(joint_fb_no_offset)   = joint_pos;

  PIN(id) = 0.0;

  if(PIN(joint_state) == 1.0 || PIN(joint_state) == 3.0) {
    PIN(pos_fb)      = mod(joint_pos + ctx->cmd_joint_offset);
    PIN(plot_fb_pos) = PIN(pos_fb);
  } else if(PIN(mot_state) == 1.0 || PIN(mot_state) == 3.0) {
    PIN(pos_fb)      = mod(mot_pos + ctx->cmd_mot_offset);
    PIN(plot_fb_pos) = PIN(pos_fb);
  } else if(PIN(com_state) == 1.0 || PIN(com_state) == 3.0) {
    PIN(pos_fb)      = mod(com_pos + ctx->cmd_com_offset);
    PIN(plot_fb_pos) = PIN(pos_fb);
  }

  if(PIN(mot_state) == 1.0 || PIN(mot_state) == 3.0) {
    PIN(vel_fb) = mot_pos;
  } else if(PIN(com_state) == 1.0 || PIN(com_state) == 3.0) {
    PIN(vel_fb) = com_pos;
  }

  if(PIN(en) <= 0.0) {
    PIN(state)           = 0.0;
    ctx->current_com_pos = 10;
    ctx->com_offset      = 0.0;
    if((PIN(track_fb) <= 0.0 || PIN(offset_first_enable) > 0) && PIN(mot_state) > 0.0) {
      PIN(offset_first_enable) = 0;
      ctx->cmd_com_offset      = minus(PIN(cmd_pos), com_pos);
      ctx->cmd_mot_offset      = minus(PIN(cmd_pos), mot_pos);
      ctx->cmd_joint_offset    = minus(PIN(cmd_pos), joint_pos);
    }
    PIN(plot_fb_pos) = mod(mot_pos);
  } else {
    PIN(state) = 1.0;
    if(PIN(joint_state) >= 2.0 && ctx->current_com_pos > 3.0) {
      ctx->current_com_pos = 3;  // joint fb absolute
      ctx->com_offset      = minus(mod((joint_abs_pos + joint_offset) * PIN(polecount) / PIN(mot_joint_ratio)), mod(mot_pos * PIN(polecount) / PIN(mot_polecount)));
    }
    if(PIN(com_state) >= 2.0 && ctx->current_com_pos > 2.0) {
      ctx->current_com_pos = 2;  // com fb absolute
      ctx->com_offset      = minus(mod((com_abs_pos + com_offset) * PIN(polecount) / PIN(com_polecount)), mod(mot_pos * PIN(polecount) / PIN(mot_polecount)));
    }
    if(PIN(mot_state) >= 2.0 && ctx->current_com_pos > 1.0) {
      ctx->current_com_pos = 1;  // mot fb absolute
      ctx->com_offset      = 0.0;
    }
    if(ctx->current_com_pos > 4.0) {  // autophasing
      switch(ctx->current_com_pos) {
        case 5:  // phasing
          ctx->phase_timer += period;
          PIN(id) = CLAMP(ctx->phase_timer / (MAX(PIN(phase_time), 0.1) / 4.0) * PIN(phase_cur), 0.0, PIN(phase_cur));  // ramp up id
          ctx->com_offset += minus(ctx->phase_start_pos, mot_pos) * PIN(phase_gain) * period;

          if(ctx->phase_timer >= MAX(PIN(phase_time), 0.1)) {  // end
            ctx->phase_timer = 0.0;
            //ctx->com_offset      = -mod(mot_pos * PIN(polecount) / PIN(mot_polecount));
            ctx->cmd_mot_offset  = minus(PIN(cmd_pos), mot_pos);
            ctx->current_com_pos = 4.0;
          }
          break;

        default:                           // init
          ctx->phase_start_pos = mot_pos;  // safe start pos
          PIN(com_fb)          = 0.0;
          ctx->com_offset      = 0.0;
          ctx->current_com_pos = 5.0;
      }
    }

    switch(ctx->current_com_pos) {
      case 4:  // autophasing + mot fb -> com fb
        PIN(com_fb) = mod(mot_pos * PIN(polecount) / PIN(mot_polecount) + ctx->com_offset);
        break;

      case 3:  // joint fb -> com fb
        if(PIN(joint_state) != 3.0) {
          PIN(com_fb) = mod(mot_pos * PIN(polecount) / PIN(mot_polecount) + ctx->com_offset);  // tracking
        } else {
          PIN(com_fb) = mod((joint_abs_pos + joint_offset) * PIN(polecount) / PIN(mot_joint_ratio));
        }
        break;

      case 2:  // com fb -> com fb
        if(PIN(com_state) != 3.0) {
          PIN(com_fb) = mod(mot_pos * PIN(polecount) / PIN(mot_polecount) + ctx->com_offset);  // tracking
        } else {
          PIN(com_fb) = mod((com_abs_pos + com_offset) * PIN(polecount) / PIN(com_polecount));
        }
        break;

      case 1:  // mot fb -> com fb
        if(PIN(mot_state) != 3.0) {
          PIN(state) = 0.0;
        } else {
          PIN(com_fb) = mod((mot_abs_pos + mot_offset) * PIN(polecount) / PIN(mot_polecount));
        }
        break;

      default:
        PIN(state)  = 0.0;
        PIN(com_fb) = mod(ctx->phase_start_pos * PIN(polecount) / PIN(mot_polecount) + ctx->com_offset);
    }
  }
  PIN(current_com_pos) = ctx->current_com_pos;
  PIN(offset)          = ctx->com_offset;
}

hal_comp_t fb_switch_comp_struct = {
    .name      = "fb_switch",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct fb_switch_ctx_t),
    .pin_count = sizeof(struct fb_switch_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
