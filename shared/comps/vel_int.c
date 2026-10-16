#include "vel_int_comp.h"
#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"

/**
* ## Brief
* `vel_int` interpolates a position command that arrives at a lower or irregular rate (e.g. from LinuxCNC via smart serial) by integrating the commanded velocity between updates. F4 component, loaded by `conf/template/sserial.txt`: `vel_int0.pos_in = linrev0.cmd_out`, `vel_int0.vel_in = linrev0.cmd_d_out`, `rev0.in = vel_int0.pos_out`, `rev0.in_d = vel_int0.vel_out` (to `pid0.vel_ext_cmd`) and `rev0.in_dd = vel_int0.acc_out` (to `pid0.acc_ext_cmd`), with `vel_int0.max_acc = conf0.max_acc`. It also gives the acceleration feedforward on the sserial path, where `vel0` is off and `pid.txt`'s `vel0.acc` link is 0.
*
* ## Component Explanation
* All work is done in `rt`.
*
* 1. **Interpolation**:
* - When `pos_in` changes (detected per instance against the last value), the internal position is set to `pos_in` and the watchdog counter is reset.
* - Otherwise the internal position is advanced by the velocity times `period`. `pos_out` is wrapped with `mod()`, `vel_out` is that velocity: `vel_in`, zeroed by the watchdog, plus the extrapolation of section 4 with `vel_ip` set.
*
* 2. **Watchdog**:
* - If `pos_in` has not changed for more than `wd` seconds while `vel_in` is not 0, the velocity is forced to 0 (no more extrapolation) and `error` is set to 1. Otherwise `error` is 0.
*
* 3. **Command rate**:
* - `real_cmd_freq` estimates how often `pos_in` changes (Hz): every change adds 1 to a counter that decays with a 1 s time constant; the result is additionally low pass filtered (factor 0.001 per period).
*
* 4. **Frame acceleration**:
* - On every change of `pos_in` (a host frame) the acceleration is the change of `vel_in` since the previous frame times `f / n`, with `f = max(cmd_freq, 100)` the nominal frame rate (not `real_cmd_freq`, which sags at standstill) and `n` the number of frames since the last one, rounded and at least 1, so a dropped frame does not double the step.
* - While the watchdog holds (`error`), or no frame came for more than `wd`, the acceleration is 0 and the next frame starts a fresh difference instead of one across the gap.
*
* 5. **Velocity extrapolation** (`vel_ip`):
* - `vel_in` is the host's velocity at the frame; held flat it runs half a frame late against `pos_out`. With `vel_ip` set, `vel_out` (and the position integration) gets `acc * min(t, 1 / f)` added, `t` the time since the last frame, so a late frame does not run away. Use it for a position axis on sserial; `conf/fanuc_a3-3000.txt` sets it together with `vel1.vel_p = 1`. On Y the drive ran about 38 um ahead of LinuxCNC through every acceleration without it; in the sim (50 mm at 750 mm/s2) the steady error went from -32.4 to 5.0 um.
*
* 6. **Frame clock** (`sync_k`):
* - A frame is seen at the first rt tick after it is parsed, so host jitter of a few tens of us moves it a whole rt tick (0.2 ms) early or late now and then (Y, 15 Oct: 9-17 % of frames at 1 kHz). Snapping `pos` to `pos_in` at that tick puts a sawtooth of `vel_in` x the timing error on the command (10-20 um at 200 mm/s on Y), which the loop then chases.
* - With `sync_k` < 1 the frames are placed on a smoothed clock: each frame's nominal instant is the previous one plus the estimated frame period `T`; the arrival's offset `e` from it moves the nominal instant by `sync_k` x `e` and `T` by `sync_k`^2 / 4 x `e`. At the frame `pos` = `pos_in` + `vel_in` x the time since its nominal instant, so a late frame continues the straight line instead of jumping back. The mean offset is 0: a constant latency is not added. `sync_k` = 1 (default) is the plain snap of section 1. An offset beyond half a period (dropped or doubled frame, startup) restarts the clock at the frame.
*
* 7. **Acceleration feedforward**:
* - `acc_out` = the frame acceleration x `acc_k`, clamped to +-`max_acc` when `max_acc > 0`, held until the next frame. In the sim `y_move` the peak `pos_error` went from 0.0467 to 0.0144 rad with it.
*
* {{% hint warning %}}
* The `cmd_freq` pin seeds the rate estimate at init and is the nominal frame rate of sections 4 and 5: it should match the host's frame rate.
* {{% /hint %}}
*/

HAL_COMP(vel_int);

HAL_PIN(pos_in);   // *input*, Position command, sampled at the command rate (rad)
HAL_PIN(pos_out);  // *output*, Interpolated position command (rad, +-pi)

HAL_PIN(vel_in);   // *input*, Velocity command used for interpolation (rad/s)
HAL_PIN(vel_out);  // *output*, Velocity command, 0 after watchdog timeout, extrapolated with vel_ip (rad/s)

HAL_PIN(cmd_freq);       // *parameter*, Nominal host frame rate (Hz), seeds real_cmd_freq and scales acc_out, at least 100 is used, default 1000
HAL_PIN(real_cmd_freq);  // *output*, Measured rate of pos_in updates (Hz)

HAL_PIN(wd);       // *parameter*, Watchdog time without pos_in change (s), default 0.002
HAL_PIN(error);    // *output*, 1 = watchdog timed out while vel_in != 0
HAL_PIN(vel_ip);   // *parameter*, 1 = extrapolate vel_out through the frame by the change of vel_in, 0 = hold (default)
HAL_PIN(acc_out);  // *output*, Acceleration feedforward (rad/s^2), change of vel_in per frame, 0 while the watchdog holds
HAL_PIN(acc_k);    // *parameter*, Scale of acc_out, 1 = as commanded (default), 0 = off
HAL_PIN(max_acc);  // *parameter*, Clamp of acc_out (rad/s^2), 0 = none (default)
HAL_PIN(sync_k);   // *parameter*, Frame clock gain 0..1: 1 = snap to each frame (default), 0.05 = smooth host jitter (section 6)
HAL_PIN(sync_err); // *output*, Last frame's offset from the frame clock (s)

struct vel_int_ctx_t {
  float pos;
  float counter;
  float cmd_freq;
  float last_pos_in;
  float v_edge;     // vel_in at the last frame
  float acc;        // change of vel_in per second over the last frame
  float since;      // time since the last frame [s]
  uint32_t have_v;  // v_edge is from a frame inside the watchdog window
  float tn;         // time since the nominal instant of the last frame [s]
  float t_frame;    // estimated frame period [s]
};

static void nrt_init(void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct vel_int_ctx_t *ctx      = (struct vel_int_ctx_t *)ctx_ptr;
  struct vel_int_pin_ctx_t *pins = (struct vel_int_pin_ctx_t *)pin_ptr;

  ctx->pos     = 0.0;
  ctx->counter = 0.0;
  ctx->last_pos_in = 0.0;
  PIN(pos_in)  = 0.0;
  PIN(pos_out) = 0.0;

  PIN(vel_in)  = 0.0;
  PIN(vel_out) = 0.0;

  PIN(wd)    = 0.002;
  PIN(error) = 0.0;

  PIN(cmd_freq) = 1000.0;
  ctx->cmd_freq = PIN(cmd_freq);
  PIN(vel_ip)   = 0.0;
  PIN(acc_out)  = 0.0;
  PIN(acc_k)    = 1.0;
  PIN(max_acc)  = 0.0;
  ctx->v_edge   = 0.0;
  ctx->acc      = 0.0;
  ctx->since    = 0.0;
  ctx->have_v   = 0;
  PIN(sync_k)   = 1.0;
  PIN(sync_err) = 0.0;
  ctx->tn       = 0.0;
  ctx->t_frame  = 0.0;
}

static void rt_func(float period, void *ctx_ptr, hal_pin_inst_t *pin_ptr) {
  struct vel_int_ctx_t *ctx      = (struct vel_int_ctx_t *)ctx_ptr;
  struct vel_int_pin_ctx_t *pins = (struct vel_int_pin_ctx_t *)pin_ptr;

  float p = PIN(pos_in);
  float v = PIN(vel_in);

  if(ctx->counter > PIN(wd) && v != 0.0) {
    v          = 0;
    PIN(error) = 1.0;
  } else {
    PIN(error) = 0.0;
    ctx->counter += period;
  }

  // per instance: EDGE() keeps one static for every instance
  int edge         = p != ctx->last_pos_in;
  ctx->last_pos_in = p;

  ctx->since += period;
  ctx->tn += period;
  if(edge) {
    ctx->counter = 0.0;
    // frame clock (section 6): offset of this frame from its nominal instant
    float t0 = 1.0 / MAX(PIN(cmd_freq), 100.0);
    float k  = CLAMP(PIN(sync_k), 0.0, 1.0);
    if(ctx->t_frame <= 0.0) {
      ctx->t_frame = t0;
    }
    float e = ctx->tn - ctx->t_frame;
    if(k >= 1.0 || ctx->since > PIN(wd) || ABS(e) > 0.5 * ctx->t_frame) {
      e            = 0.0;  // snap, or restart the clock at this frame
      ctx->tn      = 0.0;
      ctx->t_frame = t0;
    } else {
      ctx->tn = e * (1.0 - k);
      ctx->t_frame = CLAMP(ctx->t_frame + e * k * k * 0.25, 0.9 * t0, 1.1 * t0);
    }
    PIN(sync_err) = e;
    ctx->pos      = p + PIN(vel_in) * ctx->tn;
    ctx->cmd_freq += 1.0;
    // nominal host frame rate (real_cmd_freq sags at standstill); frames
    // since the last one, rounded, at least 1, so a dropped frame does not
    // double the step
    float f     = MAX(PIN(cmd_freq), 100.0);
    float n     = MAX(floorf(ctx->since * f + 0.5), 1.0);
    ctx->acc    = ctx->have_v ? (PIN(vel_in) - ctx->v_edge) * f / n : 0.0;
    ctx->v_edge = PIN(vel_in);
    ctx->since  = 0.0;
    ctx->have_v = 1;
  }
  // no frames: velocity is forced to 0 above, and the next frame starts a
  // fresh difference instead of one across the gap
  if(PIN(error) > 0.0 || ctx->since > PIN(wd)) {
    ctx->acc    = 0.0;
    ctx->have_v = 0;
  }
  if(PIN(vel_ip) > 0.0) {
    // extrapolate at most one frame, so a late frame does not run away
    v += ctx->acc * MIN(ctx->since, 1.0 / MAX(PIN(cmd_freq), 100.0));
  }
  if(!edge) {
    ctx->pos += v * period;
  }

  ctx->cmd_freq -= ctx->cmd_freq * period;

  PIN(real_cmd_freq) = ctx->cmd_freq * 0.001 + PIN(real_cmd_freq) * 0.999;

  ctx->pos = mod(ctx->pos);

  float acc = ctx->acc * PIN(acc_k);
  if(PIN(max_acc) > 0.0) {
    acc = CLAMP(acc, -PIN(max_acc), PIN(max_acc));
  }
  PIN(acc_out) = acc;

  PIN(pos_out) = ctx->pos;
  PIN(vel_out) = v;
}

hal_comp_t vel_int_comp_struct = {
    .name      = "vel_int",
    .nrt       = 0,
    .rt        = rt_func,
    .frt       = 0,
    .nrt_init  = nrt_init,
    .rt_start  = 0,
    .frt_start = 0,
    .rt_stop   = 0,
    .frt_stop  = 0,
    .ctx_size  = sizeof(struct vel_int_ctx_t),
    .pin_count = sizeof(struct vel_int_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
